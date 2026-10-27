"""Zuruecklegen nach jeder Episode der Aufnahme (AP 2.2/2.4).

Wunsch Anwender 2026-10-23: Eine Objektlage einmal teachen und die Bahn
mehrmals mit neuem Rauschen aufzeichnen. Nach JEDER Episode -- auch der
letzten -- legt der Arm das Objekt am geteachten PICK wieder ab und faehrt
an den Start, OHNE Aufzeichnung und OHNE Rauschen. Bei der Aufnahme wird
das Objekt immer wieder gebraucht; im Greifer bleibt es nur bei der
Inferenz (Uebergabe an PRE_PLACE ans Hauptprogramm, apps/infer.py).

Begruendung (Abdeckungsrechnung 2026-10-22 auf ``data_vm/2026-10-14/
ep_00004``): Der Abstand einer neuen Abweichung zum naechsten
Trainingszustand sinkt mit K Wiederholungen auf 73 % (K=2), 60 % (K=3),
53 % (K=4) -- danach kaum noch. Die Labels sind in jeder Wiederholung
dieselbe Idealbahn, neu ist nur die Observation: mehr Korrekturbeispiele
um dieselbe Bahn. Eine neue Objektlage ersetzen Wiederholungen NICHT --
die Lage im Bild bleibt identisch. Und im Trichter (Start, +-2 cm um PICK,
Ende; 22 % der Frames) sind Wiederholungen nahezu Duplikate.

Die Rueckfahrt ist die Vorwaertsbahn RUECKWAERTS: dieselben Punkte und
Bewegungsarten (das Segment B->A faehrt wie A->B), damit nur Geometrie
gefahren wird, die schon geplant und kollisionsgeprueft war. Abweichend:

* kein Ueberschleifen -- der Arm haelt an jedem Punkt. Die Fahrt wird nicht
  aufgezeichnet, Tempo ist egal; vor dem Absetzen soll der Abstieg exakt
  senkrecht beginnen (siehe ``bc/sequence.py`` zum Radius an PRE_GRASP).
* das Segment hinunter zum Greifpunkt faehrt mit Endanflug-Tempo
  (``approach``), auch wenn es vorwaerts das Anheben war.
* am Greifpunkt oeffnet der Greifer (Dwell wie beim Greifen).

Der Ablauf muss dafuer genau EINEN Greiferwechsel haben (auf -> zu) und
geschlossen enden -- so wie ``sequences/pick_to_station.json``.
"""

import numpy as np

from .kinematics import IKFailure
from . import config
from .noise import TrajectoryPlan, joint_speed
from .trajectory import Waypoint, build_ideal_trajectory


class PutBackError(ValueError):
    """Ablauf erlaubt kein Zuruecklegen, oder die Rueckfahrt ist nicht planbar."""


def grasp_index(waypoints):
    """Index des Greifpunkts; prueft, dass der Ablauf zuruecklegbar ist."""
    closed = [bool(wp.gripper_closed) for wp in waypoints]
    if closed[0] or not closed[-1]:
        raise PutBackError(
            "Zuruecklegen braucht einen Ablauf, der mit offenem Greifer beginnt "
            "und mit geschlossenem endet (Objekt am Uebergabepunkt)."
        )
    g = closed.index(True)
    if not all(closed[g:]):
        raise PutBackError(
            "Zuruecklegen braucht genau EINEN Greiferwechsel (auf -> zu); "
            "der Ablauf oeffnet danach wieder (%s)."
            % waypoints[g + closed[g:].index(False)].name
        )
    return g


def put_back_waypoints(waypoints):
    """Vorwaertsbahn -> Rueckfahrt: Uebergabepunkt -> Greifpunkt (oeffnen) -> Start."""
    g = grasp_index(waypoints)
    n = len(waypoints)
    if any(wp.joints is None for wp in waypoints):
        raise PutBackError(
            "Zuruecklegen braucht die geteachten Gelenkstellungen aller Punkte "
            "(Ablaufdatei, nicht --waypoints)."
        )
    back = []
    for k in range(n - 1, -1, -1):
        wp = waypoints[k]
        if k == n - 1:
            # Startpunkt der Rueckfahrt: Bewegungsart wird nicht benutzt
            motion, approach = wp.motion, False
        else:
            # Segment k+1 -> k faehrt wie das Vorwaertssegment k -> k+1
            nxt = waypoints[k + 1]
            motion, approach = nxt.motion, nxt.approach
        back.append(
            Waypoint(
                wp.pose_quat.copy(),
                gripper_closed=k > g,  # am Greifpunkt auf, danach offen
                approach=approach or k == g,
                name=wp.name,
                motion=motion,
                joints=wp.joints.copy(),
                blend_m=0.0,
            )
        )
    return back


def plan_put_back(kin, collision_model, waypoints, fk, collision_stride=1,
                  regrip=config.PUT_BACK_REGRIP, **speeds):
    """Unverrauschter, kollisionsgepruefter Plan der Rueckfahrt.

    ``speeds``: ``transit_speed``, ``approach_speed``, ``ptp_joint_speed``
    wie bei der Aufzeichnung (bc/trajectory.py). Soll- und Ist-Bahn des
    Plans sind identisch -- er wird nur gefahren, nie gespeichert. Der
    Abstieg zum Ablegen faehrt hoechstens ``config.PUT_BACK_APPROACH_SPEED_MS``;
    ``regrip``: nach dem Oeffnen einmal zu und wieder auf (:func:`with_regrip`).
    """
    speeds = dict(speeds)
    speeds["approach_speed"] = min(float(speeds.get("approach_speed", config.APPROACH_SPEED_MS)),
                                   config.PUT_BACK_APPROACH_SPEED_MS)
    back = put_back_waypoints(waypoints)
    ideal = build_ideal_trajectory(back, fk=fk, **speeds)
    try:
        joints = kin.solve_path(ideal.poses_quat, back[0].joints, fixed_joints=ideal.joints)
    except IKFailure as exc:
        raise PutBackError(
            "Rueckfahrt nicht loesbar (Schritt %s: %s)" % (exc.index, exc)
        ) from exc
    speed, joint, step = joint_speed(joints, ideal.rate_hz)
    if speed > config.PLAN_MAX_JOINT_SPEED_RADS:
        raise PutBackError(
            "Rueckfahrt zu schnell: Gelenk %d %.2f rad/s bei Schritt %d (Grenze %.2f)"
            % (joint + 1, speed, step, config.PLAN_MAX_JOINT_SPEED_RADS))
    violations = collision_model.check_path(kin.robot, joints, stride=collision_stride)
    if violations:
        raise PutBackError("Rueckfahrt verletzt das Kollisionsmodell: %s" % violations[0])
    plan = TrajectoryPlan(
        joints_ideal=joints,
        joints_noisy=joints.copy(),
        poses_ideal=ideal.poses_quat,
        poses_noisy=ideal.poses_quat.copy(),
        gripper=ideal.gripper,
        dwell_mask=ideal.dwell_mask,
        rate_hz=ideal.rate_hz,
    )
    return with_regrip(plan) if regrip else plan


def with_regrip(plan, dwell_steps=config.GRIPPER_DWELL_STEPS):
    """Nach dem Oeffnen am Ablagepunkt: stehen bleiben, zu, auf (Nachgreifen).

    Eingefuegt direkt hinter dem Oeffnen und dessen Dwell -- der Arm steht
    dort still auf der Greifpose. Der Parallelgreifer zentriert das Objekt
    zwischen den Backen; danach ist seine Lage durch den Greifer bestimmt
    (Anwender 2026-10-26). Ohne Oeffnen im Plan: unveraendert.
    """
    gripper = np.asarray(plan.gripper, dtype=float)
    opens = np.flatnonzero((gripper[:-1] >= config.GRIPPER_THRESHOLD)
                           & (gripper[1:] < config.GRIPPER_THRESHOLD))
    if len(opens) == 0:
        return plan
    at = int(opens[0]) + 1                     # erster Schritt mit offenem Greifer
    hold_end = at
    while hold_end + 1 < len(gripper) and plan.dwell_mask[hold_end + 1]:
        hold_end += 1                          # Dwell des Oeffnens abwarten
    insert_at = hold_end + 1
    block = 2 * dwell_steps

    def repeat(array, value=None):
        array = np.asarray(array)
        filler = np.repeat(array[hold_end:hold_end + 1], block, axis=0)
        if value is not None:
            filler = np.full((block,) + array.shape[1:], value, dtype=array.dtype)
        return np.concatenate([array[:insert_at], filler, array[insert_at:]])

    new_gripper = repeat(gripper)
    new_gripper[insert_at:insert_at + dwell_steps] = config.GRIPPER_CLOSED
    new_gripper[insert_at + dwell_steps:insert_at + block] = config.GRIPPER_OPEN
    return TrajectoryPlan(
        joints_ideal=repeat(plan.joints_ideal),
        joints_noisy=repeat(plan.joints_noisy),
        poses_ideal=repeat(plan.poses_ideal),
        poses_noisy=repeat(plan.poses_noisy),
        gripper=new_gripper,
        dwell_mask=repeat(plan.dwell_mask, True),
        rate_hz=plan.rate_hz,
    )


def stratified_scales(n, rng, lo=0.0, hi=1.0):
    """n Rauschfaktoren, je einer aus n gleich breiten Teilen von [lo, hi].

    Gegen das Zufallsloch bei wenigen Wiederholungen: Drei unabhaengige
    U(0,1)-Zuege liegen oft alle niedrig oder alle hoch. Gestaffelt kommt
    je Lage eine schwache, eine mittlere und eine starke Fahrt vor -- die
    Verteilung ueber den Datensatz bleibt U(lo, hi) wie bisher
    (config.NOISE_EPISODE_SCALE_RANGE). Die Reihenfolge ist zufaellig,
    damit die Wiederholnummer nicht mit der Rauschstaerke zusammenhaengt.
    """
    edges = np.linspace(lo, hi, n + 1)
    scales = edges[:-1] + rng.uniform(0.0, 1.0, n) * np.diff(edges)
    return [float(s) for s in rng.permutation(scales)]
