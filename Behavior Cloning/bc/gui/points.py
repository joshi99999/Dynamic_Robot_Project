"""Punkte-Liste des Aufnahme-Reiters -- Qt-frei (AP 2.2, Anwender 2026-10-26).

Die Ablauftabelle zeigt SCHRITTE (PRE_GRASP steht zweimal darin, berechnete
Punkte mittendrin). Am Arbeitsplatz zaehlt aber, welche PUNKTE es gibt und
was mit ihnen zu tun ist: PICK je Lage teachen, CLEAR_FOV und PRE_PLACE
einmal fuer den Datensatz, PRE_GRASP und ANFLUG nur in der Hoehe
einstellen. Diese Liste wird aus den Schritten abgeleitet; Aenderungen gehen
in die Schritte zurueck, die Ablaufdatei bleibt die eine Wahrheit.
"""

import numpy as np

#: Punkte, die je Objektlage neu geteacht werden (wie ``record.py
#: --object-points``); alle anderen geteachten gelten fuer den Datensatz.
PER_POSE = ("PICK", "PRE_PICK")

TAUGHT = "geteacht"
COMPUTED = "berechnet"
#: Tischpunkt (bc/table.py): geteacht, aber nicht Teil der Bahn
TABLE = "tisch"
#: Mitte einer Sperrzone (bc/table.Keepout), ebenso nicht Teil der Bahn
KEEPOUT = "sperre"


def point_rows(steps, per_pose=PER_POSE, table_points=(), keepout_points=()):
    """Schritte -> eine Zeile je Punktname, in Reihenfolge des ersten Auftretens.

    Jede Zeile: ``name``, ``kind`` (geteacht/berechnet), ``above``, ``dz``,
    ``scope`` ("je Lage" / "einmal" / ""), ``gripper`` (Greiferbefehl, nur
    wenn der Punkt genau einmal vorkommt, sonst ``"mehrfach"``),
    ``count``.
    """
    rows = {}
    for step in steps:
        name = str(step.get("point") or "").strip()
        if not name:
            continue
        row = rows.setdefault(name, {
            "name": name, "kind": TAUGHT, "above": None, "dz": 0.0,
            "scope": "", "gripper": step.get("gripper"), "count": 0,
        })
        row["count"] += 1
        if step.get("above"):
            row["kind"], row["above"], row["dz"] = COMPUTED, step["above"], float(step.get("dz") or 0)
    for row in rows.values():
        if row["kind"] == TAUGHT:
            row["scope"] = "je Lage" if row["name"] in per_pose else "einmal"
        if row["count"] > 1:
            row["gripper"] = "mehrfach"
    for kind, names in ((TABLE, table_points), (KEEPOUT, keepout_points)):
        for name in names:
            if name not in rows:
                rows[name] = {"name": name, "kind": kind, "above": None, "dz": 0.0,
                              "scope": "einmal", "gripper": None, "count": 0}
    return list(rows.values())


def chain_above(name, computed):
    """Berechnete Punkte UEBER ``name``, von unten nach oben.

    ``computed``: ``{Punkt: (Bezugspunkt, dz)}`` (``Sequence.computed``).
    PICK -> [PRE_GRASP, ANFLUG].
    """
    out, current = [], name
    while True:
        above = [p for p, (ref, _dz) in computed.items() if ref == current and p not in out]
        if not above:
            return out
        current = above[0]
        out.append(current)


def approach_path(target, points, computed, current_joints, tol=0.02):
    """Punktnamen fuer "Anfahren" -- senkrecht weg und senkrecht hin.

    Steht der Arm an einem Punkt mit berechneten Punkten darueber (z. B. an
    PICK), faehrt er erst diese hinauf; das Ziel wird ueber die Punkte
    darueber von oben angefahren (ANFLUG -> PRE_GRASP -> PICK). Ein direktes
    PTP von PICK weg oder zu PICK hin koennte seitlich am Objekt entlang
    ziehen. ``points``: ``ResolvedSequence.points``.
    """
    current = np.asarray(current_joints, dtype=float)
    at = None
    for name, data in points.items():
        if np.max(np.abs(np.asarray(data["joints"]) - current)) <= tol:
            at = name
            break
    if at == target:
        return []
    down = list(reversed(chain_above(target, computed))) + [target]
    if at in down:
        # schon auf dem Weg nach unten (z. B. an PRE_GRASP, Ziel PICK)
        return down[down.index(at) + 1:]
    up = chain_above(at, computed) if at else []
    if target in up:
        # nur hinauf (z. B. von PICK zu PRE_GRASP)
        return up[:up.index(target) + 1]
    path = []
    for name in up + down:
        if not path or path[-1] != name:
            path.append(name)
    return path
