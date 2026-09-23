"""Sollwert-Interpolation fuer servo_j zwischen zwei Policy-Takten (AP 1.3/4.1).

Die Policy und die Aufzeichnung laufen mit CONTROL_RATE_HZ (15 Hz). Wird
jeder Zielwinkel direkt per servo_j gesendet, faehrt der Controller ihn an
und steht bis zum naechsten Sollwert fast still -- gemessen VM 2026-09-15:
die Ist-Geschwindigkeit schwankte innerhalb jedes Takts zwischen 0.004 und
0.6 rad/s bei 0.1 rad/s Soll. Sichtbar als Ruckeln, besonders bei langsamer
Fahrt (Endanflug ans Objekt).

Abhilfe: das neue Ziel wird ueber den FOLGENDEN Takt linear angefahren und
mit SERVO_RATE_HZ in Zwischenschritten gesendet (First-Order-Hold). Die
Zwischenschritte kennen nur das aktuelle und das neue Ziel -- sie brauchen
keine Zukunft der Bahn. Deshalb ist dieselbe Klasse in der Aufzeichnung
(recorder.py) und in der Inferenz (apps/infer.py) verwendbar, und beide
behandeln den Befehl zeitlich identisch (AP 1.5.1): das Ziel fuer Takt t+1
ist genau im Takt t+1 erreicht, die Beobachtung wird dort gelesen.

Geschwindigkeit und Beschleunigung zu jedem Zwischenschritt: v = Ziel-
differenz / Taktdauer (konstant im Fenster), a = Aenderung dieser
Geschwindigkeit gegenueber dem Vorfenster / Taktdauer. Beides ist kausal
und passt zur gesendeten Position (ports.RobotPort.servo_j: nie pauschal 0).
"""

import numpy as np

from . import config
from .ports import ServoLimitError


class ServoInterpolator(object):
    """Zerlegt 15-Hz-Ziele in Zwischenschritte fuer servo_j."""

    def __init__(self, control_rate_hz=config.CONTROL_RATE_HZ, servo_rate_hz=config.SERVO_RATE_HZ):
        ratio = servo_rate_hz / control_rate_hz
        substeps = int(round(ratio))
        if substeps < 1 or abs(ratio - substeps) > 1e-9:
            raise ValueError(
                "SERVO_RATE_HZ (%.3f) muss ein ganzzahliges Vielfaches von "
                "CONTROL_RATE_HZ (%.3f) sein" % (servo_rate_hz, control_rate_hz)
            )
        self.substeps = substeps
        self.control_period = 1.0 / control_rate_hz
        self.servo_rate_hz = servo_rate_hz
        self._q = None
        self._v = None

    def reset(self, joints):
        """Startzustand: Arm steht bei ``joints``."""
        self._q = np.asarray(joints, dtype=float).copy()
        self._v = np.zeros_like(self._q)
        return self

    @property
    def target(self):
        return None if self._q is None else self._q.copy()

    def hold(self):
        """Ein Sollwert "hier stehen bleiben" (v = a = 0) fuer den ersten Takt."""
        if self._q is None:
            raise RuntimeError("ServoInterpolator.reset() zuerst aufrufen")
        zeros = np.zeros_like(self._q)
        return self._q.copy(), zeros, zeros

    def window(self, target):
        """Zwischenschritte zum neuen Ziel: Liste von (q, v, a), Laenge
        ``substeps``. Der letzte Eintrag ist exakt ``target``."""
        if self._q is None:
            raise RuntimeError("ServoInterpolator.reset() zuerst aufrufen")
        target = np.asarray(target, dtype=float)
        if target.shape != self._q.shape:
            raise ValueError("Ziel hat %s Werte, erwartet %s" % (target.shape, self._q.shape))
        velocity = (target - self._q) / self.control_period
        acceleration = (velocity - self._v) / self.control_period
        start = self._q
        out = []
        for k in range(1, self.substeps + 1):
            q = target.copy() if k == self.substeps else start + (k / self.substeps) * (target - start)
            out.append((q, velocity.copy(), acceleration.copy()))
        self._q = target.copy()
        self._v = velocity
        return out


class TargetLimiter(object):
    """Weicher Geschwindigkeitsfilter fuer Policy-Ziele (Inferenz, AP 4.2).

    Begrenzt die Aenderung des Ziels je Takt je Gelenk auf ``max_speed *
    control_period`` -- Richtung bleibt, nur der Betrag wird gekappt. Anders
    als der ServoGuard im Adapter lehnt er nicht ab: ein einzelner Ausreisser
    der Policy verlangsamt die Fahrt kurz, statt sie zu beenden. ``clamped``
    zaehlt die betroffenen Takte (Protokoll) -- haeufiges Kappen heisst, die
    Policy verlangt mehr Tempo, als aufgezeichnet wurde.
    """

    def __init__(self, max_speed_rads=config.POLICY_MAX_JOINT_SPEED_RADS,
                 control_rate_hz=config.CONTROL_RATE_HZ):
        self.max_step = float(max_speed_rads) / float(control_rate_hz)
        self._q = None
        self.clamped = 0

    def reset(self, joints):
        self._q = np.asarray(joints, dtype=float).copy()
        self.clamped = 0
        return self

    def limit(self, target):
        target = np.asarray(target, dtype=float)
        if self._q is None:
            raise RuntimeError("TargetLimiter.reset() zuerst aufrufen")
        step = target - self._q
        biggest = float(np.max(np.abs(step)))
        if biggest > self.max_step:
            step = step * (self.max_step / biggest)
            self.clamped += 1
        self._q = self._q + step
        return self._q.copy()


class ServoGuard(object):
    """Sprung- und Geschwindigkeitsfilter fuer servo_j (AP 4.2, Rauschstudie).

    Sitzt IN den Robot-Adaptern (NeuraRobot, SimRobot) und prueft jeden
    Sollwert, bevor er den Controller erreicht -- egal ob er vom Recorder,
    von der Inferenzschleife oder aus einem Werkzeug kommt. Die Policy kann
    so nie einen Sprung kommandieren, auch wenn die Schleife darueber einen
    Fehler hat.

    Geprueft wird gegen den ZULETZT GESENDETEN Sollwert (Start: gemessene
    Stellung beim Aktivieren):

    * endliche Werte, richtige Laenge,
    * Achsgrenzen (config.JOINT_LIMITS_RAD, provisorisch),
    * Geschwindigkeit: |dq| <= v_max * dt je Gelenk. ``dt`` ist die
      gemessene Zeit seit dem letzten Sollwert, begrenzt auf
      [1/SERVO_RATE_HZ, 1/CONTROL_RATE_HZ]: ein langsamer Aufrufer wird
      nicht faelschlich abgelehnt, und ein Stocken der Schleife erlaubt
      trotzdem keinen grossen Sprung,
    * Abstand zur gemessenen Stellung (optional, ``check_gap``) -- die
      Inferenz meldet sie je Takt, damit ein Ziel "weit weg vom Arm"
      auffaellt, auch wenn es langsam dorthin wandert.

    Bewusst NICHT geprueft: Beschleunigung. Die Rampen des Planers und
    die Mittelung der Chunks glaetten; eine harte Grenze darauf wuerde vor
    allem Fehlalarme bei Greifer-Dwell-Starts erzeugen. Grenzwerte in
    config.SERVO_MAX_*.
    """

    def __init__(
        self,
        max_speed_rads=config.SERVO_MAX_JOINT_SPEED_RADS,
        max_gap_rad=config.SERVO_MAX_TARGET_GAP_RAD,
        joint_limits=config.JOINT_LIMITS_RAD,
        min_dt=1.0 / config.SERVO_RATE_HZ,
        max_dt=1.0 / config.CONTROL_RATE_HZ,
    ):
        self.max_speed = float(max_speed_rads)
        self.max_gap = float(max_gap_rad)
        self.joint_limits = joint_limits
        self.min_dt = float(min_dt)
        self.max_dt = float(max_dt)
        self._q = None
        self._t = None
        self.rejections = 0

    def reset(self, joints, t):
        """Beim Aktivieren des Servo-Interface: Start an der Ist-Stellung."""
        self._q = np.asarray(joints, dtype=float).copy()
        self._t = float(t)
        return self

    @property
    def active(self):
        return self._q is not None

    def clear(self):
        self._q = None
        self._t = None

    def check(self, joints, t):
        """Prueft einen Sollwert; wirft ServoLimitError, sonst uebernimmt ihn."""
        if self._q is None:
            raise ServoLimitError("ServoGuard ohne Startstellung (reset fehlt)")
        q = np.asarray(joints, dtype=float)
        reason = self._violation(q, float(t))
        if reason:
            self.rejections += 1
            raise ServoLimitError("servo_j abgelehnt: %s" % reason)
        self._q = q.copy()
        self._t = float(t)

    def check_gap(self, target, measured):
        """Abstand Ziel <-> gemessene Stellung (je Gelenk). Wirft bei
        Verletzung. Nicht Teil von check(), weil nicht jeder Aufrufer je
        Sollwert eine frische Messung hat."""
        target = np.asarray(target, dtype=float)
        measured = np.asarray(measured, dtype=float)
        gap = np.abs(target - measured)
        if not np.all(np.isfinite(gap)):
            self.rejections += 1
            raise ServoLimitError("Ziel oder Messung nicht endlich")
        worst = int(np.argmax(gap))
        if gap[worst] > self.max_gap:
            self.rejections += 1
            raise ServoLimitError(
                "Ziel %.3f rad von der Ist-Stellung entfernt (Gelenk %d, Grenze %.3f)"
                % (gap[worst], worst + 1, self.max_gap)
            )

    def _violation(self, q, t):
        if q.shape != self._q.shape:
            return "%d Werte statt %d" % (q.size, self._q.size)
        if not np.all(np.isfinite(q)):
            return "Sollwert nicht endlich (%s)" % np.round(q, 4).tolist()
        if self.joint_limits is not None:
            for i, (lo, hi) in enumerate(self.joint_limits):
                if not lo <= q[i] <= hi:
                    return "Gelenk %d = %.3f rad ausserhalb [%.2f, %.2f]" % (
                        i + 1, q[i], lo, hi)
        dt = min(max(t - self._t, self.min_dt), self.max_dt)
        step = np.abs(q - self._q)
        allowed = self.max_speed * dt
        worst = int(np.argmax(step))
        if step[worst] > allowed + 1e-9:
            return (
                "Sprung %.4f rad an Gelenk %d in %.1f ms (%.2f rad/s > %.2f rad/s)"
                % (step[worst], worst + 1, dt * 1e3, step[worst] / dt, self.max_speed)
            )
        return None
