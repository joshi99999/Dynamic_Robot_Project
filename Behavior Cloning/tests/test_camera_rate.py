"""Bildrate der Kameras: einstellen, zuruecklesen, pruefen (AP 1.1/1.3).

Anlage 2026-10-26: Die Wrist-Kamera lieferte am USB3-Hub 23 statt 61 fps
-- 20-24 % der Frames ueber dem 30-ms-Budget, alle Episoden verworfen.
Ausserdem blieb nach dem Tauschen der Plaetze in der Oberflaeche die
30-fps-Begrenzung in der Kamera stehen, weil "nativ" nichts gesetzt hat.
"""

from types import SimpleNamespace

import _paths  # noqa: F401

from bc import config
from bc.adapters import (
    FPS_NATIVE,
    apply_camera_specs,
    describe_settings,
    fps_label,
    parse_camera_spec,
    split_camera_fps,
)
from bc.adapters.cam_daheng import DahengCamera
from bc.gui import cameras as cameras_module
from bc.gui import recording
from bc.sync import camera_rate_findings


def test_fps_suffix_in_camera_spec():
    assert split_camera_fps("wrist=daheng:SN1@30") == ("wrist=daheng:SN1", 30.0)
    assert split_camera_fps("wrist=daheng@nativ") == ("wrist=daheng", FPS_NATIVE)
    assert split_camera_fps("scene=uvc:1") == ("scene=uvc:1", None)
    # die Zuordnung selbst bleibt lesbar wie bisher
    assert parse_camera_spec("wrist=daheng:SN1@45") == ("wrist", "daheng", "SN1")
    for bad in ("wrist=daheng@schnell", "wrist=daheng@0", "wrist=daheng@500"):
        try:
            split_camera_fps(bad)
            assert False, "ValueError erwartet fuer %s" % bad
        except ValueError:
            pass


def test_fps_overrides_the_slot_default():
    base = list(config.CAMERAS)
    cfgs = {c.name: c for c in apply_camera_specs(
        base, ["wrist=daheng:EBK24100633@30", "scene=daheng:SN2@nativ"])}
    assert cfgs["wrist"].fps == 30.0
    assert cfgs["scene"].fps is None          # nativ = ohne Begrenzung
    assert fps_label(cfgs["scene"]) == FPS_NATIVE
    # ohne Angabe: Vorgabe des Platzes (Wrist nativ, Szene-Daheng 30)
    cfgs = {c.name: c for c in apply_camera_specs(
        base, ["wrist=daheng:EBK24100633", "scene=daheng:SN2"])}
    assert cfgs["wrist"].fps == config.WRIST_CAMERA.fps
    assert cfgs["scene"].fps == config.SCENE_CAMERA_DAHENG.fps
    # Platzhalter haben keine Rate
    assert fps_label(apply_camera_specs(base, ["wrist=sim@30"])[0]) is None


def test_gui_appends_only_chosen_rates():
    assignment = {"wrist": "daheng:SN1", "scene": "sim"}
    assert cameras_module.to_specs(assignment) == ["wrist=daheng:SN1", "scene=sim"]
    specs = cameras_module.to_specs(assignment, {"wrist": FPS_NATIVE, "scene": 30.0})
    assert specs == ["wrist=daheng:SN1@nativ", "scene=sim"]
    configs = cameras_module.to_configs(assignment, {"wrist": 45.0, "scene": None})
    assert configs[0].fps == 45.0
    assert cameras_module.fps_choice_label("wrist", None) == "Vorgabe (60 fps)"
    assert config.WRIST_CAMERA.fps == config.SCENE_CAMERA_DAHENG.fps == 60.0
    argv = recording.record_argv("data_vm/x", robot_kind="neura", camera_specs=specs)
    assert "wrist=daheng:SN1@nativ" in argv


class _Feature(object):
    def __init__(self, log, name, value=None):
        self.log, self.name, self.value = log, name, value

    def set(self, value):
        self.log.append((self.name, value))
        self.value = value

    def get(self):
        if isinstance(self.value, Exception):
            raise self.value
        return self.value


def _fake_camera(log):
    names = ["TriggerMode", "BinningHorizontal", "BinningVertical", "OffsetX", "OffsetY",
             "Width", "Height", "WidthMax", "HeightMax", "ExposureAuto", "ExposureTime",
             "GainAuto", "Gain", "BalanceWhiteAuto", "BalanceRatioSelector", "BalanceRatio",
             "AcquisitionFrameRateMode", "AcquisitionFrameRate",
             "CurrentAcquisitionFrameRate", "DeviceLinkThroughputLimitMode",
             "DeviceLinkThroughputLimit", "DeviceLinkCurrentThroughput"]
    cam = SimpleNamespace(**{n: _Feature(log, n) for n in names})
    cam.WidthMax.value, cam.HeightMax.value = 1440, 1080
    cam.OffsetX.get_range = cam.OffsetY.get_range = lambda: {"max": 0, "inc": 1}
    return cam


_GX = SimpleNamespace(
    GxSwitchEntry=SimpleNamespace(OFF=0, ON=1),
    GxAutoEntry=SimpleNamespace(OFF=0, CONTINUOUS=1, ONCE=2),
)


def test_native_rate_actively_switches_the_limit_off():
    for fps, expected in ((None, [("AcquisitionFrameRateMode", 0)]),
                          (30.0, [("AcquisitionFrameRateMode", 1),
                                  ("AcquisitionFrameRate", 30.0)])):
        log = []
        camera = DahengCamera(config.CameraConfig(name="wrist", backend="daheng",
                                                  exposure_us=8000.0, fps=fps))
        camera._cam = _fake_camera(log)
        camera._configure(_GX)
        rate_calls = [c for c in log if c[0].startswith("AcquisitionFrameRate")]
        assert rate_calls == expected, (fps, rate_calls)


def test_settings_are_read_back_from_the_camera():
    log = []
    camera = DahengCamera(config.CameraConfig(name="wrist", backend="daheng"))
    cam = _fake_camera(log)
    cam.ExposureAuto.value = (0, "Off")
    cam.ExposureTime.value = 8000.0
    cam.GainAuto.value = (1, "Continuous")
    cam.Gain.value = 16.6
    cam.AcquisitionFrameRateMode.value = (1, "On")
    cam.AcquisitionFrameRate.value = 30.0
    cam.CurrentAcquisitionFrameRate.value = 30.0
    cam.DeviceLinkThroughputLimitMode.value = RuntimeError("nicht lesbar")
    cam.DeviceLinkCurrentThroughput.value = 47_000_000
    camera._cam = cam
    s = camera.settings()
    assert s["exposure_auto"] == "Off" and s["exposure_us"] == 8000.0
    assert s["rate_limit"] == "On" and s["rate_possible_fps"] == 30.0
    assert s["link_limit_mode"] is None  # nicht lesbar -> None, kein Abbruch
    line = describe_settings("wrist", s)
    assert "Belichtung fest 8000 us" in line and "begrenzt auf 30.0 fps" in line
    assert "Gain auto" in line and "USB 47 MB/s" in line
    assert DahengCamera(config.WRIST_CAMERA).settings() == {}  # nicht geoeffnet


def test_rate_findings_name_the_hub_case():
    # Anlage 26.10.: nativ angefordert, Kamera koennte 61.3, ankommen 23.1
    findings = camera_rate_findings("wrist", 23.1, FPS_NATIVE, 61.3)
    assert len(findings) == 2
    assert "23.1 von 61.3" in findings[0] and "Hub" in findings[0]
    assert "43 ms" in findings[1]
    # direkt am Laptop: alles gut
    assert camera_rate_findings("wrist", 60.8, FPS_NATIVE, 61.3) == []
    # 30 fps angefordert und erreicht: nur der Hinweis aufs Budget
    findings = camera_rate_findings("scene", 29.9, 30.0, 61.3)
    assert len(findings) == 1 and "Latenzbudget" in findings[0]
    # ohne zurueckgelesene Moeglichkeit (Webcam): nur gegen die Anforderung
    assert camera_rate_findings("scene", 12.0, 30.0, None)
