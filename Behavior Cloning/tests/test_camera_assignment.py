"""Freie Kamerazuordnung (AP 1.1): Geraeteerkennung und Platzbesetzung.

Hardwarefrei: geprueft wird die Zuordnungslogik, nicht das Oeffnen. Dass
``sim`` immer dabei ist, ist die eigentliche Zusage -- ohne jede Kamera muss
sich die ganze Kette durchspielen lassen (AP 0.5).
"""

import _paths  # noqa: F401

from bc import config
from bc.adapters import (
    CAMERA_BACKENDS,
    apply_camera_specs,
    camera_config_for,
    camera_configs,
    list_devices,
    parse_camera_spec,
    sim_device,
)


def test_placeholder_is_always_available():
    device = sim_device()
    assert device.backend == "sim"
    assert device.available is True
    assert device.key == "sim"


def test_inventory_always_offers_the_placeholder_first():
    # Ohne Hardware und ohne SDK darf die Liste nicht leer sein.
    inventory = list_devices(uvc=False, daheng=False)
    assert [d.key for d in inventory["devices"]] == ["sim"]


def test_missing_sdk_is_a_note_not_an_empty_list():
    inventory = list_devices(uvc=False, daheng=True)
    assert inventory["devices"][0].backend == "sim"
    # Entweder es gibt Geraete oder eine Begruendung -- nie beides nicht.
    daheng = [d for d in inventory["devices"] if d.backend == "daheng"]
    assert daheng or inventory["notes"].get("daheng")


def test_device_key_is_the_command_line_spelling():
    for backend, device, expected in (("sim", None, "sim"),
                                      ("uvc", 1, "uvc:1"),
                                      ("daheng", "EBK24100633", "daheng:EBK24100633")):
        name, parsed_backend, parsed_device = parse_camera_spec("wrist=" + expected)
        assert (name, parsed_backend) == ("wrist", backend)
        assert parsed_device == device


def test_unknown_slot_and_backend_are_rejected():
    for bad in ("wrist", "greifer=sim", "wrist=infrarot", "wrist=uvc:abc",
                "wrist=sim:3"):
        try:
            parse_camera_spec(bad)
        except ValueError:
            continue
        raise AssertionError("'%s' haette auffallen muessen" % bad)


def test_same_backend_keeps_the_settings_and_only_swaps_the_device():
    # Belichtung, ROI und Rate der Wrist-Kamera duerfen beim Geraetewechsel
    # nicht verlorengehen -- sonst aendert sich die Bildstatistik.
    cfg = camera_config_for("wrist", "daheng", "ANDERE_SN")
    assert cfg.device == "ANDERE_SN"
    assert cfg.exposure_us == config.WRIST_CAMERA.exposure_us
    assert cfg.width == config.WRIST_CAMERA.width
    assert cfg.fps == config.WRIST_CAMERA.fps


def test_prepared_configurations_are_used_when_the_backend_differs():
    # Fuer die Szene gibt es eine vorbereitete zweite Daheng (AP 1.1).
    cfg = camera_config_for("scene", "daheng", "SN2")
    assert cfg.name == "scene"
    assert cfg.exposure_us == config.SCENE_CAMERA_DAHENG.exposure_us
    assert cfg.fps == config.SCENE_CAMERA_DAHENG.fps


def test_every_slot_can_be_a_placeholder():
    for cam in config.CAMERAS:
        cfg = camera_config_for(cam.name, "sim")
        assert cfg.backend == "sim"
        assert cfg.name == cam.name


def test_every_backend_can_fill_every_slot():
    # Das ist die Zusage "Komponententausch ohne Code-Aenderung".
    for cam in config.CAMERAS:
        for backend in CAMERA_BACKENDS:
            device = {"sim": None, "uvc": 0, "daheng": "SN"}[backend]
            cfg = camera_config_for(cam.name, backend, device)
            assert cfg.backend == backend and cfg.name == cam.name


def test_assignment_replaces_only_the_named_slot():
    cfgs = camera_configs("sim", specs=["scene=uvc:2"])
    by_name = {c.name: c for c in cfgs}
    assert by_name["wrist"].backend == "sim"
    assert by_name["scene"].backend == "uvc" and by_name["scene"].device == 2
    # Reihenfolge bleibt die der Konfiguration -- der Datensatz haengt daran.
    assert [c.name for c in cfgs] == [c.name for c in config.CAMERAS]


def test_assignment_without_specs_changes_nothing():
    assert apply_camera_specs(config.SIM_CAMERAS, None) == list(config.SIM_CAMERAS)
    assert apply_camera_specs(config.SIM_CAMERAS, []) == list(config.SIM_CAMERAS)


def test_a_real_mode_can_be_turned_into_placeholders():
    # Der Fall des Anwenders: "real" gewaehlt, aber zum Testen ohne Kamera.
    cfgs = camera_configs("real", specs=["wrist=sim", "scene=sim"])
    assert all(c.backend == "sim" for c in cfgs)
