"""tools/check_daheng_api.py: Namensliste aus dem Adapter, Pruefung gegen gxipy (AP 1.1).

Hardwarefrei: statt gxipy ein nachgebautes Modul mit absichtlich fehlenden
Namen.
"""

import sys
from pathlib import Path

import _paths  # noqa: F401

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import check_daheng_api as tool  # noqa: E402


def _names():
    return tool.adapter_names(tool.ADAPTER.read_text(encoding="utf-8"))


def test_adapter_names_are_read_from_the_adapter():
    names = _names()
    assert names["module"]["GxAutoEntry"] == {"CONTINUOUS", "OFF", "ONCE"}
    assert names["module"]["GxSwitchEntry"] == {"ON", "OFF"}
    assert names["module"]["GxFrameStatusList"] == {"SUCCESS"}
    assert "DeviceManager" in names["module"]
    assert names["manager"] == {"update_device_list", "open_device_by_sn", "open_device_by_index"}
    assert names["device"] == {"stream_on", "stream_off", "close_device", "data_stream"}
    assert names["stream"] == {"get_image"}
    assert names["image"] == {"get_status", "convert", "get_numpy_array"}
    # Die Features, deren stilles Fehlen falsche Aufnahmen erzeugen wuerde
    for feature in ("ExposureAuto", "ExposureTime", "GainAuto", "Gain", "BalanceWhiteAuto",
                    "BalanceRatioSelector", "BalanceRatio", "TriggerMode", "Width", "Height",
                    "OffsetX", "OffsetY", "WidthMax", "HeightMax", "AcquisitionFrameRate"):
        assert feature in names["features"], feature
    # Methoden landen nicht bei den Features
    assert not names["features"] & names["device"]


class _Enum:
    def __init__(self, *members):
        for m in members:
            setattr(self, m, m)


class _FakeManager:
    def update_device_list(self):
        return 0, []

    def open_device_by_sn(self, sn):
        raise AssertionError("darf ohne --open nicht aufgerufen werden")
    # open_device_by_index fehlt absichtlich


class _FakeGx:
    DeviceManager = _FakeManager
    GxSwitchEntry = _Enum("ON", "OFF")
    GxAutoEntry = _Enum("CONTINUOUS", "OFF")  # ONCE fehlt absichtlich
    GxFrameStatusList = _Enum("SUCCESS")


FAKE_SOURCES = {
    "gxiapi.py": (
        "class Device:\n"
        "    def __init__(self):\n"
        "        self.data_stream = []\n"
        "        self.ExposureTime = None\n"
        "        self.ExposureAuto = None\n"
        "    def stream_on(self): pass\n"
        "    def stream_off(self): pass\n"
        "    def close_device(self): pass\n"
        "class DataStream:\n"
        "    def get_image(self): pass\n"
        "class RawImage:\n"
        "    def get_status(self): pass\n"
        "    def convert(self, mode): pass\n"
        "    def get_numpy_array(self): pass\n"
    )
}


def test_static_check_reports_missing_names():
    findings = []
    tool.check_static(_FakeGx, _names(), FAKE_SOURCES,
                      lambda level, text: findings.append((level, text)))
    fails = [t for lvl, t in findings if lvl == "fail"]
    warns = [t for lvl, t in findings if lvl == "warn"]
    oks = [t for lvl, t in findings if lvl == "ok"]

    assert any("GxAutoEntry" in t and "ONCE" in t for t in fails)
    assert any("open_device_by_index" in t for t in fails)
    assert len(fails) == 2, fails
    # Vorhandenes wird als in Ordnung gemeldet
    assert any("DeviceManager.update_device_list" in t for t in oks)
    assert any(t.startswith("Datenstrom.get_image") for t in oks)
    assert any(t.startswith("ExposureTime") for t in oks)
    # Features ohne Treffer sind nur Hinweise (koennen dynamisch am Geraet haengen)
    assert any(t.startswith("BalanceRatio ") for t in warns)
    assert not any(t.startswith("ExposureTime") for t in warns)
