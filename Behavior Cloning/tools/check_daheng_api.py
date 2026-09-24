"""Daheng-Adapter gegen die INSTALLIERTE gxipy-Version pruefen (AP 1.1, AP 0.6 Punkt 7).

NUR LESEN -- es wird keine Kameraeinstellung geschrieben. Ohne Kamera
nutzbar; mit angeschlossener Kamera (``--open``) zusaetzlich am Geraet.

Warum: ``bc/adapters/cam_daheng.py`` ist gegen die gxipy-Doku geschrieben,
nicht gegen die Bibliothek. Jede Einstellung laeuft dort durch ``_try()`` --
gibt es ein Feature unter diesem Namen nicht, erscheint nur eine Hinweiszeile
und die Kamera laeuft mit FALSCHEN Einstellungen weiter (z. B. Auto-Belichtung
statt der festen 8 ms). Das faellt erst im Datensatz auf. Dieses Werkzeug
findet solche Namen vor dem Labortag.

1. Import wie im Adapter (``import_gxipy``: Registry-Umgebung, SDK-Pfade).
2. Alle gxipy-Namen, die der Adapter benutzt, werden AUS SEINEM QUELLTEXT
   gelesen (kann nicht veralten) und gegen die installierte Version geprueft:
   Modul-Attribute und Enum-Werte, Methoden von DeviceManager/Device/Bild.
3. Kamera-Features (TriggerMode, ExposureTime, ...) haengen je nach
   gxipy-Version erst am geoeffneten Geraet. Ohne Kamera: Suche im
   gxipy-Quelltext (Hinweis, kein Beweis). Mit ``--open``: am Geraet
   vorhanden / implementiert / schreibbar, aktueller Wert.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_daheng_api.py           # ohne Kamera
    python tools/check_daheng_api.py --open    # Kamera angeschlossen
"""

import argparse
import json
import os
import re
import sys
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

ADAPTER = Path(__file__).resolve().parent.parent / "bc" / "adapters" / "cam_daheng.py"

#: Geraete-Attribute des Adapters, die Methoden bzw. Objekte sind -- alle
#: anderen ``cam.<Name>`` sind GenICam-Features.
DEVICE_METHODS = ("stream_on", "stream_off", "close_device", "data_stream")


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def show(label, value):
    print("  %-40s %s" % (label, value))


def adapter_names(source):
    """Liest aus dem Adapter-Quelltext, welche gxipy-Namen er benutzt.

    Rueckgabe: dict mit
      module    {Name: set(Enum-Werte)}  -- gx.<Name>[.<Wert>]
      manager   set                      -- DeviceManager-Methoden
      device    set                      -- Methoden/Objekte am Geraet
      features  set                      -- GenICam-Features am Geraet
      image     set                      -- Methoden der Bildobjekte
      stream    set                      -- Methoden des Datenstroms
    """
    code = "\n".join(line.split("#", 1)[0] for line in source.splitlines())
    module = {}
    for name, member in re.findall(r"\b(?:self\._)?gx\.(\w+)(?:\.(\w+))?", code):
        module.setdefault(name, set())
        if member:
            module[name].add(member)
    device_attrs = set(re.findall(r"\bself\._cam\.(\w+)", code))
    device_attrs |= set(re.findall(r"(?<![\w.])cam\.(\w+)", code))
    return {
        "module": module,
        "manager": set(re.findall(r"\bself\._device_manager\.(\w+)", code)),
        "device": {n for n in device_attrs if n in DEVICE_METHODS},
        "features": {n for n in device_attrs if n not in DEVICE_METHODS},
        "image": set(re.findall(r"\b(?:raw|rgb_image)\.(\w+)\(", code)),
        "stream": set(re.findall(r"data_stream\[0\]\.(\w+)", code)),
    }


def gxipy_sources(gx):
    """Quelltext aller .py-Dateien des gxipy-Pakets (fuer die Textsuche)."""
    root = Path(gx.__file__).resolve().parent
    return {p.name: p.read_text(encoding="utf-8", errors="replace") for p in root.glob("*.py")}


def find_in_sources(sources, name, pattern=None):
    """Dateien, in denen ``name`` (oder ``pattern``) vorkommt."""
    rx = re.compile(pattern or r"\b%s\b" % re.escape(name))
    return sorted(f for f, text in sources.items() if rx.search(text))


def check_static(gx, names, sources, finding):
    """Stufe 2 und 3: ohne Kamera."""
    section("2. Modul-Attribute und Enum-Werte")
    for name in sorted(names["module"]):
        obj = getattr(gx, name, None)
        if obj is None:
            finding("fail", "gx.%s fehlt in der installierten gxipy" % name)
            continue
        missing = sorted(m for m in names["module"][name] if not hasattr(obj, m))
        members = ", ".join(sorted(names["module"][name])) or "-"
        if missing:
            finding("fail", "gx.%s: Werte fehlen: %s" % (name, ", ".join(missing)))
        else:
            finding("ok", "gx.%s (%s)" % (name, members))

    section("3. Methoden: DeviceManager, Geraet, Datenstrom, Bild")
    manager = getattr(gx, "DeviceManager", None)
    for method in sorted(names["manager"]):
        if manager is not None and hasattr(manager, method):
            finding("ok", "DeviceManager.%s" % method)
        else:
            finding("fail", "DeviceManager.%s fehlt" % method)
    for label, group in (("Geraet", "device"), ("Datenstrom", "stream"), ("Bild", "image")):
        for method in sorted(names[group]):
            files = find_in_sources(sources, method, r"\bdef %s\b|\bself\.%s\b"
                                    % (re.escape(method), re.escape(method)))
            if files:
                finding("ok", "%s.%s (in %s)" % (label, method, ", ".join(files)))
            else:
                finding("fail", "%s.%s: weder als Methode noch als Attribut im gxipy-Quelltext"
                        % (label, method))

    section("4. Kamera-Features im gxipy-Quelltext (ohne Kamera nur ein Hinweis)")
    for feature in sorted(names["features"]):
        files = find_in_sources(sources, feature)
        if files:
            finding("ok", "%s (in %s)" % (feature, ", ".join(files)))
        else:
            finding("warn", "%s nicht im gxipy-Quelltext -- wird evtl. erst am Geraet "
                    "aufgeloest; mit --open pruefen" % feature)


def check_device(gx, names, serial, finding, report):
    """Stufe 5: am geoeffneten Geraet, ohne etwas zu schreiben."""
    section("5. Am Geraet (--open)")
    manager = gx.DeviceManager()
    count, info = manager.update_device_list()
    report["geraete"] = [{k: str(v) for k, v in i.items()} for i in info or []]
    show("gefundene Daheng-Kameras", count)
    for i in info or []:
        show("  %s" % i.get("model_name", "?"), "SN=%s" % i.get("sn", "?"))
    if count == 0:
        finding("fail", "--open: keine Daheng-Kamera gefunden (USB3-Kabel/Treiber, Galaxy Viewer)")
        return
    known = [str(i.get("sn", "")) for i in info]
    if serial and serial in known:
        cam = manager.open_device_by_sn(serial)
        show("geoeffnet", "SN %s (config.WRIST_CAMERA)" % serial)
    else:
        if serial:
            finding("warn", "Seriennummer %s aus config.WRIST_CAMERA nicht angeschlossen "
                    "(gefunden: %s) -- erstes Geraet geprueft" % (serial, ", ".join(known)))
        cam = manager.open_device_by_index(1)
        show("geoeffnet", "Index 1")

    report["features"] = {}
    try:
        for method in sorted(names["device"]):
            finding("ok" if hasattr(cam, method) else "fail",
                    "Geraet.%s %s" % (method, "vorhanden" if hasattr(cam, method) else "FEHLT"))
        for feature in sorted(names["features"]):
            obj = getattr(cam, feature, None)
            entry = {"vorhanden": obj is not None}
            if obj is None:
                finding("fail", "Feature %s gibt es an diesem Geraet nicht -- die Einstellung "
                        "im Adapter wirkt NICHT" % feature)
                report["features"][feature] = entry
                continue
            for attr in ("is_implemented", "is_readable", "is_writable"):
                fn = getattr(obj, attr, None)
                try:
                    entry[attr] = bool(fn()) if callable(fn) else None
                except Exception as exc:
                    entry[attr] = "Fehler: %s" % exc
            if entry.get("is_readable"):
                try:
                    value = obj.get()
                    entry["wert"] = value if isinstance(value, (int, float, str)) else repr(value)
                except Exception as exc:
                    entry["wert"] = "nicht lesbar: %s" % exc
            report["features"][feature] = entry
            text = "%s: implementiert=%s, schreibbar=%s, Wert=%s" % (
                feature, entry.get("is_implemented"), entry.get("is_writable"), entry.get("wert"))
            if entry.get("is_implemented") is False or entry.get("is_writable") is False:
                finding("warn", text + " -- Einstellung im Adapter wirkt nicht")
            else:
                finding("ok", text)
    finally:
        try:
            cam.close_device()
        except Exception:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--open", action="store_true",
                        help="zusaetzlich am angeschlossenen Geraet pruefen (nur lesen)")
    parser.add_argument("--serial", default=None,
                        help="Seriennummer fuer --open (Default config.WRIST_CAMERA.device)")
    parser.add_argument("--no-report", action="store_true", help="kein JSON in Berichte/")
    args = parser.parse_args(argv)

    from bc import config
    from bc.adapters.cam_daheng import import_gxipy
    from bc.ports import CameraError

    report = {"erzeugt": datetime.now().isoformat(timespec="seconds"), "befunde": []}

    def finding(level, text):
        report["befunde"].append([level, text])
        print("  [%s] %s" % (level.upper(), text))

    section("1. gxipy importieren (wie bc/adapters/cam_daheng.py)")
    try:
        gx = import_gxipy()
    except CameraError as exc:
        print(exc)
        finding("fail", "gxipy nicht importierbar -- Galaxy SDK installieren, Shell neu starten")
        return summary(report, args)
    show("gxipy", gx.__file__)
    show("Version", getattr(gx, "__version__", "unbekannt"))
    show("GALAXY_GENICAM_ROOT", os.environ.get("GALAXY_GENICAM_ROOT", "-"))
    report["gxipy"] = {"pfad": gx.__file__, "version": str(getattr(gx, "__version__", None))}
    finding("ok", "gxipy importiert")

    names = adapter_names(ADAPTER.read_text(encoding="utf-8"))
    report["adapter_namen"] = {k: sorted(v) if isinstance(v, set) else
                               {n: sorted(m) for n, m in v.items()} for k, v in names.items()}
    check_static(gx, names, gxipy_sources(gx), finding)

    if args.open:
        serial = args.serial or config.WRIST_CAMERA.device
        try:
            check_device(gx, names, str(serial) if serial else None, finding, report)
        except Exception as exc:
            finding("fail", "--open abgebrochen: %s: %s" % (type(exc).__name__, exc))
    else:
        print("\n  (Geraete-Pruefung uebersprungen -- mit Kamera: --open)")
    return summary(report, args)


def summary(report, args):
    section("Zusammenfassung")
    problems = [(lvl, text) for lvl, text in report["befunde"] if lvl != "ok"]
    ok = sum(1 for lvl, _ in report["befunde"] if lvl == "ok")
    for level, text in problems:
        print("  %-5s %s" % (level.upper(), text))
    print("  %d in Ordnung, %d Hinweise, %d Fehler" % (
        ok, sum(1 for l, _ in problems if l == "warn"), sum(1 for l, _ in problems if l == "fail")))
    if not args.no_report:
        out = Path("Berichte") / ("%s_Daheng-API.json" % datetime.now().strftime("%Y-%m-%d_%H-%M"))
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str),
                       encoding="utf-8")
        print("  Protokoll: %s" % out)
    return 1 if any(lvl == "fail" for lvl, _ in report["befunde"]) else 0


if __name__ == "__main__":
    sys.exit(main())
