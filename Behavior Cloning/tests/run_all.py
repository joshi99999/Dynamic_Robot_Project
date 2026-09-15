"""Minimaler Test-Runner ohne pytest (AP 0.10: Abhaengigkeiten nur
deklariert).

Fuehrt alle test_*-Funktionen der Testmodule aus. Unterstuetzt die
Parameter ``tmp_path`` (Wegwerf-Verzeichnis), ``robot`` und ``camera``
(Sim-Implementierungen) -- dieselben Tests laufen unveraendert auch unter
pytest, dort inklusive ``--robot=neura`` fuer den Hardwaretag.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tests/run_all.py
    python tests/run_all.py test_noise                 # nur ein Modul

Abnahme der Adapter an der Hardware (AP 0.6) -- dieselben Tests, nur
gegen die echten Geraete:
    python tests/run_all.py --camera=uvc    contract.test_camera_contract
    python tests/run_all.py --camera=daheng contract.test_camera_contract
    python tests/run_all.py --robot=neura   contract.test_robot_contract
"""

import importlib
import inspect
import shutil
import sys
import tempfile
import traceback
from pathlib import Path

import _paths  # noqa: F401

import _fixtures

MODULES = [
    "test_clock",
    "test_geometry",
    "test_urdf_kinematics",
    "test_collision",
    "test_trajectory",
    "test_noise",
    "test_sequence",
    "test_sync",
    "test_dataset",
    "test_recorder",
    "test_neura_adapter",
    "test_rectify",
    "test_safety",
    "test_policy",
    "test_layering",
    "test_apps",
    "contract.test_robot_contract",
    "contract.test_camera_contract",
]


#: Welche Port-Implementierung geprueft wird (via --robot=/--camera=),
#: analog zu den pytest-Optionen in conftest.py.
BACKENDS = {"robot": "sim", "camera": "sim"}


def _make_argument(name):
    """Erzeugt Testparameter analog zu den pytest-Fixtures (conftest.py)."""
    if name == "tmp_path":
        path = Path(tempfile.mkdtemp(prefix="bc_test_"))
        return path, lambda: shutil.rmtree(path, ignore_errors=True)
    if name == "robot":
        bot = _fixtures.make_robot(BACKENDS["robot"])
        return bot, bot.close
    if name == "camera":
        cam = _fixtures.make_camera(BACKENDS["camera"])
        return cam, (lambda: cam.close() if cam.is_open else None)
    raise ValueError("Unbekannter Testparameter '%s'" % name)


def run_module(module_name):
    sys.path.insert(0, str(Path(__file__).resolve().parent / "contract"))
    if module_name.startswith("contract."):
        module = importlib.import_module(module_name.split(".", 1)[1])
    else:
        module = importlib.import_module(module_name)

    passed, failed = 0, []
    for name in sorted(dir(module)):
        if not name.startswith("test_"):
            continue
        fn = getattr(module, name)
        if not callable(fn):
            continue
        params = list(inspect.signature(fn).parameters)
        args, cleanups = [], []
        try:
            for param in params:
                value, cleanup = _make_argument(param)
                args.append(value)
                cleanups.append(cleanup)
            fn(*args)
            passed += 1
            print("  OK   %s" % name)
        except Exception:
            failed.append((module_name, name))
            print("  FAIL %s" % name)
            traceback.print_exc()
        finally:
            for cleanup in cleanups:
                try:
                    cleanup()
                except Exception:
                    pass
    return passed, failed


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]

    modules = []
    for arg in argv:
        if arg.startswith("--robot=") or arg.startswith("--camera="):
            key, value = arg[2:].split("=", 1)
            BACKENDS[key] = value
        elif arg.startswith("-"):
            raise SystemExit("Unbekannte Option '%s'" % arg)
        else:
            modules.append(arg)
    modules = modules or MODULES

    for key, value in BACKENDS.items():
        if value != "sim":
            print("!! %s-Backend: %s (HARDWARE)" % (key, value))

    total_passed, total_failed = 0, []
    for module_name in modules:
        print("\n== %s ==" % module_name)
        passed, failed = run_module(module_name)
        total_passed += passed
        total_failed.extend(failed)

    print("\n" + "=" * 60)
    print("%d Tests bestanden, %d fehlgeschlagen" % (total_passed, len(total_failed)))
    for module_name, name in total_failed:
        print("  FAIL %s::%s" % (module_name, name))
    return 1 if total_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
