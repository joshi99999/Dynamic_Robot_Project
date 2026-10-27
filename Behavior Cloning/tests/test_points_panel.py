"""Punkte-Liste, Anfahren und Metadaten-Auswahl des Aufnahme-Reiters (Anwender 2026-10-26).

Qt-frei: geprueft wird bc/gui/points.py und bc/gui/vocabulary.py, dazu
``NeuraRobot.prepare_motion`` fuer "Anfahren" aus der Oberflaeche.
"""

import json

import _paths  # noqa: F401

import numpy as np

from bc.adapters.neura import NeuraRobot
from bc.gui import points as points_module
from bc.gui import vocabulary
from bc.ports import MotionRefused

from test_computed_points import COMPUTED, _TeachClient


def _steps():
    return [dict({"above": None, "dz": 0.0, "gripper": None}, **s) for s in COMPUTED["sequence"]]


def test_one_row_per_point_with_what_to_do():
    rows = {r["name"]: r for r in points_module.point_rows(_steps())}
    assert list(rows) == ["CLEAR_FOV", "ANFLUG", "PRE_GRASP", "PICK", "PRE_PLACE"]
    assert rows["PICK"]["scope"] == "je Lage" and rows["PICK"]["gripper"] == "close"
    assert rows["CLEAR_FOV"]["scope"] == rows["PRE_PLACE"]["scope"] == "einmal"
    assert rows["PRE_GRASP"]["kind"] == points_module.COMPUTED
    assert rows["PRE_GRASP"]["above"] == "PICK" and rows["PRE_GRASP"]["dz"] == 0.05
    assert rows["PRE_GRASP"]["gripper"] == "mehrfach"  # kommt zweimal vor


def _resolved_points():
    names = {"CLEAR_FOV": 0.0, "ANFLUG": 1.0, "PRE_GRASP": 2.0, "PICK": 3.0, "PRE_PLACE": 4.0}
    return {n: {"joints": [v] * 6} for n, v in names.items()}


def test_anfahren_comes_from_above_and_leaves_upwards():
    computed = {"ANFLUG": ("PRE_GRASP", 0.1), "PRE_GRASP": ("PICK", 0.05)}
    points = _resolved_points()
    path = points_module.approach_path
    assert points_module.chain_above("PICK", computed) == ["PRE_GRASP", "ANFLUG"]
    # vom Start zu PICK: senkrecht von oben
    assert path("PICK", points, computed, [0.0] * 6) == ["ANFLUG", "PRE_GRASP", "PICK"]
    # von PICK weg: erst hinauf
    assert path("PRE_PLACE", points, computed, [3.0] * 6) == ["PRE_GRASP", "ANFLUG", "PRE_PLACE"]
    assert path("CLEAR_FOV", points, computed, [3.0] * 6) == ["PRE_GRASP", "ANFLUG", "CLEAR_FOV"]
    # schon unterwegs nach unten / nur hinauf / schon da
    assert path("PICK", points, computed, [2.0] * 6) == ["PICK"]
    assert path("PRE_GRASP", points, computed, [3.0] * 6) == ["PRE_GRASP"]
    assert path("PICK", points, computed, [3.0] * 6) == []
    # irgendwo dazwischen: direkt bzw. von oben
    assert path("PRE_PLACE", points, computed, [9.0] * 6) == ["PRE_PLACE"]
    # ohne berechnete Punkte wie bisher direkt
    assert path("PICK", points, {}, [0.0] * 6) == ["PICK"]


def test_vocabulary_keeps_one_spelling(tmp_path):
    known = vocabulary.load(tmp_path)
    assert "Decke an" in known["light"]
    values = vocabulary.remember(tmp_path, {"light": "  decke   AN ", "camera_pose": "",
                                            "object_note": "Teil A, links"})
    assert values["light"] == "Decke an"          # bekannte Schreibweise
    assert values["camera_pose"] is None
    stored = json.loads((tmp_path / vocabulary.VOCABULARY_FILE).read_text(encoding="utf-8"))
    assert stored["object_note"] == ["Teil A, links"]
    assert stored["light"].count("Decke an") == 1
    # beim naechsten Mal als Auswahl da, auch anders geschrieben derselbe Wert
    assert vocabulary.remember(tmp_path, {"object_note": "teil a,  LINKS"})["object_note"] == "Teil A, links"
    assert vocabulary.load(tmp_path)["object_note"] == ["Teil A, links"]


def test_prepare_motion_switches_to_automatic_and_powers_on():
    client = _TeachClient(teach=True)
    robot = NeuraRobot(robot=client).connect()      # lesend, wie die Oberflaeche
    robot.emergency_stop()
    assert robot.stop_requested
    robot.prepare_motion()
    names = [c[0] for c in client.calls]
    assert client.teach is False and not robot.stop_requested
    assert names.index("switch_to_automatic_mode") < names.index("init_program") \
        < names.index("power_on")
    robot.move_to_joints(np.asarray(client.joints) + 0.0)  # nimmt Bewegung an

    real = NeuraRobot(robot=_TeachClient(in_simulation=False)).connect()
    try:
        real.prepare_motion()
        assert False, "MotionRefused erwartet"
    except MotionRefused:
        pass
