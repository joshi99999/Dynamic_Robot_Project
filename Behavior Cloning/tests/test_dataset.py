"""Datensatz-Schema und Ablage (AP 0.9 Punkt 1/2, AP 2.3)."""

import _paths  # noqa: F401

import numpy as np

from bc import config, dataset


def _dummy_episode(n=5, cameras=("wrist", "scene")):
    arrays = {
        "observation.state": np.zeros((n, config.STATE_DIM), dtype=np.float32),
        "action": np.zeros((n, config.ACTION_DIM), dtype=np.float32),
        "aux.joints_ideal": np.zeros((n, 6), dtype=np.float32),
        "aux.joints_command": np.zeros((n, 6), dtype=np.float32),
        "aux.pose_ideal": np.zeros((n, 7), dtype=np.float32),
        "aux.pose_noisy": np.zeros((n, 7), dtype=np.float32),
        "aux.sync_ok": np.ones((n, 1), dtype=bool),
        "next.done": np.zeros((n, 1), dtype=bool),
    }
    for name in cameras:
        arrays["observation.images.%s" % name] = np.zeros(
            (n, config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3), dtype=np.uint8
        )
    return dataset.Episode(arrays=arrays, metadata={"mode": "test"})


def test_state_layout():
    joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    tcp = [1, 2, 3, 1, 0, 0, 0]
    state = dataset.build_state(joints, tcp, config.GRIPPER_CLOSED)
    assert state.shape == (config.STATE_DIM,)
    assert state.dtype == np.float32
    parts = dataset.split_state(state)
    assert np.allclose(parts["joints"], joints)
    assert np.allclose(parts["tcp_pos"], tcp[:3])
    assert np.allclose(parts["tcp_quat"], tcp[3:])
    assert parts["gripper"][0] == config.GRIPPER_CLOSED


def test_state_quaternion_is_canonical():
    # Schema 2: q und -q (gleiche Orientierung) ergeben denselben State --
    # sonst lernt die Policy zwei scheinbar verschiedene Zustaende.
    # Pose wie PICK in der VM: Greifer unten, w ~ 0 (2026-09-14).
    q = np.array([-0.025, 0.001, -1.0, -0.002])
    q /= np.linalg.norm(q)
    pose_a = np.concatenate([[0.45, 0.0, 0.17], q])
    pose_b = np.concatenate([[0.45, 0.0, 0.17], -q])
    joints = [0.0] * 6
    state_a = dataset.build_state(joints, pose_a, config.GRIPPER_OPEN)
    state_b = dataset.build_state(joints, pose_b, config.GRIPPER_OPEN)
    assert np.array_equal(state_a, state_b)
    # Die Wahl haengt an der Referenz, nicht an w: y-Komponente positiv
    assert dataset.split_state(state_a)["tcp_quat"][2] > 0


def test_action_layout_and_threshold():
    action = dataset.build_action([0.1] * 6, config.GRIPPER_CLOSED)
    assert action.shape == (config.ACTION_DIM,)
    assert dataset.gripper_from_action(action) is True
    action_open = dataset.build_action([0.1] * 6, config.GRIPPER_OPEN)
    assert dataset.gripper_from_action(action_open) is False


def test_episode_validation_catches_shape_errors():
    ep = _dummy_episode()
    ep.validate()

    bad = _dummy_episode()
    bad.arrays["action"] = bad.arrays["action"][:, :5]
    try:
        bad.validate()
        assert False, "ValueError erwartet"
    except ValueError:
        pass

    bad2 = _dummy_episode()
    bad2.arrays["observation.state"] = bad2.arrays["observation.state"].astype(
        np.float64
    )
    try:
        bad2.validate()
        assert False, "ValueError erwartet (dtype)"
    except ValueError:
        pass


def test_writer_roundtrip(tmp_path):
    root = tmp_path / "ds"
    writer = dataset.DatasetWriter(root)
    ep = _dummy_episode()
    ep.arrays["observation.state"][2, 0] = 0.777
    ep.success = True
    idx = writer.append(ep)
    assert idx == 0

    loaded = dataset.load_episode(root, 0)
    assert len(loaded) == len(ep)
    assert loaded.success is True
    assert abs(loaded.arrays["observation.state"][2, 0] - 0.777) < 1e-6
    assert loaded.metadata["mode"] == "test"
    assert set(loaded.arrays) == set(ep.arrays)


def test_writer_rejects_other_schema_version(tmp_path):
    import json

    root = tmp_path / "ds"
    dataset.DatasetWriter(root)
    index = json.loads((root / "index.json").read_text(encoding="utf-8"))
    index["schema_version"] = 999
    (root / "index.json").write_text(json.dumps(index), encoding="utf-8")
    try:
        dataset.DatasetWriter(root)
        assert False, "ValueError erwartet (Schema-Version)"
    except ValueError:
        pass


def test_writer_sets_aside_a_left_over_episode_folder(tmp_path):
    # Befund 27.10.2026: ein abgebrochener Lauf hinterliess ep_00000 ohne
    # Indexeintrag, der naechste Lauf scheiterte nach der ersten Episode
    # mit FileExistsError.
    root = tmp_path / "ds"
    dataset.DatasetWriter(root)
    (root / "ep_00000").mkdir()
    (root / "ep_00000" / "arrays.npz").write_bytes(b"halb")
    (root / "ep_00001.partial").mkdir()

    writer = dataset.DatasetWriter(root)
    assert [old for old, _new in writer.set_aside] == ["ep_00000", "ep_00001.partial"]
    for _old, new in writer.set_aside:
        assert (root / new).is_dir()      # beiseitegelegt, nicht geloescht

    ep = _dummy_episode()
    ep.success = True
    assert writer.append(ep) == 0
    assert dataset.load_episode(root, 0).success is True
    assert not list(root.glob("*.partial"))


def test_writer_keeps_known_episodes(tmp_path):
    root = tmp_path / "ds"
    dataset.DatasetWriter(root).append(_dummy_episode())
    writer = dataset.DatasetWriter(root)
    assert writer.set_aside == []
    assert writer.append(_dummy_episode()) == 1


def test_relabel_changes_only_the_meta_file(tmp_path):
    import json

    root = tmp_path / "ds"
    ep = _dummy_episode()
    ep.success = True
    dataset.DatasetWriter(root).append(ep)

    dataset.relabel(root, "ep_00000", "fehler")
    loaded = dataset.load_episode(root, 0)
    assert loaded.success is False and loaded.discarded is False

    meta = dataset.relabel(root, "ep_00000", "verwerfen", reason="Objekt verrutscht")
    assert meta["discarded"] is True and meta["success"] is None
    assert meta["discard_reason"] == "Objekt verrutscht"
    # Die erste Bewertung bleibt nachvollziehbar.
    assert meta["relabeled_from"]["success"] is True
    index = json.loads((root / "index.json").read_text(encoding="utf-8"))
    assert index["episodes"][0]["discarded"] is False
