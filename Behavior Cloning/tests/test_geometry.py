"""Geometrie: Quaternionen, Posen, der +/-pi-Umschlagpunkt (AP 2.4)."""

import math

import _paths  # noqa: F401

import numpy as np

from bc import geometry


def test_rpy_quat_roundtrip():
    rpy = (0.3, -0.7, 1.2)
    q = geometry.quat_from_rpy(*rpy)
    back = geometry.quat_to_rpy(q)
    assert np.allclose(rpy, back, atol=1e-9)


def test_pi_wraparound_is_same_orientation():
    # Die realen Arbeitsposen liegen exakt am +/-pi-Umschlagpunkt
    # (tools/log.txt) -- genau deshalb rechnet das Projekt in Quaternionen.
    pose_a = np.array([0.44, -0.09, 0.43, -math.pi + 1e-4, 0.0, -math.pi + 1e-4])
    pose_b = np.array([0.44, -0.09, 0.43, math.pi - 1e-4, 0.0, math.pi - 1e-4])
    qa = geometry.pose_rpy_to_quat(pose_a)
    qb = geometry.pose_rpy_to_quat(pose_b)
    assert geometry.quat_angle_between(qa[3:7], qb[3:7]) < 1e-3
    # ... waehrend naive RPY-Differenz einen 2*pi-Sprung zeigt:
    assert float(np.max(np.abs(pose_b[3:] - pose_a[3:]))) > 6.0


def test_slerp_stays_put_at_wraparound():
    qa = geometry.pose_rpy_to_quat([0, 0, 0, -math.pi + 1e-4, 0, -math.pi + 1e-4])
    qb = geometry.pose_rpy_to_quat([0, 0, 0, math.pi - 1e-4, 0, math.pi - 1e-4])
    mid = geometry.slerp(qa[3:7], qb[3:7], 0.5)
    assert geometry.quat_angle_between(mid, qa[3:7]) < 1e-3


def test_matrix_roundtrip():
    rng = np.random.default_rng(42)
    for _ in range(50):
        axis = rng.standard_normal(3)
        angle = rng.uniform(-np.pi, np.pi)
        q = geometry.quat_from_axis_angle(axis, angle)
        R = geometry.quat_to_matrix(q)
        q2 = geometry.matrix_to_quat(R)
        # acos verstaerkt Rundungsfehler nahe der Identitaet -> 1e-6 rad
        # ist die realistische (und voellig ausreichende) Toleranz
        assert geometry.quat_angle_between(q, q2) < 1e-6
        # Rotationsmatrix-Eigenschaften
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert abs(np.linalg.det(R) - 1.0) < 1e-12


def test_quat_rotate_matches_matrix():
    q = geometry.quat_from_rpy(0.2, -0.4, 0.9)
    v = np.array([0.3, -0.1, 0.7])
    assert np.allclose(geometry.quat_rotate(q, v), geometry.quat_to_matrix(q) @ v)


def test_pose_interpolation():
    p0 = geometry.pose_rpy_to_quat([0, 0, 0, 0, 0, 0])
    p1 = geometry.pose_rpy_to_quat([1, 2, 3, 0, 0, 0])
    half = geometry.pose_interpolate(p0, p1, 0.5)
    assert np.allclose(half[:3], [0.5, 1.0, 1.5])

    path = geometry.pose_path([p0, p1], steps_per_segment=10)
    assert path.shape == (11, 7)
    assert np.allclose(np.linalg.norm(path[:, 3:7], axis=1), 1.0, atol=1e-9)
