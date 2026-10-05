"""Unit tests for reach_math.py -- runs anywhere plain Python does, no
Isaac Sim needed."""

from __future__ import annotations

import pytest
from reach_math import (
    FLEXION_RANGE_DEG,
    LINEAR_RANGE_M,
    ROTATION_RANGE_DEG,
    JointTargets,
    build_observation,
    compute_reward,
    integrate_targets,
    scale_action,
)


def test_scale_action_at_full_deflection_hits_max_rate():
    linear, rotation, flexion = scale_action((1.0, -1.0, 1.0))
    assert linear == pytest.approx(0.002)
    assert rotation == pytest.approx(-30.0)
    assert flexion == pytest.approx(35.0)


def test_scale_action_clamps_out_of_range_input():
    linear, rotation, flexion = scale_action((5.0, -5.0, 0.0))
    assert linear == pytest.approx(0.002)
    assert rotation == pytest.approx(-30.0)
    assert flexion == pytest.approx(0.0)


def test_integrate_targets_moves_by_rate_times_dt():
    start = JointTargets(linear_m=0.0, rotation_deg=0.0, flexion_deg=0.0)
    result = integrate_targets(start, rates=(0.002, 30.0, 35.0), dt_s=1.0)
    assert result.linear_m == pytest.approx(0.002)
    assert result.rotation_deg == pytest.approx(30.0)
    assert result.flexion_deg == pytest.approx(35.0)


def test_integrate_targets_clamps_to_real_ranges():
    start = JointTargets(linear_m=LINEAR_RANGE_M[1] - 0.0001)
    result = integrate_targets(start, rates=(0.002, 0.0, 0.0), dt_s=1.0)
    assert result.linear_m == pytest.approx(LINEAR_RANGE_M[1])

    start2 = JointTargets(rotation_deg=ROTATION_RANGE_DEG[0] + 0.1)
    result2 = integrate_targets(start2, rates=(0.0, -30.0, 0.0), dt_s=1.0)
    assert result2.rotation_deg == pytest.approx(ROTATION_RANGE_DEG[0])

    start3 = JointTargets(flexion_deg=FLEXION_RANGE_DEG[1] - 0.1)
    result3 = integrate_targets(start3, rates=(0.0, 0.0, 35.0), dt_s=1.0)
    assert result3.flexion_deg == pytest.approx(FLEXION_RANGE_DEG[1])


def test_compute_reward_is_negative_distance_when_far():
    reward, success = compute_reward((0.0, 0.0, 0.0), (0.03, 0.04, 0.0))
    assert reward == pytest.approx(-0.05)  # 3-4-5 triangle
    assert success is False


def test_compute_reward_success_within_tolerance():
    reward, success = compute_reward((0.0, 0.0, 0.0), (0.001, 0.001, 0.001))
    assert success is True
    assert reward > 0  # distance penalty outweighed by the success bonus


def test_compute_reward_exact_hit():
    reward, success = compute_reward((1.0, 2.0, 3.0), (1.0, 2.0, 3.0))
    assert success is True
    assert reward == pytest.approx(10.0)


def test_build_observation_centre_of_range_is_zero():
    mid_targets = JointTargets(
        linear_m=(LINEAR_RANGE_M[0] + LINEAR_RANGE_M[1]) / 2.0,
        rotation_deg=(ROTATION_RANGE_DEG[0] + ROTATION_RANGE_DEG[1]) / 2.0,
        flexion_deg=(FLEXION_RANGE_DEG[0] + FLEXION_RANGE_DEG[1]) / 2.0,
    )
    obs = build_observation(mid_targets, tip_pos=(1.0, 2.0, 3.0), target_pos=(1.0, 2.0, 3.0))
    assert obs[0] == pytest.approx(0.0)
    assert obs[1] == pytest.approx(0.0)
    assert obs[2] == pytest.approx(0.0)
    assert obs[3:] == pytest.approx((0.0, 0.0, 0.0))


def test_build_observation_at_range_extremes_is_plus_minus_one():
    max_targets = JointTargets(
        linear_m=LINEAR_RANGE_M[1],
        rotation_deg=ROTATION_RANGE_DEG[1],
        flexion_deg=FLEXION_RANGE_DEG[1],
    )
    obs = build_observation(max_targets, tip_pos=(0.0, 0.0, 0.0), target_pos=(0.0, 0.0, 0.0))
    assert obs[0] == pytest.approx(1.0)
    assert obs[1] == pytest.approx(1.0)
    assert obs[2] == pytest.approx(1.0)


def test_build_observation_reports_raw_offset_in_meters():
    obs = build_observation(JointTargets(), tip_pos=(0.1, 0.2, 0.3), target_pos=(0.0, 0.0, 0.0))
    assert obs[3:] == pytest.approx((0.1, 0.2, 0.3))
