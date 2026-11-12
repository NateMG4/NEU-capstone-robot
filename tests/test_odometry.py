import math
from dataclasses import replace

import pytest

from capstone_base.hiwonder_i2c import HiwonderConfig
from capstone_base.kinematics import Geometry
from capstone_base.odometry import (
    EncoderOdometry,
    EncoderSampleError,
    Pose,
    ReferenceFrame,
    counter_delta,
    integrate,
)

GEOMETRY = Geometry(0.05, 0.3, 0.3)
CONFIG = HiwonderConfig(
    motor_type=3,
    encoder_polarity=0,
    wheel_channels=(3, 1, 4, 2),
    encoder_signs=(1, -1, -1, 1),
    motor_signs=(1, -1, -1, 1),
    counts_per_wheel_rev=(1000, 2000, 3000, 4000),
    max_speed_counts=30,
)


def raw_for_wheel_revolutions(revolutions):
    raw = [0] * 4
    for rev, channel, sign, counts in zip(
        revolutions,
        CONFIG.wheel_channels,
        CONFIG.encoder_signs,
        CONFIG.counts_per_wheel_rev,
        strict=True,
    ):
        raw[channel - 1] = round(rev * sign * counts)
    return tuple(raw)


@pytest.mark.parametrize(
    ("revs", "expected"),
    [
        ((0.1, 0.1, 0.1, 0.1), (math.pi / 100, 0, 0)),
        ((-0.1, 0.1, 0.1, -0.1), (0, math.pi / 100, 0)),
        ((-0.1, 0.1, -0.1, 0.1), (0, 0, math.pi / 3 / 10)),
    ],
)
def test_asymmetric_mapping_scales_and_motion_patterns(revs, expected):
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    odo.update((0, 0, 0, 0), 10)
    odo.update(raw_for_wheel_revolutions(revs), 10.1)
    assert (odo.pose.x, odo.pose.y, odo.pose.yaw) == pytest.approx(expected)
    assert odo.wheel_rad_s == pytest.approx(tuple(rev * math.tau / 0.1 for rev in revs))


def test_nonzero_initial_counts_are_a_baseline():
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    odo.update((100, -234, 500, 130), 1)
    assert odo.pose == Pose()
    assert odo.delta_counts == (0, 0, 0, 0)


def test_wrap_both_directions_and_normalized_count_deltas():
    assert counter_delta(-2147483647, 2147483646) == 3
    assert counter_delta(2147483646, -2147483647) == -3
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    before = [2147483646] * 4
    after = [-2147483647] * 4
    odo.update(before, 0)
    odo.update(after, 0.1)
    assert odo.delta_counts == (3, -3, -3, 3)


def test_se2_arc_and_rotation_into_world():
    # Forward one meter over a 90 degree turn follows a constant-twist arc.
    p = integrate(Pose(), 1, 0, math.pi / 2)
    assert (p.x, p.y, p.yaw) == pytest.approx((2 / math.pi, 2 / math.pi, math.pi / 2))
    p = integrate(Pose(0, 0, math.pi / 2), 1, 0, 0)
    assert (p.x, p.y) == pytest.approx((0, 1))


@pytest.mark.parametrize("now", [0, -1, 0.51, math.nan])
def test_bad_clock_or_stale_sample_rejected_without_integrating(now):
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    odo.update((0, 0, 0, 0), 0)
    with pytest.raises(ValueError):
        odo.update((1, 1, 1, 1), now)
    assert odo.pose == Pose()
    assert odo.raw == (0, 0, 0, 0)


@pytest.mark.parametrize("counts", [(1, 2), (True, 0, 0, 0), (2**31, 0, 0, 0)])
def test_invalid_samples(counts):
    with pytest.raises(ValueError):
        EncoderOdometry(GEOMETRY, CONFIG).update(counts, 0)


def test_jump_reset_and_missing_calibration():
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    odo.update((0, 0, 0, 0), 1)
    with pytest.raises(ValueError, match="jump"):
        odo.update((100000, 0, 0, 0), 1.05)
    assert odo.pose == Pose()
    odo.reset(Pose(1, 2, 0))
    odo.update((100000, 0, 0, 0), 2)
    assert odo.pose == Pose(1, 2, 0)
    with pytest.raises(ValueError):
        EncoderOdometry(GEOMETRY, replace(CONFIG, encoder_signs=None))


def test_history_and_absolute_alignment_preserve_since_capture_motion():
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    odo.update((0, 0, 0, 0), 1)
    odo.update(raw_for_wheel_revolutions((0.1,) * 4), 1.1)
    captured = odo.pose_at(1.05)
    reference = ReferenceFrame()
    reference.align(Pose(4, 5, math.pi / 2), captured, "room")
    aligned = reference.transform(odo.pose)
    assert (aligned.x, aligned.y) == pytest.approx((4, 5 + math.pi / 200))
    assert odo.pose.x == pytest.approx(math.pi / 100)  # Local odom is continuous.
    with pytest.raises(ValueError):
        odo.pose_at(0.99)
    with pytest.raises(ValueError):
        reference.align(Pose(), Pose(), "odom")


def test_jump_has_raw_evidence_for_the_correct_physical_wheel():
    odo = EncoderOdometry(GEOMETRY, CONFIG, max_wheel_rad_s=20)
    odo.update((10, 20, 30, 40), 1)
    with pytest.raises(EncoderSampleError) as caught:
        odo.update((10, 20, 5030, 40), 1.05)
    report = caught.value.details
    assert report["kind"] == "encoder_jump"
    assert report["raw_previous"] == (10, 20, 30, 40)
    assert report["raw_current"] == (10, 20, 5030, 40)
    assert report["dt_s"] == pytest.approx(0.05)
    fl = report["wheels"][0]
    assert (fl["wheel"], fl["channel"], fl["delta_counts"]) == ("FL", 3, 5000)
    assert fl["counts_per_wheel_rev"] == 1000
    assert fl["allowed_delta_counts"] == pytest.approx(20 * 0.05 * 1000 / math.tau + 2)
    assert fl["inferred_rad_s"] == pytest.approx(5000 * math.tau / 1000 / 0.05)
    assert [w["wheel"] for w in report["wheels"] if w["rejected"]] == ["FL"]
    assert odo.raw == (10, 20, 30, 40)
    assert odo.pose == Pose()


def test_rebaseline_discards_gap_but_preserves_valid_pose_and_wheel_travel():
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    odo.update((0, 0, 0, 0), 1)
    odo.update(raw_for_wheel_revolutions((0.1,) * 4), 1.1)
    pose, distances = odo.pose, odo.wheel_distances
    odo.rebaseline((5000, 5000, 5000, 5000), 20)
    assert odo.pose == pose
    assert odo.wheel_distances == distances
    assert odo.wheel_rad_s == (0, 0, 0, 0)
    assert list(odo.history) == [(20, pose)]
    odo.update((5000, 5000, 5000, 5000), 20.05)
    assert odo.pose == pose
    with pytest.raises(ValueError):
        odo.pose_at(1.1)


@pytest.mark.parametrize(("delta", "accepted"), [(159, True), (162, False)])
def test_guard_in_count_units_keeps_the_original_speed_threshold(delta, accepted):
    odo = EncoderOdometry(GEOMETRY, CONFIG, max_wheel_rad_s=20)
    odo.update((0, 0, 0, 0), 0)
    raw = (0, 0, delta, 0)  # M3 is FL with 1000 counts/rev.
    if accepted:
        odo.update(raw, 0.05)
    else:
        with pytest.raises(EncoderSampleError):
            odo.update(raw, 0.05)


def test_stale_sample_report_keeps_timing_evidence():
    odo = EncoderOdometry(GEOMETRY, CONFIG)
    odo.update((0, 0, 0, 0), 0)
    with pytest.raises(EncoderSampleError) as caught:
        odo.update((1, 2, 3, 4), 2)
    assert caught.value.details["kind"] == "sample_timing"
    assert caught.value.details["dt_s"] == 2
