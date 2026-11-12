from dataclasses import replace

import pytest

from capstone_base.servo import (
    FakeServoController,
    ServoConfig,
    ServoStatus,
    validate_feedback,
    validate_status,
)


@pytest.mark.parametrize(
    "settings",
    [
        {"servo_id": True},
        {"servo_id": 254},
        {"servo_id": -1},
        {"port": ""},
        {"port": 42},
        {"baudrate": 1000000},
        {"timeout_s": 0},
        {"write_timeout_s": float("inf")},
        {"min_angle_deg": 0},
        {"min_angle_deg": 120, "max_angle_deg": 100},
        {"min_angle_deg": -1, "max_angle_deg": 240},
        {"max_step_deg": 241},
        {"max_step_deg": float("nan")},
        {"min_duration_s": 0},
        {"max_duration_s": 31},
        {"position_tolerance_deg": 0},
        {"max_temperature_c": True},
    ],
)
def test_invalid_config(settings):
    with pytest.raises(ValueError):
        ServoConfig(**settings)


def test_status_without_limits_and_fake_lifecycle():
    fake = FakeServoController(ServoConfig())
    with pytest.raises(RuntimeError):
        fake.read_status()
    with fake:
        assert fake.read_position_deg() == 120
        assert fake.read_voltage_v() == 7.4
        assert fake.read_temperature_c() == 25
        with pytest.raises(ValueError, match="measured"):
            fake.initialize()
    assert fake.history == []


def test_fake_move_and_stop_preserve_torque():
    config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
    fake = FakeServoController(config)
    with fake:
        fake.initialize()
        fake.move_to(122, 1)
        assert fake.read_position_deg() == pytest.approx(121.92)
    assert fake.history == [("move", 508, 1000), ("stop",)]
    assert fake.status.torque_enabled


@pytest.mark.parametrize(
    "angle,duration",
    [(float("nan"), 1), (float("inf"), 1), (114, 1), (126, 1), (120, 0), (120, 3), (True, 1)],
)
def test_move_validation(angle, duration):
    config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
    with pytest.raises(ValueError):
        config.validate_move(angle, duration)


def test_quantization_must_not_cross_mechanical_limit():
    config = ServoConfig(min_angle_deg=120.1, max_angle_deg=121)
    with pytest.raises(ValueError, match="quantized angle"):
        config.validate_move(120.1, 1)


def test_configurable_larger_slower_moves_still_respect_angle_limits():
    config = ServoConfig(min_angle_deg=90, max_angle_deg=150, max_step_deg=60, max_duration_s=30)
    with FakeServoController(config) as controller:
        controller.initialize()
        controller.move_to(144, 30)
        assert controller.read_position_deg() == 144
        with pytest.raises(ValueError, match="angle_deg"):
            controller.move_to(160, 10)


@pytest.mark.parametrize(
    "changes",
    [
        {"mode": 1},
        {"voltage_v": 12},
        {"voltage_v": 5.9},
        {"temperature_c": 61},
        {"position_deg": 130},
        {"torque_enabled": False},
    ],
)
def test_status_rejects_motion(changes):
    config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
    status = replace(ServoStatus(120, 7.4, 25, 0, True), **changes)
    with pytest.raises(ValueError):
        validate_status(status, config, require_torque=True)


def test_health_checks_for_recovery_are_independent_of_calibration_and_torque():
    status = ServoStatus(40.08, 7.4, 25, 0, False)
    for config in (ServoConfig(), ServoConfig(min_angle_deg=115, max_angle_deg=125)):
        validate_feedback(status, config)
        with pytest.raises(ValueError):
            validate_status(status, config)
        for changes in ({"mode": 1}, {"voltage_v": 12}, {"temperature_c": 70}):
            with pytest.raises(ValueError):
                validate_feedback(replace(status, **changes), config)


def test_fake_id_assignment_requires_reinitialization_and_records_change():
    fake = FakeServoController(ServoConfig(min_angle_deg=115, max_angle_deg=125))
    with pytest.raises(RuntimeError):
        fake.set_new_id(2)
    with fake:
        fake.initialize()
        with pytest.raises(ValueError):
            fake.set_new_id(True)
        fake.set_new_id(2)
        assert fake.read_id() == 2
        assert fake.history == [("id", 1, 2)]
        with pytest.raises(RuntimeError, match="Initialize"):
            fake.move_to(122, 1)
        fake.initialize()
        fake.move_to(122, 1)
        with pytest.raises(RuntimeError, match="Stop"):
            fake.set_new_id(3)
