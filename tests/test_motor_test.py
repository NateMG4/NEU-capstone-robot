import pytest
from test_hiwonder import RecordingBus

from capstone_base import motor_test
from capstone_base.hiwonder_i2c import HiwonderConfig, HiwonderController, wheel_targets_to_counts


def test_raw_config_needs_no_wheel_calibration():
    config = HiwonderConfig(motor_type=3, encoder_polarity=0)
    bus = RecordingBus()
    with HiwonderController(config, bus=bus, sleep=lambda _: None) as controller:
        controller.initialize()
        controller.set_motor_speed_counts((2, 0, 0, 0))
        assert bus.events[-1] == ("write", 0x34, 0x33, [2, 0, 0, 0])
        assert controller.read_encoder_counts() == (1, -2, 3, -4)
        with pytest.raises(RuntimeError, match="calibration"):
            controller.read_wheel_encoder_counts()
    with pytest.raises(ValueError, match="calibration"):
        wheel_targets_to_counts((1, 1, 1, 1), config)


def test_single_channel_command_stops(monkeypatch, capsys):
    bus = RecordingBus()
    sleeps = []
    monkeypatch.setattr(
        motor_test,
        "HiwonderController",
        lambda cfg: HiwonderController(cfg, bus=bus, sleep=lambda _: None),
    )
    monkeypatch.setattr(motor_test.time, "sleep", sleeps.append)
    assert (
        motor_test.main(
            ["--motor-type", "3", "--channel", "2", "--speed", "-2", "--duration", "0.25"]
        )
        == 0
    )
    assert ("write", 0x34, 0x33, [0, 254, 0, 0]) in bus.events
    assert bus.events[-1] == ("write", 0x34, 0x33, [0, 0, 0, 0])
    assert sleeps == [0.25]
    assert "Delta M1-M4" in capsys.readouterr().out


@pytest.mark.parametrize(
    "extra",
    [
        ["--speed", "11"],
        ["--speed", "-11"],
        ["--duration", "0"],
        ["--duration", "nan"],
        ["--duration", "2"],
        ["--channel", "5"],
    ],
)
def test_invalid_test_settings_never_open_bus(monkeypatch, extra):
    monkeypatch.setattr(motor_test, "HiwonderController", lambda _: pytest.fail("Bus opened"))
    with pytest.raises(SystemExit):
        motor_test.main(["--motor-type", "3", "--channel", "1", "--duration", "0.25", *extra])
