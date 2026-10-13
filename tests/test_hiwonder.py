import json
import math
import struct
import sys
from dataclasses import replace
from types import SimpleNamespace

import pytest

from capstone_base import cli
from capstone_base.hiwonder_i2c import (
    HiwonderConfig,
    HiwonderController,
    encode_speed_counts,
    wheel_targets_to_counts,
)


class RecordingBus:
    def __init__(self):
        self.events = []
        self.responses = {0x00: [0xE0, 0x2E], 0x3C: list(struct.pack("<4i", 1, -2, 3, -4))}
        self.fail_next = False
        self.fail_always = False
        self.closed = False

    def _record(self, event):
        self.events.append(event)
        if self.fail_next or self.fail_always:
            self.fail_next = False
            raise OSError("I2C unavailable")

    def write_byte_data(self, address, register, value):
        self._record(("byte", address, register, value))

    def write_i2c_block_data(self, address, register, data):
        self._record(("write", address, register, list(data)))

    def read_i2c_block_data(self, address, register, length):
        self._record(("read", address, register, length))
        return self.responses[register]

    def close(self):
        self.closed = True


@pytest.fixture
def config():
    # Deliberately asymmetric mapping/scales to catch wheel/channel mixups.
    return HiwonderConfig(
        motor_type=3,
        encoder_polarity=0,
        wheel_channels=(3, 1, 4, 2),
        motor_signs=(-1, 1, -1, 1),
        encoder_signs=(1, -1, 1, -1),
        counts_per_wheel_rev=(100, 200, 300, 400),
        max_speed_counts=50,
    )


@pytest.fixture
def ready(config):
    bus = RecordingBus()
    driver = HiwonderController(config, bus=bus, sleep=lambda _: None)
    driver.open()
    driver.initialize()
    bus.events.clear()
    return driver, bus


def test_initialize_order_and_delays(config):
    bus = RecordingBus()
    delays = []
    driver = HiwonderController(config, bus=bus, sleep=delays.append)
    assert bus.events == []
    driver.open()
    assert bus.events == []
    driver.initialize()
    assert bus.events == [
        ("write", 0x34, 0x33, [0, 0, 0, 0]),
        ("byte", 0x34, 0x14, 3),
        ("byte", 0x34, 0x15, 0),
        ("write", 0x34, 0x33, [0, 0, 0, 0]),
    ]
    assert delays == [0.5, 0.5]


def test_signed_encoding_boundaries():
    assert encode_speed_counts((127, -127, 0, -1), 127) == [127, 129, 0, 255]


def test_exact_four_motor_write(ready):
    driver, bus = ready
    driver.set_motor_speed_counts((10, -10, 0, 1))
    assert bus.events == [("write", 0x34, 0x33, [10, 246, 0, 1])]


def test_wheel_conversion_and_channel_mapping(config, ready):
    # One wheel revolution/second -> 1, 2, 3, 4 counts per 10ms.
    assert wheel_targets_to_counts((math.tau,) * 4, config) == (2, 4, -1, -3)
    driver, bus = ready
    driver.set_wheel_speeds((math.tau,) * 4)
    assert bus.events[-1] == ("write", 0x34, 0x33, [2, 4, 255, 253])


def test_speed_quantization(config):
    assert wheel_targets_to_counts((0.1,) * 4, config) == (0, 0, 0, 0)


@pytest.mark.parametrize(
    "values", [(51, 0, 0, 0), (-51, 0, 0, 0), (1.1, 0, 0, 0), (True, 0, 0, 0), (0, 0, 0)]
)
def test_invalid_raw_command_stops_and_latches_fault(ready, values):
    driver, bus = ready
    with pytest.raises(ValueError):
        driver.set_motor_speed_counts(values)
    assert bus.events == [("write", 0x34, 0x33, [0, 0, 0, 0])]
    with pytest.raises(RuntimeError):
        driver.set_motor_speed_counts((1, 1, 1, 1))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, 1e308, True, "fast"])
def test_invalid_wheel_target_stops(ready, value):
    driver, bus = ready
    with pytest.raises((ValueError, OverflowError)):
        driver.set_wheel_speeds((value, 0, 0, 0))
    assert bus.events[-1] == ("write", 0x34, 0x33, [0, 0, 0, 0])


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("motor_type", 0),
        ("motor_type", None),
        ("encoder_polarity", 2),
        ("encoder_polarity", True),
        ("wheel_channels", (1, 1, 3, 4)),
        ("wheel_channels", (True, 2, 3, 4)),
        ("motor_signs", (1, 0, 1, 1)),
        ("encoder_signs", (1, 1, 1)),
        ("counts_per_wheel_rev", (1, 1, 0, 1)),
        ("counts_per_wheel_rev", (1, 1, math.nan, 1)),
        ("max_speed_counts", 128),
        ("bus_number", -1),
        ("address", 0x80),
    ],
)
def test_invalid_config_rejected(config, field, value):
    with pytest.raises(ValueError):
        replace(config, **{field: value})


def test_config_copies_mutable_sequences(config):
    mapping = [1, 2, 3, 4]
    copied = replace(config, wheel_channels=mapping)
    mapping[0] = 4
    assert copied.wheel_channels == (1, 2, 3, 4)


def test_encoder_signed_little_endian_and_mapping(ready):
    driver, bus = ready
    values = (-2147483648, -2, 0x12345678, 2147483647)
    bus.responses[0x3C] = list(struct.pack("<4i", *values))
    assert driver.read_encoder_counts() == values
    assert driver.read_wheel_encoder_counts() == (0x12345678, 2147483648, 2147483647, 2)
    assert bus.events[-1] == ("read", 0x34, 0x3C, 16)


def test_voltage_bytes_and_units(ready):
    driver, bus = ready
    assert driver.read_battery_voltage() == 12.0
    assert bus.events[-1] == ("read", 0x34, 0, 2)


@pytest.mark.parametrize("reply", [[], [0] * 15, [0] * 17, [256] * 16])
def test_bad_encoder_response_trips(ready, reply):
    driver, bus = ready
    bus.responses[0x3C] = reply
    with pytest.raises(OSError, match="Malformed"):
        driver.read_encoder_counts()
    assert bus.events[-1] == ("write", 0x34, 0x33, [0, 0, 0, 0])
    with pytest.raises(RuntimeError):
        driver.set_motor_speed_counts((1, 1, 1, 1))


def test_bus_failure_stops_and_requires_reinitialize(ready):
    driver, bus = ready
    bus.fail_next = True
    with pytest.raises(OSError):
        driver.set_motor_speed_counts((1, 2, 3, 4))
    assert bus.events[-1] == ("write", 0x34, 0x33, [0, 0, 0, 0])
    with pytest.raises(RuntimeError):
        driver.set_motor_speed_counts((1, 1, 1, 1))
    driver.initialize()
    driver.set_motor_speed_counts((1, 1, 1, 1))
    assert bus.events[-1][-1] == [1, 1, 1, 1]


def test_bus_read_failure_trips(ready):
    driver, bus = ready
    bus.fail_next = True
    with pytest.raises(OSError):
        driver.read_battery_voltage()
    assert bus.events[-1][-1] == [0, 0, 0, 0]


def test_failed_stop_is_reported(ready):
    driver, bus = ready
    bus.fail_always = True
    with pytest.warns(RuntimeWarning, match="stop could not be confirmed"):
        with pytest.raises(OSError):
            driver.set_motor_speed_counts((1, 1, 1, 1))


def test_context_stops_on_keyboard_interrupt(config):
    bus = RecordingBus()
    with pytest.raises(KeyboardInterrupt):
        with HiwonderController(config, bus=bus, sleep=lambda _: None) as driver:
            driver.initialize()
            driver.set_motor_speed_counts((1, 1, 1, 1))
            raise KeyboardInterrupt
    assert bus.events[-1][-1] == [0, 0, 0, 0]
    assert not bus.closed  # Injected bus belongs to caller.
    with pytest.raises(RuntimeError):
        driver.stop()


def test_real_factory_is_lazy_and_owned_bus_closes(config, monkeypatch):
    bus = RecordingBus()
    opened = []

    def factory(number):
        opened.append(number)
        return bus

    monkeypatch.setitem(sys.modules, "smbus2", SimpleNamespace(SMBus=factory))
    driver = HiwonderController(config, sleep=lambda _: None)
    assert opened == []
    with driver:
        driver.initialize()
    assert opened == [1]
    assert bus.closed
    driver.close()  # Idempotent, no second bus operation.


def test_owned_bus_closed_even_when_stop_fails(config, monkeypatch):
    bus = RecordingBus()
    monkeypatch.setitem(sys.modules, "smbus2", SimpleNamespace(SMBus=lambda _: bus))
    driver = HiwonderController(config)
    driver.open()
    bus.fail_always = True
    with pytest.raises(OSError):
        driver.close()
    assert bus.closed


def test_motion_requires_open_and_init(config):
    driver = HiwonderController(config, bus=RecordingBus())
    with pytest.raises(RuntimeError):
        driver.set_motor_speed_counts((1, 1, 1, 1))
    driver.open()
    with pytest.raises(RuntimeError):
        driver.set_motor_speed_counts((1, 1, 1, 1))


def test_missing_optional_dependency_has_actionable_error(config, monkeypatch):
    monkeypatch.setitem(sys.modules, "smbus2", None)
    with pytest.raises(RuntimeError, match="--extra pi"):
        HiwonderController(config).open()


def test_hardware_cli_rejects_demo_before_open(monkeypatch):
    monkeypatch.setattr(cli, "HiwonderController", lambda _: pytest.fail("Hardware was opened"))
    with pytest.raises(SystemExit) as error:
        cli.main(["--backend", "hiwonder", "--duration", "0.1"])
    assert error.value.code == 2


@pytest.fixture
def hardware_file(tmp_path, config):
    from dataclasses import asdict

    path = tmp_path / "robot.json"
    path.write_text(
        json.dumps(
            {
                "simulation_only": False,
                "wheel_radius_m": 0.05,
                "wheelbase_m": 0.3,
                "track_width_m": 0.3,
                "max_wheel_rad_s": 1,
                "hiwonder": asdict(config),
            }
        )
    )
    return path


@pytest.mark.parametrize("duration", [None, "0", "-1", "3", "nan", "inf"])
def test_hardware_cli_requires_bounded_duration(hardware_file, monkeypatch, duration):
    monkeypatch.setattr(cli, "HiwonderController", lambda _: pytest.fail("Hardware was opened"))
    args = ["--backend", "hiwonder", "--config", str(hardware_file)]
    if duration is not None:
        args.extend(["--duration", duration])
    with pytest.raises(SystemExit):
        cli.main(args)


def test_hardware_cli_full_path_uses_injected_bus(hardware_file, monkeypatch):
    bus = RecordingBus()
    sleeps = []
    monkeypatch.setattr(
        cli,
        "HiwonderController",
        lambda cfg: HiwonderController(cfg, bus=bus, sleep=lambda _: None),
    )
    monkeypatch.setattr(cli.time, "sleep", sleeps.append)
    assert (
        cli.main(
            [
                "--backend",
                "hiwonder",
                "--config",
                str(hardware_file),
                "--vx",
                "0.01",
                "--duration",
                "0.1",
            ]
        )
        == 0
    )
    assert sleeps == [0.1]
    assert bus.events[-1] == ("write", 0x34, 0x33, [0, 0, 0, 0])


def test_example_config_is_incomplete_and_cannot_drive(monkeypatch):
    monkeypatch.setattr(cli, "HiwonderController", lambda _: pytest.fail("Hardware was opened"))
    with pytest.raises(SystemExit):
        cli.main(
            [
                "--backend",
                "hiwonder",
                "--duration",
                "0.1",
                "--config",
                "config/hiwonder-v1.3.example.json",
            ]
        )
