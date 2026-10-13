import math
import os
import select
import signal
import subprocess
import sys
import termios
import time
from pathlib import Path

import pytest

from capstone_base.hiwonder_i2c import HiwonderConfig
from capstone_base.keyboard_teleop import TerminalInput, main, run_loop
from capstone_base.teleop import ZERO, FakeMotorOutput, KeyDecoder, TeleopState, mix_direction


@pytest.fixture
def config():
    return HiwonderConfig(
        motor_type=3, encoder_polarity=0, wheel_channels=(1, 3, 2, 4), motor_signs=(1, -1, -1, 1)
    )


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        ("w", (5, -5, -5, 5)),
        ("s", (-5, 5, 5, -5)),
        ("a", (-5, -5, -5, -5)),
        ("d", (5, 5, 5, 5)),
        ("q", (-5, 5, -5, 5)),
        ("e", (5, -5, 5, -5)),
        ("7", (0, -5, -5, 0)),
        ("9", (5, 0, 0, 5)),
        ("1", (-5, 0, 0, -5)),
        ("3", (0, 5, 5, 0)),
    ],
)
def test_direction_mapping(config, key, expected):
    state = TeleopState(config, acceleration=1000)
    state.accept(key, 0)
    assert state.step(0.05) == expected


def test_initial_zero_and_lease_expiry(config):
    state = TeleopState(config, acceleration=1000)
    assert state.step(0.01) == ZERO
    state.accept("w", 0.01)
    assert state.step(0.1) != ZERO
    assert state.step(0.31) == ZERO
    assert state.step(0.4) == ZERO
    # A fresh key is required after a stop, rather than resuming old motion.
    state.accept("w", 0.4)
    assert state.step(0.45) != ZERO


def test_repeated_input_extends_lease(config):
    state = TeleopState(config, acceleration=1000)
    state.accept("w", 0)
    state.step(0.1)
    state.accept("w", 0.2)
    assert state.step(0.4) != ZERO
    assert state.step(0.51) == ZERO


def test_stop_bypasses_acceleration(config):
    state = TeleopState(config, acceleration=10)
    state.accept("w", 0)
    assert state.step(0.1) == (1, -1, -1, 1)
    state.accept("stop", 0.11)
    assert state.step(0.11) == ZERO


def test_acceleration_and_reversal_are_bounded(config):
    state = TeleopState(config, acceleration=20)
    state.accept("w", 0)
    first = state.step(0.05)
    second = state.step(0.1)
    assert first == (1, -1, -1, 1)
    assert second == (2, -2, -2, 2)
    state.accept("s", 0.1)
    assert state.step(0.15) == first
    assert state.step(0.2) == ZERO
    assert state.step(0.25) == (-1, 1, 1, -1)


def test_diagonal_and_combined_mixer_limits():
    assert mix_direction((1, 1, 0), 5) == (0, 5, 5, 0)
    assert max(abs(v) for v in mix_direction((1, 1, 1), 10)) == 10


@pytest.mark.parametrize("now", [-1, math.nan, math.inf])
def test_bad_clock_clears_motion(config, now):
    state = TeleopState(config)
    state.accept("w", 0)
    with pytest.raises(ValueError):
        state.step(now)
    assert state.wheels == ZERO


@pytest.mark.parametrize(
    "kwargs",
    [{"speed": 11}, {"speed": True}, {"timeout": 0}, {"timeout": math.nan}, {"acceleration": -1}],
)
def test_invalid_settings_rejected(config, kwargs):
    with pytest.raises(ValueError):
        TeleopState(config, **kwargs)


def test_mapping_required():
    with pytest.raises(ValueError, match="wheel_channels"):
        TeleopState(HiwonderConfig(motor_type=3, encoder_polarity=0))


def test_split_arrows_and_modified_sequences():
    decoder = KeyDecoder()
    assert decoder.feed(b"\x1b") == []
    assert decoder.feed(b"[") == []
    assert decoder.feed(b"A") == ["w"]
    assert decoder.feed(b"\x1b[D") == ["a"]
    assert decoder.feed(b"\x1b[1;5A") == ["stop"]
    assert decoder.feed(b"x ") == ["stop", "stop"]
    assert decoder.feed(b"c") == ["quit"]


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class ScheduledReader:
    def __init__(self, clock, events):
        self.clock = clock
        self.events = list(events)

    def poll(self, timeout):
        end = self.clock.now + timeout
        if self.events and self.events[0][0] <= end + 1e-9:
            self.clock.now, result = self.events.pop(0)
            if isinstance(result, BaseException):
                raise result
            return result
        self.clock.now = end
        return None


@pytest.mark.parametrize("exit_event", [b"", b"c", KeyboardInterrupt(), OSError("disconnected")])
def test_loop_stops_on_exit_or_disconnect(config, exit_event):
    clock = Clock()
    reader = ScheduledReader(clock, [(0, b"w"), (0.2, exit_event)])
    driver = FakeMotorOutput()
    state = TeleopState(config, acceleration=1000)
    if isinstance(exit_event, BaseException):
        with pytest.raises(type(exit_event)):
            run_loop(driver, state, reader, clock=clock)
    else:
        run_loop(driver, state, reader, clock=clock)
    assert any(v != ZERO for v in driver.history)
    assert driver.history[-1] == ZERO


def test_loop_sends_zero_when_input_goes_stale(config):
    clock = Clock()
    reader = ScheduledReader(clock, [(0, b"w"), (0.7, b"c")])
    driver = FakeMotorOutput()
    reports = []
    run_loop(
        driver,
        TeleopState(config, acceleration=1000),
        reader,
        clock=clock,
        report=lambda values: reports.append((clock.now, values)),
    )
    assert any(t >= 0.3 and values == ZERO for t, values in reports)
    assert driver.history[-1] == ZERO


def test_terminal_mode_restored_after_exception():
    master, slave = os.openpty()
    before = termios.tcgetattr(slave)
    try:
        with pytest.raises(RuntimeError):
            with TerminalInput(slave) as reader:
                assert not termios.tcgetattr(slave)[3] & termios.ICANON
                os.write(master, b"w")
                assert reader.poll(0.5) == b"w"
                raise RuntimeError("test")
        assert termios.tcgetattr(slave) == before
    finally:
        os.close(master)
        os.close(slave)


def test_noninteractive_cli_rejects_before_hardware(monkeypatch):
    from capstone_base import keyboard_teleop

    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    monkeypatch.setattr(keyboard_teleop, "HiwonderController", lambda _: pytest.fail("Bus opened"))
    with pytest.raises(SystemExit) as error:
        main(["--backend", "hiwonder"])
    assert error.value.code == 2


def _read_available(process, timeout=0.05):
    ready, _, _ = select.select([process.stdout], [], [], timeout)
    return os.read(process.stdout.fileno(), 4096) if ready else b""


@pytest.mark.parametrize("termination", ["quit", "signal"])
def test_fake_cli_with_real_pty(termination):
    master, slave = os.openpty()
    before = termios.tcgetattr(slave)
    root = Path(__file__).resolve().parents[1]
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "from capstone_base.keyboard_teleop import main; raise SystemExit(main())",
        ],
        cwd=root,
        stdin=slave,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    output = b""
    try:
        deadline = time.monotonic() + 4
        while b"no key-release" not in output and time.monotonic() < deadline:
            output += _read_available(process)
        assert b"SIMULATION" in output
        for _ in range(7):
            os.write(master, b"w")
            time.sleep(0.08)
            output += _read_available(process, 0)
        deadline = time.monotonic() + 0.65
        while time.monotonic() < deadline:
            output += _read_available(process)
        assert b"(5, -5, -5, 5)" in output
        assert output.rfind(b"(0, 0, 0, 0)") > output.rfind(b"(5, -5, -5, 5)")
        if termination == "quit":
            os.write(master, b"c")
        else:
            process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=3) == (0 if termination == "quit" else 130)
        assert termios.tcgetattr(slave) == before
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        process.stdout.close()
        os.close(master)
        os.close(slave)
