import json
import signal

import pytest
from test_hiwonder_serial import LX224Bus, LX224Controller, RecordingChainSerial, RecordingSerial

from capstone_base import servo_test


def hardware(monkeypatch, tmp_path, *, calibrated=True):
    port = RecordingSerial()
    settings = {"port": "/dev/test-servo"}
    if calibrated:
        settings.update(min_angle_deg=115, max_angle_deg=125)
    path = tmp_path / "servo.json"
    path.write_text(json.dumps(settings))
    monkeypatch.setattr(
        servo_test,
        "LX224Controller",
        lambda config: LX224Controller(config, port=port, clock=port.clock),
    )
    return port, ["--backend", "buslinker", "--config", str(path)]


def test_default_cli_is_fake_status(monkeypatch, capsys):
    monkeypatch.setattr(
        servo_test, "LX224Controller", lambda _: pytest.fail("Opened real hardware")
    )
    assert servo_test.main([]) == 0
    assert "SIMULATION (no serial connection)" in capsys.readouterr().out


def test_hardware_status_requires_no_calibration_and_never_writes_motion(monkeypatch, tmp_path):
    port, args = hardware(monkeypatch, tmp_path, calibrated=False)
    assert servo_test.main([*args, "status"]) == 0
    assert [p[4] for p in port.packets] == [30, 32, 27, 26, 28]
    with pytest.raises(SystemExit):
        servo_test.main([*args, "move", "--delta", "2"])
    assert len(port.packets) == 5  # Rejected before opening or reading hardware.


def test_relative_move_waits_verifies_position_and_stops(monkeypatch, tmp_path, capsys):
    port, args = hardware(monkeypatch, tmp_path)
    sleeps = []
    monkeypatch.setattr(servo_test.time, "sleep", sleeps.append)
    assert servo_test.main([*args, "move", "--delta", "2", "--duration", "1"]) == 0
    assert sleeps == [1.1]
    assert port.position == 508
    assert port.packets[-1][4] == 12
    assert port.torque == 1
    assert "within tolerance" in capsys.readouterr().out


def test_failed_motion_readback_returns_failure_and_stops(monkeypatch, tmp_path, capsys):
    port, args = hardware(monkeypatch, tmp_path)
    port.follow_move = False
    monkeypatch.setattr(servo_test.time, "sleep", lambda _: None)
    assert servo_test.main([*args, "move", "--delta", "4"]) == 1
    assert "Target not reached" in capsys.readouterr().err
    assert port.packets[-1][4] == 12


def test_interrupt_stops_and_restores_signal_handler(monkeypatch, tmp_path):
    port, args = hardware(monkeypatch, tmp_path)
    previous = signal.getsignal(signal.SIGTERM)

    def interrupt(_):
        servo_test._interrupt(signal.SIGTERM, None)

    monkeypatch.setattr(servo_test.time, "sleep", interrupt)
    assert servo_test.main([*args, "move", "--delta", "2"]) == 130
    assert port.packets[-1][4] == 12
    assert signal.getsignal(signal.SIGTERM) == previous


def test_sweep_repeats_fixed_endpoints_and_stops(monkeypatch, tmp_path):
    port, args = hardware(monkeypatch, tmp_path)
    sleeps = []
    monkeypatch.setattr(servo_test.time, "sleep", sleeps.append)
    assert servo_test.main([*args, "sweep", "--amplitude", "2", "--cycles", "2"]) == 0
    moves = [int.from_bytes(p[5:7], "little") for p in port.packets if p[4] == 1]
    assert moves == [508, 492, 508, 492]
    assert sleeps == [1.1] * 4
    assert port.packets[-1][4] == 12
    assert port.torque == 1


@pytest.mark.parametrize(
    "extra,amplitude",
    [
        ([], "4"),  # Both endpoints fit, but the full span exceeds max_step_deg.
        (["--max-step", "30"], "6"),  # Mechanical limits still apply with an override.
    ],
)
def test_sweep_prevalidates_both_endpoints_and_span(monkeypatch, tmp_path, extra, amplitude):
    port, args = hardware(monkeypatch, tmp_path)
    assert servo_test.main([*args, *extra, "sweep", "--amplitude", amplitude, "--cycles", "1"]) == 1
    assert not any(p[4] == 1 for p in port.packets)


def test_larger_sweep_with_session_override(monkeypatch, tmp_path):
    port, args = hardware(monkeypatch, tmp_path)
    monkeypatch.setattr(servo_test.time, "sleep", lambda _: None)
    assert (
        servo_test.main([*args, "--max-step", "12", "sweep", "--amplitude", "4", "--cycles", "1"])
        == 0
    )
    moves = [int.from_bytes(p[5:7], "little") for p in port.packets if p[4] == 1]
    assert moves == [517, 483]


def test_continuous_sweep_interrupts_and_stops(monkeypatch, tmp_path):
    port, args = hardware(monkeypatch, tmp_path)
    calls = 0

    def interrupt_on_third_leg(_):
        nonlocal calls
        calls += 1
        if calls == 3:
            raise KeyboardInterrupt

    monkeypatch.setattr(servo_test.time, "sleep", interrupt_on_third_leg)
    assert servo_test.main([*args, "sweep"]) == 130
    assert calls == 3
    assert [p[4] for p in port.packets].count(1) == 3
    assert port.packets[-1][4] == 12


def test_sweep_failure_does_not_issue_next_move(monkeypatch, tmp_path, capsys):
    port, args = hardware(monkeypatch, tmp_path)
    port.follow_move = False
    monkeypatch.setattr(servo_test.time, "sleep", lambda _: None)
    assert servo_test.main([*args, "--max-step", "12", "sweep", "--amplitude", "4"]) == 1
    assert "Target not reached" in capsys.readouterr().err
    assert [p[4] for p in port.packets].count(1) == 1
    assert port.packets[-1][4] == 12


def chain_hardware(monkeypatch, tmp_path, *, ids=(1, 2), calibrated=True):
    port = RecordingChainSerial(ids)
    settings = {"port": "/dev/test-servo"}
    if calibrated:
        settings.update(min_angle_deg=115, max_angle_deg=125)
    path = tmp_path / "servo.json"
    path.write_text(json.dumps(settings))
    monkeypatch.setattr(
        servo_test,
        "LX224Bus",
        lambda config: LX224Bus(config, port=port, clock=port.clock),
    )
    monkeypatch.setattr(
        servo_test,
        "LX224Controller",
        lambda config: LX224Controller(config, port=port, clock=port.clock),
    )
    return port, ["--backend", "buslinker", "--config", str(path)]


@pytest.mark.parametrize("servo_id", (1, 2))
@pytest.mark.parametrize("subcommand", ("status", "move", "sweep"))
def test_cli_operates_one_connected_servo_without_its_neighbor(
    monkeypatch, tmp_path, servo_id, subcommand
):
    port, args = chain_hardware(monkeypatch, tmp_path, ids=(servo_id,))
    monkeypatch.setattr(servo_test.time, "sleep", lambda _: None)
    options = {"status": [], "move": ["--delta", "2"], "sweep": ["--cycles", "1"]}
    assert servo_test.main([*args, "--id", str(servo_id), subcommand, *options[subcommand]]) == 0
    assert port.packets
    assert all(packet[2] == servo_id for packet in port.packets)
    if subcommand == "status":
        assert all(packet[4] in (26, 27, 28, 30, 32) for packet in port.packets)
    else:
        assert any(packet[4] == 1 for packet in port.packets)


def test_startup_id_assignment_then_status_needs_no_limits(monkeypatch, tmp_path, capsys):
    port, args = chain_hardware(monkeypatch, tmp_path, ids=(1,), calibrated=False)
    assert servo_test.main([*args, "--id", "1", "--new-id", "3", "status"]) == 0
    output = capsys.readouterr().out
    assert "ID assignment verified: 1 -> 3" in output
    assert "ID 3 | Position" in output
    assert list(port.nodes) == [3]
    assert all(p[2] == 3 for p in port.packets[3:])
    assert all(p[4] in (13, 14, 26, 27, 28, 30, 32) for p in port.packets)


def test_startup_id_assignment_then_motion_uses_new_id(monkeypatch, tmp_path):
    port, args = chain_hardware(monkeypatch, tmp_path, ids=(1,))
    monkeypatch.setattr(servo_test.time, "sleep", lambda _: None)
    assert servo_test.main([*args, "--new-id", "3", "move", "--delta", "2"]) == 0
    assert [(p[2], p[4]) for p in port.packets if p[4] in (1, 12)] == [(3, 1), (3, 12)]


@pytest.mark.parametrize("failure", ["conflict", "ignored_write"])
def test_startup_id_failure_prevents_command(monkeypatch, tmp_path, capsys, failure):
    ids = (1, 2) if failure == "conflict" else (1,)
    port, args = chain_hardware(monkeypatch, tmp_path, ids=ids)
    port.ignore_id_write = failure == "ignored_write"
    assert servo_test.main([*args, "--new-id", "2", "move", "--delta", "2"]) == 1
    assert "Servo test failed" in capsys.readouterr().err
    assert all(p[4] in (13, 14) for p in port.packets)


@pytest.mark.parametrize(
    "options",
    [
        ["--new-id", "254"],
        ["--new-id", "-1"],
        ["--ids", "1,1"],
        ["--ids", "1,254"],
        ["--ids", "1,wrong"],
        ["--ids", "1,"],
        ["--ids", ""],
        ["--ids", "1,2", "--new-id", "3"],
        ["--id", "1", "--ids", "1,2"],
    ],
)
def test_invalid_chain_options_rejected_before_hardware(monkeypatch, tmp_path, options):
    port, args = chain_hardware(monkeypatch, tmp_path)
    with pytest.raises(SystemExit) as error:
        servo_test.main([*args, *options, "status"])
    assert error.value.code == 2
    assert port.events == []


def test_chain_status_labels_every_id_and_sends_no_writes(monkeypatch, tmp_path, capsys):
    port, args = chain_hardware(monkeypatch, tmp_path, calibrated=False)
    port.nodes[2]["position"] = 400
    assert servo_test.main([*args, "--ids", "1,2", "status"]) == 0
    output = capsys.readouterr().out
    assert "ID 1 | Position: 120.00" in output
    assert "ID 2 | Position: 96.00" in output
    assert [(p[2], p[4]) for p in port.packets] == [
        (servo_id, command) for servo_id in (1, 2) for command in (30, 32, 27, 26, 28)
    ]


def test_chain_relative_move_uses_each_position_and_waits_once(monkeypatch, tmp_path):
    port, args = chain_hardware(monkeypatch, tmp_path)
    port.nodes[2]["position"] = 495
    sleeps = []
    monkeypatch.setattr(servo_test.time, "sleep", sleeps.append)
    assert servo_test.main([*args, "--ids", "1,2", "move", "--delta", "2"]) == 0
    assert port.nodes[1]["position"] == 508
    assert port.nodes[2]["position"] == 503
    assert sleeps == [1.1]
    assert {(p[2], p[4]) for p in port.packets if p[4] == 12} == {(1, 12), (2, 12)}


@pytest.mark.parametrize("command", ["move", "sweep"])
def test_chain_prevalidates_every_target_before_first_move(monkeypatch, tmp_path, command):
    port, args = chain_hardware(monkeypatch, tmp_path)
    port.nodes[2]["position"] = 520  # 124.8: status valid, positive target out of bounds.
    target = ["--delta", "2"] if command == "move" else ["--amplitude", "2", "--cycles", "1"]
    assert servo_test.main([*args, "--ids", "1,2", command, *target]) == 1
    assert not any(p[4] in (1, 12, 31) for p in port.packets)


def test_chain_sweep_uses_independent_fixed_centers(monkeypatch, tmp_path):
    port, args = chain_hardware(monkeypatch, tmp_path)
    port.nodes[2]["position"] = 495
    monkeypatch.setattr(servo_test.time, "sleep", lambda _: None)
    assert servo_test.main([*args, "--ids", "1,2", "sweep", "--cycles", "2"]) == 0
    moves = [(p[2], int.from_bytes(p[5:7], "little")) for p in port.packets if p[4] == 1]
    assert moves == [(1, 508), (2, 503), (1, 492), (2, 487)] * 2


def test_chain_interrupt_stops_all_active_servos(monkeypatch, tmp_path):
    port, args = chain_hardware(monkeypatch, tmp_path)

    def interrupt(_):
        raise KeyboardInterrupt

    monkeypatch.setattr(servo_test.time, "sleep", interrupt)
    assert servo_test.main([*args, "--ids", "1,2", "move", "--delta", "2"]) == 130
    assert {(p[2], p[4]) for p in port.packets if p[4] == 12} == {(1, 12), (2, 12)}


@pytest.mark.parametrize("command", ["stop", "torque"])
def test_chain_explicit_stop_or_release_without_limits(monkeypatch, tmp_path, command):
    port, args = chain_hardware(monkeypatch, tmp_path, calibrated=False)
    tail = ["off"] if command == "torque" else []
    assert servo_test.main([*args, "--ids", "1,2", command, *tail]) == 0
    assert {(p[2], p[4]) for p in port.packets if p[4] == 12} == {(1, 12), (2, 12)}
    assert all(
        state["torque"] == (0 if command == "torque" else 1) for state in port.nodes.values()
    )


def test_chain_stop_failure_still_attempts_other_servo(monkeypatch, tmp_path):
    port, args = chain_hardware(monkeypatch, tmp_path)
    port.fail_command = 12
    assert servo_test.main([*args, "--ids", "1,2", "stop"]) == 1
    assert [(p[2], p[4]) for p in port.packets] == [(1, 12), (2, 12)]


def test_fake_chain_and_startup_id_stay_offline(monkeypatch, capsys):
    monkeypatch.setattr(servo_test, "LX224Controller", lambda _: pytest.fail("Opened hardware"))
    monkeypatch.setattr(servo_test, "LX224Bus", lambda _: pytest.fail("Opened hardware"))
    assert servo_test.main(["--new-id", "3"]) == 0
    assert "ID assignment verified: 1 -> 3" in capsys.readouterr().out
    assert servo_test.main(["--ids", "1,2,3"]) == 0
    output = capsys.readouterr().out
    assert all(f"ID {servo_id} | Position" in output for servo_id in (1, 2, 3))


def test_chain_failed_second_motion_write_stops_both_without_retry(monkeypatch, tmp_path):
    port, args = chain_hardware(monkeypatch, tmp_path)
    write = port.write

    def fail_second_move(data):
        if (data[2], data[4]) == (2, 1):
            port.events.append(data)
            raise OSError("Second servo write failed")
        return write(data)

    monkeypatch.setattr(port, "write", fail_second_move)
    assert servo_test.main([*args, "--ids", "1,2", "move", "--delta", "2"]) == 1
    assert [(p[2], p[4]) for p in port.packets if p[4] in (1, 12)] == [
        (1, 1),
        (2, 1),
        (1, 12),
        (2, 12),
    ]


def test_missing_chain_member_in_status_never_sends_actuator_writes(monkeypatch, tmp_path):
    port, args = chain_hardware(monkeypatch, tmp_path, ids=(1,), calibrated=False)
    assert servo_test.main([*args, "--ids", "1,2", "status"]) == 1
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets)
