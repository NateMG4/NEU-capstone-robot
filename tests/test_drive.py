import math

import pytest

from capstone_base.cli import main
from capstone_base.controller import FakeController
from capstone_base.kinematics import Geometry, wheel_speeds

GEOMETRY = Geometry(0.05, 0.30, 0.30)


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ((0, 0, 0), (0, 0, 0, 0)),
        ((0.1, 0, 0), (2, 2, 2, 2)),
        ((-0.1, 0, 0), (-2, -2, -2, -2)),
        ((0, 0.1, 0), (-2, 2, 2, -2)),
        ((0, 0, 0.5), (-3, 3, -3, 3)),
        ((0.1, 0.1, 0), (0, 4, 4, 0)),
    ],
)
def test_motion_patterns(command, expected):
    assert wheel_speeds(*command, GEOMETRY, 10) == pytest.approx(expected)


def test_saturation_preserves_wheel_ratios():
    raw = wheel_speeds(0.8, 0.3, 0.2, GEOMETRY, 100)
    limited = wheel_speeds(0.8, 0.3, 0.2, GEOMETRY, 4)
    scale = 4 / max(abs(v) for v in raw)
    assert max(abs(v) for v in limited) == pytest.approx(4)
    assert limited == pytest.approx(tuple(v * scale for v in raw))


@pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize("axis", [0, 1, 2])
def test_rejects_nonfinite_commands(invalid, axis):
    command = [0.0, 0.0, 0.0]
    command[axis] = invalid
    with pytest.raises(ValueError):
        wheel_speeds(*command, GEOMETRY, 10)


@pytest.mark.parametrize("invalid", [0, -1, math.nan, math.inf])
def test_rejects_invalid_geometry_and_limit(invalid):
    with pytest.raises(ValueError):
        Geometry(invalid, 0.3, 0.3)
    with pytest.raises(ValueError):
        wheel_speeds(0, 0, 0, GEOMETRY, invalid)


def test_stop_clears_all_targets():
    controller = FakeController()
    controller.set_wheel_speeds((1, -2, 3, -4))
    controller.stop()
    assert controller.targets == (0, 0, 0, 0)
    assert len(controller.history) == 2


def test_invalid_controller_command_does_not_replace_target():
    controller = FakeController()
    with pytest.raises(ValueError):
        controller.set_wheel_speeds((1, 2, math.nan, 4))
    assert controller.targets == (0, 0, 0, 0)


def test_cli_demo(capsys):
    assert main(["--vx", "0.1"]) == 0
    output = capsys.readouterr().out
    assert output.count("+2.000 rad/s") == 4
    assert "SIMULATION ONLY" in output
    assert "returned to zero" in output


def test_cli_rejects_nan():
    with pytest.raises(SystemExit) as error:
        main(["--vx", "nan"])
    assert error.value.code == 2
