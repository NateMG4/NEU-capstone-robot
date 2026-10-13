"""Shared controller contract and offline implementation."""

import math
from typing import Protocol

WheelTargets = tuple[float, float, float, float]


class Controller(Protocol):
    def set_wheel_speeds(self, targets: WheelTargets) -> None: ...
    def stop(self) -> None: ...


class FakeController:
    """Record desired rad/s; does not simulate traction or encoder feedback."""

    def __init__(self):
        self.targets: WheelTargets = (0.0, 0.0, 0.0, 0.0)
        self.history: list[WheelTargets] = []

    def set_wheel_speeds(self, targets: WheelTargets) -> None:
        if len(targets) != 4 or not all(math.isfinite(v) for v in targets):
            raise ValueError("Expected four finite wheel targets")
        self.targets = tuple(targets)
        self.history.append(self.targets)

    def stop(self) -> None:
        self.set_wheel_speeds((0.0, 0.0, 0.0, 0.0))
