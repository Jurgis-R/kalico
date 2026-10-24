import math

import pytest

from klippy.extras.gcode_move import GCodeMove


class FakeGCodeCommand:
    def __init__(self, params):
        self.params = params

    def get_float(self, name, default=None):
        return self.params.get(name, default)

    def error(self, message):
        return ValueError(message)


def make_move():
    move = GCodeMove.__new__(GCodeMove)
    move.base_position = [0.0, 0.0, 0.0, 0.0]
    move.last_position = [110.0, 75.0, 3.0, 0.0]
    move.rotation_angle = 0.0
    move.rotation_center = None
    move.position_with_transform = lambda: [110.0, 75.0, 3.0, 0.0]
    move_calls = []
    move.move_with_transform = lambda pos, speed: move_calls.append(
        (list(pos), speed)
    )
    return move, move_calls


def test_g68_rotates_xy_about_requested_pivot_and_preserves_other_axes():
    move, _ = make_move()
    move.cmd_G68(FakeGCodeCommand({"X": 100.0, "Y": 75.0, "R": 90.0}))

    result = move._rotate_position([110.0, 75.0, 3.0, 4.0])
    assert result == pytest.approx([100.0, 85.0, 3.0, 4.0])
    assert move.last_position[:2] == pytest.approx([100.0, 65.0])


def test_g68_and_g69_keep_underlying_machine_position_stationary():
    move, _ = make_move()
    machine_position = [110.0, 75.0, 3.0, 0.0]
    move.position_with_transform = lambda: list(machine_position)

    move.cmd_G68(FakeGCodeCommand({"X": 100.0, "Y": 75.0, "R": 90.0}))
    assert move._rotate_position(move.last_position)[:2] == pytest.approx(
        machine_position[:2]
    )

    move.cmd_G69(FakeGCodeCommand({}))
    assert move.rotation_center is None
    assert move.rotation_angle == 0.0
    assert move.last_position[:2] == pytest.approx(machine_position[:2])


def test_rotation_is_inverse_consistent_and_does_not_mutate_input():
    move, _ = make_move()
    move.rotation_center = (100.0, 75.0)
    move.rotation_angle = math.radians(12.5)
    position = [124.0, 83.0, 7.0, 9.0, 11.0]

    rotated = move._rotate_position(position)
    recovered = move._rotate_position(rotated, inverse=True)

    assert position == [124.0, 83.0, 7.0, 9.0, 11.0]
    assert recovered == pytest.approx(position)


def test_move_applies_rotation_before_existing_transform():
    move, calls = make_move()
    move.rotation_center = (100.0, 75.0)
    move.rotation_angle = math.pi / 2.0
    position = [110.0, 75.0, 3.0, 4.0]

    move._move_with_rotation(position, 12.0)

    assert position == [110.0, 75.0, 3.0, 4.0]
    assert calls == [([100.0, 85.0, 3.0, 4.0], 12.0)]


@pytest.mark.parametrize(
    "params",
    [
        {"X": 100.0, "Y": 75.0},
        {"X": 100.0, "R": 2.0},
        {"Y": 75.0, "R": 2.0},
        {"X": float("nan"), "Y": 75.0, "R": 2.0},
    ],
)
def test_g68_rejects_missing_or_nonfinite_parameters(params):
    move, _ = make_move()
    with pytest.raises(ValueError):
        move.cmd_G68(FakeGCodeCommand(params))
    assert move.rotation_center is None
    assert move.rotation_angle == 0.0
