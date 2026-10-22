"""Unit tests for the trigger -> gripper joint mapping (needs rclpy)."""

import math

import pytest

pytest.importorskip("rclpy")

from yubi_bringup.trigger_gripper_node import trigger_to_position  # noqa: E402


def test_released_trigger_is_open():
    assert trigger_to_position(0.0, 0.94, 0.0) == pytest.approx(0.94)


def test_pulled_trigger_is_closed():
    assert trigger_to_position(1.0, 0.94, 0.0) == pytest.approx(0.0)


def test_linear_in_between():
    assert trigger_to_position(0.25, 0.94, 0.0) == pytest.approx(0.705)


def test_out_of_range_is_clamped():
    assert trigger_to_position(-0.2, 0.94, 0.0) == pytest.approx(0.94)
    assert trigger_to_position(1.3, 0.94, 0.0) == pytest.approx(0.0)


def test_nan_means_no_value():
    assert trigger_to_position(math.nan, 0.94, 0.0) is None
