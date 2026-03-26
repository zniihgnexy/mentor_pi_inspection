#!/usr/bin/env python3
"""Tests for keyboard teleop key bindings."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mentor_pi_inspection.control_layer.keyboard_bindings import command_for_key


def test_forward_binding():
    command = command_for_key('w', 0.3, 0.2, 0.7)
    assert command is not None
    assert command.linear_x == 0.3
    assert command.linear_y == 0.0
    assert command.angular_z == 0.0
    assert not command.emergency_stop


def test_mecanum_strafe_binding():
    command = command_for_key('a', 0.3, 0.2, 0.7)
    assert command is not None
    assert command.linear_x == 0.0
    assert command.linear_y == 0.2
    assert command.angular_z == 0.0


def test_rotation_binding_is_case_insensitive():
    command = command_for_key('Q', 0.3, 0.2, 0.7)
    assert command is not None
    assert command.angular_z == 0.7


def test_stop_binding_latches_estop():
    command = command_for_key(' ', 0.3, 0.2, 0.7)
    assert command is not None
    assert not command.is_motion
    assert command.emergency_stop


def test_unknown_key_is_ignored():
    assert command_for_key('z', 0.3, 0.2, 0.7) is None
