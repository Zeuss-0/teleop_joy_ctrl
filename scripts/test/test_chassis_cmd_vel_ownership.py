#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from teleop_joy_ctrl.joy_parser import ChassisIntent, SPEED_GEAR_HIGH, SPEED_GEAR_NONE
from teleop_joy_ctrl.teleop_executor import TeleopExecutor


class Vector3(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist(object):
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class RecordingPublisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_executor(was_active=False):
    executor = TeleopExecutor(
        chassis={
            "holonomic": False,
            "high_speed_scale": 1.0,
            "low_speed_scale": 0.35,
            "max_linear_speed": 0.4,
            "max_angular_speed": 0.8,
        },
        camera={},
        light={"enabled": False},
        arm={"enabled": False},
        video={"enabled": False},
        joy_timeout=0.5,
        debug_success_logs=False,
        publishers={"cmd_vel": RecordingPublisher()},
        twist_cls=Twist,
        logger=type("Silent", (), {
            "info": staticmethod(lambda _text: None),
            "warn": staticmethod(lambda _text: None),
            "error": staticmethod(lambda _text: None),
        })(),
    )
    executor.chassis_command_active = was_active
    executor._last_joy_time = 10.0
    return executor


class ChassisCmdVelOwnershipTest(unittest.TestCase):
    def test_idle_joystick_does_not_publish_zero_velocity(self):
        executor = make_executor()
        executor._chassis_intent = ChassisIntent(False, SPEED_GEAR_NONE, 0.0, 1.0)

        executor.publish_chassis(10.0)

        self.assertEqual([], executor.cmd_vel_pub.messages)
        self.assertFalse(executor.chassis_command_active)

    def test_shoulder_button_claims_control_and_publishes_velocity(self):
        executor = make_executor()
        executor._chassis_intent = ChassisIntent(True, SPEED_GEAR_HIGH, 0.0, 1.0)

        executor.publish_chassis(10.0)

        self.assertEqual(1, len(executor.cmd_vel_pub.messages))
        self.assertEqual(0.4, executor.cmd_vel_pub.messages[0].linear.x)
        self.assertTrue(executor.chassis_command_active)

    def test_release_publishes_one_stop_then_becomes_silent(self):
        executor = make_executor(was_active=True)
        executor._chassis_intent = ChassisIntent(False, SPEED_GEAR_NONE, 0.0, 1.0)

        executor.publish_chassis(10.0)
        executor.publish_chassis(10.0)

        self.assertEqual(1, len(executor.cmd_vel_pub.messages))
        self.assertEqual(0.0, executor.cmd_vel_pub.messages[0].linear.x)
        self.assertFalse(executor.chassis_command_active)

    def test_joystick_timeout_releases_active_control(self):
        executor = make_executor(was_active=True)
        executor._chassis_intent = ChassisIntent(True, SPEED_GEAR_HIGH, 0.0, 1.0)

        executor.publish_chassis(10.6)

        self.assertEqual(1, len(executor.cmd_vel_pub.messages))
        self.assertEqual(0.0, executor.cmd_vel_pub.messages[0].linear.x)
        self.assertFalse(executor.chassis_command_active)

    def test_centered_stick_while_active_keeps_publishing_zeros(self):
        executor = make_executor()
        executor._chassis_intent = ChassisIntent(True, SPEED_GEAR_HIGH, 0.0, 0.0)

        executor.publish_chassis(10.0)
        executor.publish_chassis(10.1)

        self.assertEqual(2, len(executor.cmd_vel_pub.messages))
        self.assertEqual(0.0, executor.cmd_vel_pub.messages[0].linear.x)
        self.assertEqual(0.0, executor.cmd_vel_pub.messages[1].linear.x)
        self.assertTrue(executor.chassis_command_active)


if __name__ == "__main__":
    unittest.main()
