#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib.util
import sys
import types
import unittest
from pathlib import Path


class Vector3(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist(object):
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class Message(object):
    def __init__(self, data=None):
        self.data = data


class RecordingPublisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def load_controller_module():
    rospy = types.ModuleType('rospy')
    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_msg.Twist = Twist
    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.Joy = Message
    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Bool = Message
    std_msgs_msg.Float32 = Message
    std_msgs_msg.String = Message

    sys.modules.update({
        'rospy': rospy,
        'geometry_msgs': geometry_msgs,
        'geometry_msgs.msg': geometry_msgs_msg,
        'sensor_msgs': sensor_msgs,
        'sensor_msgs.msg': sensor_msgs_msg,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msgs_msg,
    })

    source = Path(__file__).with_name('teleop_joy_ctrl_node.py')
    spec = importlib.util.spec_from_file_location('teleop_joy_ctrl_node_under_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = load_controller_module()


def make_controller(control_requested, was_active=False, timed_out=False):
    controller = MODULE.TeleopJoyCtrl.__new__(MODULE.TeleopJoyCtrl)
    controller.last_joy = object()
    controller.chassis_command_active = was_active
    controller.cmd_vel_pub = RecordingPublisher()
    controller.joy_timed_out = lambda: timed_out
    controller.chassis_control_requested = lambda _msg: control_requested
    command = Twist()
    command.linear.x = 0.4
    controller.make_chassis_twist = lambda _msg: command
    controller.maybe_log_chassis_success = lambda _twist: None
    controller.log_success = lambda _text: None
    return controller


class ChassisCmdVelOwnershipTest(unittest.TestCase):
    def test_idle_joystick_does_not_publish_zero_velocity(self):
        controller = make_controller(control_requested=False)

        controller.publish_loop(None)

        self.assertEqual([], controller.cmd_vel_pub.messages)
        self.assertFalse(controller.chassis_command_active)

    def test_shoulder_button_claims_control_and_publishes_velocity(self):
        controller = make_controller(control_requested=True)

        controller.publish_loop(None)

        self.assertEqual(1, len(controller.cmd_vel_pub.messages))
        self.assertEqual(0.4, controller.cmd_vel_pub.messages[0].linear.x)
        self.assertTrue(controller.chassis_command_active)

    def test_release_publishes_one_stop_then_becomes_silent(self):
        controller = make_controller(control_requested=False, was_active=True)

        controller.publish_loop(None)
        controller.publish_loop(None)

        self.assertEqual(1, len(controller.cmd_vel_pub.messages))
        self.assertEqual(0.0, controller.cmd_vel_pub.messages[0].linear.x)
        self.assertFalse(controller.chassis_command_active)

    def test_joystick_timeout_releases_active_control(self):
        controller = make_controller(
            control_requested=True,
            was_active=True,
            timed_out=True)

        controller.publish_loop(None)

        self.assertEqual(1, len(controller.cmd_vel_pub.messages))
        self.assertEqual(0.0, controller.cmd_vel_pub.messages[0].linear.x)
        self.assertFalse(controller.chassis_command_active)


if __name__ == '__main__':
    unittest.main()
