#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib.util
import sys
import threading
import types
import unittest
from pathlib import Path


class Message(object):
    def __init__(self, data=None):
        self.data = data


def load_controller_module():
    rospy = types.ModuleType("rospy")
    rospy.logwarn = lambda *_args, **_kwargs: None
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    geometry_msgs_msg.Twist = object
    sensor_msgs_msg.Joy = Message
    std_msgs_msg.Bool = Message
    std_msgs_msg.Float32 = Message
    std_msgs_msg.String = Message

    sys.modules.update({
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    })

    source = Path(__file__).with_name("teleop_joy_ctrl_node.py")
    spec = importlib.util.spec_from_file_location(
        "teleop_video_button_under_test", source
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = load_controller_module()


class JoyMessage(object):
    def __init__(self, pressed=False):
        self.buttons = [0] * 11
        self.buttons[10] = 1 if pressed else 0


class FakeQueue(object):
    def __init__(self):
        self.items = []

    def put_nowait(self, item):
        self.items.append(item)


class R3GestureInterpreterTest(unittest.TestCase):
    def setUp(self):
        self.subject = MODULE.R3GestureInterpreter(0.4, 1.5)
        self.subject.reset(False, 0.0)

    def click(self, pressed_at, released_at, state):
        self.subject.update(True, pressed_at, state)
        return self.subject.update(False, released_at, state)

    def test_idle_requires_two_clicks_inside_window(self):
        self.assertEqual([], self.click(0.10, 0.18, MODULE.VIDEO_IDLE))
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_START],
            self.subject.update(True, 0.30, MODULE.VIDEO_IDLE),
        )
        self.assertEqual(
            [], self.subject.update(False, 0.38, MODULE.VIDEO_STARTING)
        )

    def test_idle_clicks_outside_window_do_not_start(self):
        self.assertEqual([], self.click(0.10, 0.18, MODULE.VIDEO_IDLE))
        self.subject.update(False, 0.70, MODULE.VIDEO_IDLE)
        self.assertEqual([], self.click(0.80, 0.90, MODULE.VIDEO_IDLE))

    def test_recording_short_click_pauses(self):
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_PAUSE],
            self.click(1.00, 1.10, MODULE.VIDEO_RECORDING),
        )

    def test_paused_short_click_resumes(self):
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_RESUME],
            self.click(1.00, 1.10, MODULE.VIDEO_PAUSED),
        )

    def test_long_press_stops_once_without_release_click(self):
        self.subject.update(True, 1.00, MODULE.VIDEO_RECORDING)
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_STOP],
            self.subject.update(True, 2.50, MODULE.VIDEO_RECORDING),
        )
        self.assertEqual(
            [], self.subject.update(True, 2.80, MODULE.VIDEO_RECORDING)
        )
        self.assertEqual(
            [], self.subject.update(False, 2.90, MODULE.VIDEO_RECORDING)
        )

    def test_starting_and_stopping_ignore_clicks(self):
        for state in (MODULE.VIDEO_STARTING, MODULE.VIDEO_STOPPING):
            self.subject.reset(False, 0.0)
            self.assertEqual([], self.click(0.10, 0.20, state))

    def test_error_state_allows_double_click_retry(self):
        self.assertEqual([], self.click(0.10, 0.18, MODULE.VIDEO_ERROR))
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_START],
            self.subject.update(True, 0.30, MODULE.VIDEO_ERROR),
        )

    def test_second_press_inside_window_starts_even_if_release_is_late(self):
        self.assertEqual([], self.click(0.10, 0.18, MODULE.VIDEO_IDLE))
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_START],
            self.subject.update(True, 0.50, MODULE.VIDEO_IDLE),
        )
        self.assertEqual(
            [], self.subject.update(False, 0.65, MODULE.VIDEO_STARTING)
        )

    def test_idle_long_press_does_not_arm_a_click(self):
        self.subject.update(True, 0.10, MODULE.VIDEO_IDLE)
        self.assertEqual(
            [], self.subject.update(False, 1.70, MODULE.VIDEO_IDLE)
        )
        self.assertEqual([], self.click(1.80, 1.90, MODULE.VIDEO_IDLE))


class VideoControllerIntegrationTest(unittest.TestCase):
    def make_controller(self, state):
        controller = MODULE.TeleopJoyCtrl.__new__(MODULE.TeleopJoyCtrl)
        controller.buttons = {"right_stick": 10}
        controller.video_enabled = True
        controller.video_state = state
        controller.video_input_state = state
        controller.video_command_queue = FakeQueue()
        controller.video_operation_lock = threading.Lock()
        controller.video_operation_pending = False
        controller.video_gesture = MODULE.R3GestureInterpreter(0.4, 1.5)
        controller.video_gesture.reset(False, 0.0)
        controller.log_success = lambda _text: None
        return controller

    def test_controller_queues_start_after_idle_double_click(self):
        controller = self.make_controller(MODULE.VIDEO_IDLE)
        controller.locked = True
        controller.handle_video_button(JoyMessage(True), 0.10)
        controller.handle_video_button(JoyMessage(False), 0.18)
        controller.handle_video_button(JoyMessage(True), 0.30)
        controller.handle_video_button(JoyMessage(False), 0.38)
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_START],
            controller.video_command_queue.items,
        )

    def test_pending_operation_ignores_followup_r3_command(self):
        controller = self.make_controller(MODULE.VIDEO_RECORDING)
        controller.handle_video_button(JoyMessage(True), 1.00)
        controller.handle_video_button(JoyMessage(False), 1.10)
        controller.handle_video_button(JoyMessage(True), 1.20)
        controller.handle_video_button(JoyMessage(False), 1.30)
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_PAUSE],
            controller.video_command_queue.items,
        )
        self.assertEqual(MODULE.VIDEO_RECORDING, controller.video_state)
        self.assertEqual(
            MODULE.VIDEO_PAUSED, controller.video_input_state
        )
        self.assertTrue(controller.video_operation_pending)


if __name__ == "__main__":
    unittest.main()
