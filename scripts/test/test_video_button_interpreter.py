#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from teleop_joy_ctrl.teleop_executor import (
    R3GestureInterpreter,
    VIDEO_COMMAND_PAUSE,
    VIDEO_COMMAND_RESUME,
    VIDEO_COMMAND_START,
    VIDEO_COMMAND_STOP,
    VIDEO_ERROR,
    VIDEO_IDLE,
    VIDEO_PAUSED,
    VIDEO_RECORDING,
    VIDEO_STARTING,
    VIDEO_STOPPING,
)


class _Module(object):
    pass


MODULE = _Module()
MODULE.R3GestureInterpreter = R3GestureInterpreter
MODULE.VIDEO_COMMAND_PAUSE = VIDEO_COMMAND_PAUSE
MODULE.VIDEO_COMMAND_RESUME = VIDEO_COMMAND_RESUME
MODULE.VIDEO_COMMAND_START = VIDEO_COMMAND_START
MODULE.VIDEO_COMMAND_STOP = VIDEO_COMMAND_STOP
MODULE.VIDEO_ERROR = VIDEO_ERROR
MODULE.VIDEO_IDLE = VIDEO_IDLE
MODULE.VIDEO_PAUSED = VIDEO_PAUSED
MODULE.VIDEO_RECORDING = VIDEO_RECORDING
MODULE.VIDEO_STARTING = VIDEO_STARTING
MODULE.VIDEO_STOPPING = VIDEO_STOPPING


class FakeQueue(object):
    def __init__(self):
        self.items = []

    def put_nowait(self, item):
        self.items.append(item)


class SilentLogger(object):
    def info(self, _text):
        return None

    def warn(self, _text):
        return None

    def error(self, _text):
        return None


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
    def make_executor(self, state):
        from teleop_joy_ctrl.teleop_executor import TeleopExecutor

        executor = TeleopExecutor.__new__(TeleopExecutor)
        executor.video_enabled = True
        executor.video_state = state
        executor.video_input_state = state
        executor.video_command_queue = FakeQueue()
        executor.video_operation_lock = threading.Lock()
        executor.video_operation_pending = False
        executor.video_gesture = MODULE.R3GestureInterpreter(0.4, 1.5)
        executor.video_gesture.reset(False, 0.0)
        executor.debug_success_logs = False
        executor._logger = SilentLogger()
        return executor

    def feed(self, executor, pressed, when):
        executor._handle_r3({
            "phase": "hold" if pressed else "idle",
            "pressed": pressed,
            "time": when,
        })

    def test_controller_queues_start_after_idle_double_click(self):
        executor = self.make_executor(MODULE.VIDEO_IDLE)
        self.feed(executor, True, 0.10)
        self.feed(executor, False, 0.18)
        self.feed(executor, True, 0.30)
        self.feed(executor, False, 0.38)
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_START],
            executor.video_command_queue.items,
        )

    def test_pending_operation_ignores_followup_r3_command(self):
        executor = self.make_executor(MODULE.VIDEO_RECORDING)
        self.feed(executor, True, 1.00)
        self.feed(executor, False, 1.10)
        self.feed(executor, True, 1.20)
        self.feed(executor, False, 1.30)
        self.assertEqual(
            [MODULE.VIDEO_COMMAND_PAUSE],
            executor.video_command_queue.items,
        )
        self.assertEqual(MODULE.VIDEO_RECORDING, executor.video_state)
        self.assertEqual(MODULE.VIDEO_PAUSED, executor.video_input_state)
        self.assertTrue(executor.video_operation_pending)


if __name__ == "__main__":
    unittest.main()
