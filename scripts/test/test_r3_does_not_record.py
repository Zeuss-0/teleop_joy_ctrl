#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from teleop_joy_ctrl.intent_codec import (
    decode_events,
    encode_events,
    events_message,
    frame_has_discrete_content,
)
from teleop_joy_ctrl.joy_parser import EVENT_START, JoyParser
from teleop_joy_ctrl.teleop_executor import TeleopExecutor


BUTTONS = {
    "a": 0,
    "b": 1,
    "x": 2,
    "y": 3,
    "left_shoulder": 4,
    "right_shoulder": 5,
    "start": 7,
    "home": 8,
    "right_stick": 10,
}
AXES = {
    "left_x": 0,
    "left_y": 1,
    "left_trigger": 2,
    "dpad_x": 6,
    "dpad_y": 7,
}


class Message(object):
    def __init__(self, data=None):
        self.data = data


class ListLogger(object):
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warn(self, text):
        self.lines.append(("warn", text))

    def error(self, text):
        self.lines.append(("error", text))


def make_axes():
    return [0.0] * 8


def make_buttons(**pressed):
    buttons = [0] * 11
    for name, is_down in pressed.items():
        buttons[BUTTONS[name]] = 1 if is_down else 0
    return buttons


def make_parser():
    return JoyParser(
        axes=AXES,
        buttons=BUTTONS,
        axis_deadzone=0.08,
        adjust_repeat_interval=0.20,
        left_trigger_pressed_threshold=0.95,
    )


def make_executor():
    logger = ListLogger()
    executor = TeleopExecutor(
        chassis={"max_linear_speed": 0.4, "max_angular_speed": 0.8},
        camera={"exposure_default": 5000.0, "gain_default": 10.0},
        light={
            "enabled": True,
            "assume_initial_on": True,
            "brightness_default": 0.2,
            "home_restore_brightness": 0.2,
            "brightness_min": 0.0,
            "brightness_max": 1.0,
            "power_brightness_delay": 0.0,
            "power_publish_repeat": 1,
        },
        arm={"enabled": False},
        joy_timeout=0.5,
        debug_success_logs=True,
        publishers={
            "state": type("Pub", (), {"messages": [], "publish": lambda self, message: self.messages.append(message)})(),
            "debug": type("Pub", (), {"messages": [], "publish": lambda self, message: self.messages.append(message)})(),
            "light_power": type("Pub", (), {"messages": [], "publish": lambda self, message: self.messages.append(message)})(),
            "light_brightness": type("Pub", (), {"messages": [], "publish": lambda self, message: self.messages.append(message)})(),
        },
        sleeper=lambda _seconds: None,
        logger=logger,
        bool_cls=Message,
        float_cls=Message,
        string_cls=Message,
    )
    return executor, logger


class R3DoesNotRecordTest(unittest.TestCase):
    def test_press_hold_release_and_double_click_emit_nothing(self):
        sequences = (
            ((0.0, False), (0.10, True), (0.20, False)),
            ((0.0, False), (0.10, True), (0.18, False), (0.30, True), (0.38, False)),
            ((0.0, False), (0.10, True), (0.18, False), (0.70, False), (0.80, True), (0.90, False)),
            ((0.0, False), (1.00, True), (2.60, True), (2.70, False)),
            ((0.0, True), (0.20, False), (0.30, True), (1.90, True), (2.00, False)),
        )
        for samples in sequences:
            parser = make_parser()
            executor, logger = make_executor()
            for now, pressed in samples:
                result = parser.update(make_axes(), make_buttons(right_stick=pressed), now)
                self.assertEqual([], result.events)
                self.assertFalse(frame_has_discrete_content(result.events))
                self.assertIsNone(events_message(result.events, now, 1, session="intent"))
                executor.apply(result, now)
            self.assertEqual([], logger.lines)
            self.assertFalse(hasattr(executor, "video_command_queue"))
            self.assertFalse(hasattr(executor, "init_video_clients"))

    def test_r3_held_with_start_keeps_start_and_drops_stick(self):
        parser = make_parser()
        executor, logger = make_executor()
        parser.update(make_axes(), make_buttons(right_stick=True), 1.0)
        result = parser.update(
            make_axes(),
            make_buttons(right_stick=True, start=True),
            1.2,
        )
        self.assertEqual([EVENT_START], [event["type"] for event in result.events])
        raw = events_message(result.events, 1.2, 1, session="intent")
        self.assertIsNotNone(raw)
        self.assertNotIn("r3", raw)
        executor.apply_events(decode_events(raw).events)
        self.assertTrue(any("START按键已识别" in text for _level, text in logger.lines))
        self.assertFalse(any("录像" in text for _level, text in logger.lines))

    def test_codec_strips_forged_r3_and_executor_ignores_it(self):
        mixed = [
            {"type": "r3", "phase": "press", "pressed": True, "time": 1.0, "held_for": 0.0},
            {"type": "home"},
        ]
        raw = events_message(mixed, 1.0, 4, session="s")
        decoded = decode_events(raw)
        self.assertEqual([{"type": "home"}], decoded.events)
        self.assertFalse(frame_has_discrete_content([mixed[0]]))
        self.assertIsNone(events_message([mixed[0]], 1.0, 1))

        executor, logger = make_executor()
        stale = decode_events(encode_events(mixed, 1.0, seq=1, session="stale"))
        self.assertTrue(executor.apply_event_message(stale, 1.0))
        self.assertFalse(executor.light_on)
        self.assertAlmostEqual(0.0, executor.brightness)
        self.assertFalse(any("录像" in text or "R3" in text for _level, text in logger.lines))

    def test_config_and_sources_keep_video_nodes_off_the_gamepad(self):
        config = yaml.safe_load((ROOT / "config" / "default.yaml").read_text())
        self.assertNotIn("right_stick", config["buttons"])
        self.assertNotIn("double_click_interval", config["video"])
        self.assertNotIn("long_press_duration", config["video"])
        self.assertNotIn("manager_command_timeout", config["video"])
        self.assertEqual("/teleop_joy_ctrl/video/start", config["video"]["start_service"])
        self.assertEqual("/teleop_joy_ctrl/video/pause", config["video"]["pause_service"])
        self.assertEqual("/teleop_joy_ctrl/video/resume", config["video"]["resume_service"])
        self.assertEqual("/teleop_joy_ctrl/video/stop", config["video"]["stop_service"])

        launch = (ROOT / "launch" / "teleop_joy_ctrl.launch").read_text()
        self.assertIn('start_video_manager" default="true"', launch)
        self.assertIn('start_hybrid_video_manager" default="true"', launch)
        self.assertIn('start_legacy_video_manager" default="false"', launch)
        self.assertIn('legacy_single_node" default="true"', launch)
        self.assertIn("video_recording_manager_node.py", launch)

        parser_source = (ROOT / "src" / "teleop_joy_ctrl" / "joy_parser.py").read_text()
        executor_source = (ROOT / "src" / "teleop_joy_ctrl" / "teleop_executor.py").read_text()
        self.assertNotIn("right_stick", parser_source)
        self.assertNotIn("R3GestureInterpreter", executor_source)
        self.assertNotIn("video_command_queue", executor_source)
        self.assertNotIn("init_video_clients", executor_source)


if __name__ == "__main__":
    unittest.main()
