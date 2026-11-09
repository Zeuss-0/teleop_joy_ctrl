#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from teleop_joy_ctrl.joy_parser import (
    EVENT_ADJUST,
    EVENT_HOME,
    EVENT_LOCK,
    EVENT_MODE,
    EVENT_START,
    JoyParser,
    SPEED_GEAR_HIGH,
    SPEED_GEAR_LOW,
    SPEED_GEAR_NONE,
    TARGET_EXPOSURE,
    TARGET_GAIN,
    TARGET_LINEAR_SPEED,
)
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
    def __init__(self, timeline=None, name=None):
        self.messages = []
        self.timeline = timeline
        self.name = name

    def publish(self, message):
        self.messages.append(message)
        if self.timeline is not None:
            self.timeline.append((self.name, getattr(message, "data", message)))


class ListLogger(object):
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warn(self, text):
        self.lines.append(("warn", text))

    def error(self, text):
        self.lines.append(("error", text))


def make_axes(left_x=0.0, left_y=0.0, left_trigger=1.0, dpad_x=0.0, dpad_y=0.0):
    axes = [0.0] * 8
    axes[AXES["left_x"]] = left_x
    axes[AXES["left_y"]] = left_y
    axes[AXES["left_trigger"]] = left_trigger
    axes[AXES["dpad_x"]] = dpad_x
    axes[AXES["dpad_y"]] = dpad_y
    return axes


def make_buttons(**pressed):
    buttons = [0] * 11
    for name, is_down in pressed.items():
        buttons[BUTTONS[name]] = 1 if is_down else 0
    return buttons


def event_types(result):
    return [event["type"] for event in result.events]


def event_list(result):
    return list(result.events)


class StepASplitTest(unittest.TestCase):
    def setUp(self):
        self.timeline = []
        self.logger = ListLogger()
        self.sleeps = []
        self.pubs = {
            "cmd_vel": RecordingPublisher(),
            "state": RecordingPublisher(),
            "debug": RecordingPublisher(),
            "exposure": RecordingPublisher(self.timeline, "exposure"),
            "gain": RecordingPublisher(self.timeline, "gain"),
            "light_power": RecordingPublisher(self.timeline, "power"),
            "light_brightness": RecordingPublisher(self.timeline, "brightness"),
        }
        self.parser = JoyParser(
            axes=AXES,
            buttons=BUTTONS,
            axis_deadzone=0.08,
            adjust_repeat_interval=0.20,
            left_trigger_pressed_threshold=0.95,
        )
        self.executor = TeleopExecutor(
            chassis={
                "holonomic": False,
                "low_speed_scale": 0.60,
                "high_speed_scale": 1.0,
                "max_linear_speed": 0.4,
                "min_linear_speed": 0.05,
                "max_linear_speed_limit": 3.0,
                "linear_speed_step_min": 0.02,
                "linear_speed_step_ratio": 0.10,
                "max_angular_speed": 0.8,
                "min_angular_speed": 0.05,
                "max_angular_speed_limit": 4.0,
                "angular_speed_step_min": 0.05,
                "angular_speed_step_ratio": 0.10,
            },
            camera={
                "exposure_default": 5000.0,
                "exposure_min": 20.0,
                "exposure_max": 50000.0,
                "exposure_step_min": 100.0,
                "exposure_step_ratio": 0.12,
                "gain_default": 10.0,
                "gain_min": 0.0,
                "gain_max": 24.0,
                "gain_step_min": 0.5,
                "gain_step_ratio": 0.10,
                "exposure_topic": "/camera/exposure_cmd",
                "gain_topic": "/camera/gain_cmd",
            },
            light={
                "enabled": True,
                "assume_initial_on": True,
                "brightness_default": 0.2,
                "home_restore_brightness": 0.2,
                "off_brightness_epsilon": 0.001,
                "brightness_min": 0.0,
                "brightness_max": 1.0,
                "brightness_step": 0.02,
                "power_brightness_delay": 0.05,
                "power_publish_repeat": 2,
                "power_publish_repeat_interval": 0.05,
                "power_topic": "/light/power_cmd",
                "brightness_topic": "/light/brightness_cmd",
            },
            arm={"enabled": False},
            joy_timeout=0.5,
            debug_success_logs=True,
            chassis_debug_interval=1.0,
            cmd_vel_topic="/teleop_joy_ctrl/cmd_vel",
            publishers=self.pubs,
            sleeper=self._sleep,
            logger=self.logger,
            twist_cls=Twist,
            bool_cls=Message,
            float_cls=Message,
            string_cls=Message,
        )

    def _sleep(self, seconds):
        self.sleeps.append(seconds)
        self.timeline.append(("sleep", seconds))

    def feed(self, now, buttons, axes, publish=False):
        result = self.parser.update(axes, buttons, now)
        self.executor.apply(result, now)
        if publish:
            self.executor.publish_chassis(now)
        return result

    def boot(self, now=10.0):
        return self.feed(now, make_buttons(), make_axes())

    def state_events(self):
        return [message.data.split(" ", 1)[0] for message in self.pubs["state"].messages]

    def test_default_yaml_home_restore_brightness_is_point_two(self):
        config = yaml.safe_load((ROOT / "config" / "default.yaml").read_text())
        self.assertEqual(0.2, config["light"]["home_restore_brightness"])
        self.assertEqual(0.2, config["light"]["brightness_default"])

    def test_parser_reports_normalized_axes_and_gear_without_twist_scale(self):
        self.boot(10.0)
        low = self.feed(
            10.1,
            make_buttons(left_shoulder=True),
            make_axes(left_x=0.05, left_y=1.0),
        )
        self.assertTrue(low.chassis.active)
        self.assertEqual(SPEED_GEAR_LOW, low.chassis.speed_gear)
        self.assertEqual(0.0, low.chassis.left_x)
        self.assertEqual(1.0, low.chassis.left_y)

        both = self.feed(
            10.2,
            make_buttons(left_shoulder=True, right_shoulder=True),
            make_axes(left_y=-0.5),
        )
        self.assertTrue(both.chassis.active)
        self.assertEqual(SPEED_GEAR_HIGH, both.chassis.speed_gear)
        self.assertEqual(-0.5, both.chassis.left_y)

        released = self.feed(10.3, make_buttons(), make_axes(left_y=1.0))
        self.assertFalse(released.chassis.active)
        self.assertEqual(SPEED_GEAR_NONE, released.chassis.speed_gear)
        self.assertEqual(1.0, released.chassis.left_y)

    def test_held_button_on_first_sample_is_not_an_edge(self):
        result = self.feed(
            10.0,
            make_buttons(x=True, a=True, home=True, right_stick=True),
            make_axes(),
        )
        self.assertEqual([], event_types(result))
        self.assertFalse(any(event.get("type") == "r3" for event in result.events))
        self.assertFalse(self.parser.locked)
        self.assertEqual("none", self.parser.active_mode)

    def test_scripted_sequence_adjusts_speed_while_driving_and_releases_channel(self):
        self.boot(10.0)
        self.feed(
            10.1,
            make_buttons(a=True, right_shoulder=True),
            make_axes(left_y=1.0),
            publish=True,
        )
        self.assertEqual(0.4, self.pubs["cmd_vel"].messages[-1].linear.x)
        self.assertTrue(self.executor.chassis_command_active)

        adjusted = self.feed(
            10.4,
            make_buttons(right_shoulder=True),
            make_axes(left_y=1.0, left_trigger=-1.0, dpad_y=1.0),
            publish=True,
        )
        adjust_events = [event for event in event_list(adjusted) if event["type"] == EVENT_ADJUST]
        self.assertEqual(
            [{"type": EVENT_ADJUST, "target": TARGET_LINEAR_SPEED, "direction": 1}],
            adjust_events,
        )
        self.assertAlmostEqual(0.44, self.executor.max_linear_speed)
        self.assertAlmostEqual(0.44, self.pubs["cmd_vel"].messages[-1].linear.x)

        blocked = self.feed(
            10.5,
            make_buttons(right_shoulder=True),
            make_axes(left_y=1.0, left_trigger=-1.0, dpad_y=1.0),
            publish=True,
        )
        self.assertEqual([], [event for event in event_list(blocked) if event["type"] == EVENT_ADJUST])
        self.assertAlmostEqual(0.44, self.executor.max_linear_speed)

        centered = self.feed(
            10.8,
            make_buttons(right_shoulder=True),
            make_axes(left_y=0.0),
            publish=True,
        )
        self.assertTrue(centered.chassis.active)
        self.assertEqual(0.0, centered.chassis.left_y)
        self.assertEqual(0.0, self.pubs["cmd_vel"].messages[-1].linear.x)
        self.assertTrue(self.executor.chassis_command_active)
        published_while_centered = len(self.pubs["cmd_vel"].messages)
        self.executor.publish_chassis(10.85)
        self.assertEqual(published_while_centered + 1, len(self.pubs["cmd_vel"].messages))
        self.assertEqual(0.0, self.pubs["cmd_vel"].messages[-1].linear.x)

        released = self.feed(
            11.0,
            make_buttons(),
            make_axes(left_y=1.0),
            publish=True,
        )
        self.assertFalse(released.chassis.active)
        self.assertEqual(1.0, released.chassis.left_y)
        self.assertEqual(0.0, self.pubs["cmd_vel"].messages[-1].linear.x)
        self.assertFalse(self.executor.chassis_command_active)
        held_count = len(self.pubs["cmd_vel"].messages)
        self.executor.publish_chassis(11.05)
        self.assertEqual(held_count, len(self.pubs["cmd_vel"].messages))

    def test_low_gear_and_holonomic_twist_use_executor_limits(self):
        self.boot(10.0)
        low = self.feed(
            10.1,
            make_buttons(left_shoulder=True),
            make_axes(left_x=1.0, left_y=1.0),
        )
        twist = self.executor.make_chassis_twist(low.chassis)
        self.assertAlmostEqual(0.24, twist.linear.x)
        self.assertAlmostEqual(0.48, twist.angular.z)
        self.assertEqual(0.0, twist.linear.y)

        self.executor.chassis = dict(self.executor.chassis, holonomic=True)
        holo = self.executor.make_chassis_twist(low.chassis)
        self.assertAlmostEqual(0.24, holo.linear.y)
        self.assertEqual(0.0, holo.angular.z)

    def test_same_frame_mode_switch_adjusts_new_target_only(self):
        self.boot(10.0)
        self.feed(10.2, make_buttons(a=True), make_axes())
        self.feed(10.3, make_buttons(), make_axes())
        switched = self.feed(
            10.5,
            make_buttons(b=True),
            make_axes(left_trigger=-1.0, dpad_x=-1.0, dpad_y=1.0),
        )
        payload = event_list(switched)
        self.assertEqual(EVENT_MODE, payload[0]["type"])
        self.assertEqual("camera", payload[0]["mode"])
        self.assertEqual(
            [
                {"type": EVENT_ADJUST, "target": TARGET_GAIN, "direction": 1},
                {"type": EVENT_ADJUST, "target": TARGET_EXPOSURE, "direction": 1},
            ],
            payload[1:],
        )
        self.assertAlmostEqual(0.4, self.executor.max_linear_speed)
        self.assertAlmostEqual(11.0, self.executor.gain)
        self.assertAlmostEqual(5600.0, self.executor.exposure)
        self.assertEqual(
            ["事件=模式=底盘参数", "事件=模式=相机参数", "事件=相机增益=11.000", "事件=相机曝光=5600.0"],
            self.state_events(),
        )

    def test_same_frame_selects_chassis_before_speed_adjust(self):
        self.boot(10.0)
        selected = self.feed(
            10.4,
            make_buttons(a=True),
            make_axes(left_trigger=-1.0, dpad_y=1.0),
        )
        payload = event_list(selected)
        self.assertEqual(
            [EVENT_MODE, EVENT_ADJUST],
            [event["type"] for event in payload],
        )
        self.assertEqual(TARGET_LINEAR_SPEED, payload[1]["target"])
        self.assertAlmostEqual(0.44, self.executor.max_linear_speed)
        self.assertEqual(
            ["事件=模式=底盘参数", "事件=底盘线速度上限=0.440"],
            self.state_events(),
        )

    def test_positive_dpad_x_decreases_angular_limit(self):
        self.boot(10.0)
        self.feed(10.2, make_buttons(a=True), make_axes())
        self.feed(10.3, make_buttons(), make_axes())
        adjusted = self.feed(
            10.6,
            make_buttons(),
            make_axes(left_trigger=-1.0, dpad_x=1.0),
        )
        adjust_events = [event for event in event_list(adjusted) if event["type"] == EVENT_ADJUST]
        self.assertEqual(
            [{"type": EVENT_ADJUST, "target": "angular_speed", "direction": -1}],
            adjust_events,
        )
        self.assertAlmostEqual(0.72, self.executor.max_angular_speed)

    def test_x_lock_blocks_mode_home_and_adjust_but_not_drive_or_start(self):
        self.boot(10.0)
        self.feed(10.2, make_buttons(a=True), make_axes())
        locked = self.feed(10.3, make_buttons(x=True), make_axes())
        self.assertEqual([EVENT_LOCK], [event["type"] for event in event_list(locked)])
        self.assertTrue(locked.events[0]["locked"])
        self.assertEqual("none", self.parser.active_mode)
        self.assertTrue(self.executor.locked)
        self.assertEqual("none", self.executor.active_mode)

        while_locked = self.feed(
            10.5,
            make_buttons(b=True, home=True, start=True, right_shoulder=True),
            make_axes(left_y=1.0, left_trigger=-1.0, dpad_y=1.0),
            publish=True,
        )
        payload = event_list(while_locked)
        self.assertEqual([EVENT_START], [event["type"] for event in payload])
        self.assertEqual(7, payload[0]["index"])
        self.assertTrue(while_locked.chassis.active)
        self.assertEqual(SPEED_GEAR_HIGH, while_locked.chassis.speed_gear)
        self.assertAlmostEqual(0.4, self.pubs["cmd_vel"].messages[-1].linear.x)
        self.assertAlmostEqual(10.0, self.executor.gain)
        self.assertAlmostEqual(0.2, self.executor.brightness)
        self.assertTrue(any("START按键已识别 index=7 locked=是" in text for _level, text in self.logger.lines))
        self.assertTrue(any("已忽略" in text for _level, text in self.logger.lines))

        self.feed(10.6, make_buttons(), make_axes())
        same_frame_lock = self.feed(
            10.7,
            make_buttons(x=True, a=True),
            make_axes(left_trigger=-1.0, dpad_y=1.0),
        )
        self.assertFalse(self.parser.locked)
        self.assertEqual("chassis", self.parser.active_mode)
        self.assertEqual(
            [EVENT_LOCK, EVENT_MODE, EVENT_ADJUST],
            [event["type"] for event in event_list(same_frame_lock)],
        )
        self.assertAlmostEqual(0.44, self.executor.max_linear_speed)

    def test_same_frame_lock_suppresses_mode_that_arrives_together(self):
        self.boot(10.0)
        locked = self.feed(
            10.2,
            make_buttons(x=True, a=True, home=True),
            make_axes(left_trigger=-1.0, dpad_y=1.0),
        )
        self.assertTrue(self.parser.locked)
        self.assertEqual("none", self.parser.active_mode)
        self.assertEqual([EVENT_LOCK], [event["type"] for event in event_list(locked)])
        self.assertAlmostEqual(0.4, self.executor.max_linear_speed)

    def test_r3_gestures_do_not_emit_events_or_change_executor(self):
        self.boot(1.0)
        before = (
            self.executor.max_linear_speed,
            self.executor.max_angular_speed,
            self.executor.light_on,
            self.executor.brightness,
            self.executor.locked,
        )
        samples = (
            (1.10, True),
            (1.20, False),
            (1.30, True),
            (1.40, False),
            (2.10, True),
            (3.70, True),
            (3.80, False),
        )
        for now, pressed in samples:
            result = self.feed(now, make_buttons(right_stick=pressed), make_axes())
            self.assertEqual([], event_types(result))
        self.assertEqual(before[0], self.executor.max_linear_speed)
        self.assertEqual(before[1], self.executor.max_angular_speed)
        self.assertEqual(before[2], self.executor.light_on)
        self.assertEqual(before[3], self.executor.brightness)
        self.assertEqual(before[4], self.executor.locked)
        self.assertFalse(hasattr(self.executor, "video_command_queue"))
        self.assertFalse(hasattr(self.executor, "video_gesture"))
        self.assertFalse(any("录像" in text or "R3" in text for _level, text in self.logger.lines))

    def test_home_write_order_uses_yaml_restore_brightness(self):
        self.boot(10.0)
        self.assertAlmostEqual(0.2, self.executor.brightness)
        self.assertTrue(self.executor.light_on)

        self.feed(10.2, make_buttons(home=True), make_axes())
        self.assertEqual(
            [
                ("brightness", 0.0),
                ("sleep", 0.05),
                ("power", False),
                ("sleep", 0.05),
                ("power", False),
            ],
            self.timeline,
        )
        self.assertFalse(self.executor.light_on)
        self.assertIn("事件=HOME关闭光源，亮度=0", self.state_events())

        self.timeline[:] = []
        self.feed(10.3, make_buttons(), make_axes())
        self.feed(10.4, make_buttons(home=True), make_axes())
        self.assertEqual(
            [
                ("power", True),
                ("sleep", 0.05),
                ("power", True),
                ("sleep", 0.05),
                ("brightness", 0.2),
            ],
            self.timeline,
        )
        self.assertAlmostEqual(0.2, self.executor.brightness)
        self.assertNotAlmostEqual(0.5, self.executor.brightness)
        self.assertTrue(self.executor.light_on)
        self.assertIn("事件=HOME恢复光源固定亮度=0.20", self.state_events())

    def test_release_after_timeout_does_not_publish_stick_command(self):
        self.boot(20.0)
        self.feed(
            20.1,
            make_buttons(right_shoulder=True),
            make_axes(left_y=1.0),
            publish=True,
        )
        self.assertAlmostEqual(0.4, self.pubs["cmd_vel"].messages[-1].linear.x)
        self.executor.publish_chassis(20.7)
        self.assertEqual(0.0, self.pubs["cmd_vel"].messages[-1].linear.x)
        self.assertFalse(self.executor.chassis_command_active)
        count = len(self.pubs["cmd_vel"].messages)
        self.executor.publish_chassis(20.8)
        self.assertEqual(count, len(self.pubs["cmd_vel"].messages))

    def test_boundaries_stay_in_their_modules(self):
        parser_source = (ROOT / "src" / "teleop_joy_ctrl" / "joy_parser.py").read_text()
        executor_source = (ROOT / "src" / "teleop_joy_ctrl" / "teleop_executor.py").read_text()
        node_source = (ROOT / "scripts" / "teleop_joy_ctrl_node.py").read_text()
        launch_source = (ROOT / "launch" / "teleop_joy_ctrl.launch").read_text()
        mux_source = (ROOT / "config" / "twist_mux_takeover.yaml").read_text()

        self.assertNotIn("import rospy", parser_source)
        self.assertNotIn("geometry_msgs", parser_source)
        self.assertNotIn("max_linear_speed", parser_source)
        self.assertIn("def make_chassis_twist", executor_source)
        self.assertIn("def connect", executor_source)
        self.assertIn("wait_for_service", executor_source)
        self.assertNotIn("wait_for_service", parser_source)
        self.assertNotIn("wait_for_service", node_source)
        self.assertIn("self.parser.update", node_source)
        self.assertIn("self.executor.apply", node_source)
        self.assertNotIn("init_video_clients", node_source)
        self.assertNotIn("R3GestureInterpreter", executor_source)
        self.assertNotIn("right_stick", parser_source)
        self.assertEqual(1, launch_source.count("teleop_joy_ctrl_node.py"))
        self.assertNotIn("joy_parser", launch_source)
        self.assertNotIn("video_recording_manager", launch_source)
        self.assertNotIn("hybrid_video", launch_source)
        self.assertNotIn("start_video_manager", launch_source)
        self.assertIn("priority: 254", mux_source)

    def test_node_callback_forwards_one_joy_sample(self):
        rospy = sys.modules.get("rospy")
        sensor_msgs = sys.modules.get("sensor_msgs")
        sensor_msgs_msg = sys.modules.get("sensor_msgs.msg")
        try:
            import types
            fake_rospy = types.ModuleType("rospy")

            class Stamp(object):
                def __init__(self, value):
                    self.value = value

                def to_sec(self):
                    return self.value

            fake_rospy.Time = types.SimpleNamespace(now=staticmethod(lambda: Stamp(12.5)))
            fake_sensor = types.ModuleType("sensor_msgs")
            fake_sensor_msg = types.ModuleType("sensor_msgs.msg")

            class Joy(object):
                def __init__(self):
                    self.axes = []
                    self.buttons = []

            fake_sensor_msg.Joy = Joy
            sys.modules["rospy"] = fake_rospy
            sys.modules["sensor_msgs"] = fake_sensor
            sys.modules["sensor_msgs.msg"] = fake_sensor_msg

            import importlib.util
            source = ROOT / "scripts" / "teleop_joy_ctrl_node.py"
            spec = importlib.util.spec_from_file_location("teleop_node_under_test", source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            seen = {}

            class Parser(object):
                def update(self, axes, buttons, now):
                    seen["parsed"] = (list(axes), list(buttons), now)
                    return "intent"

            class Executor(object):
                def apply(self, result, now):
                    seen["applied"] = (result, now)

                def publish_chassis(self, now):
                    seen["published"] = now

            node = module.TeleopJoyCtrl.__new__(module.TeleopJoyCtrl)
            node.parser = Parser()
            node.executor = Executor()
            msg = Joy()
            msg.axes = [0.2, 0.4]
            msg.buttons = [1, 0]
            node.joy_callback(msg)
            node.publish_loop(None)

            self.assertEqual(([0.2, 0.4], [1, 0], 12.5), seen["parsed"])
            self.assertEqual(("intent", 12.5), seen["applied"])
            self.assertEqual(12.5, seen["published"])
            self.assertFalse(hasattr(module, "R3GestureInterpreter"))
        finally:
            if rospy is None:
                sys.modules.pop("rospy", None)
            else:
                sys.modules["rospy"] = rospy
            if sensor_msgs is None:
                sys.modules.pop("sensor_msgs", None)
            else:
                sys.modules["sensor_msgs"] = sensor_msgs
            if sensor_msgs_msg is None:
                sys.modules.pop("sensor_msgs.msg", None)
            else:
                sys.modules["sensor_msgs.msg"] = sensor_msgs_msg


if __name__ == "__main__":
    unittest.main()
