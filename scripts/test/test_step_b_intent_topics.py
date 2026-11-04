#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import threading
import time
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from teleop_joy_ctrl.intent_codec import (
    decode_chassis,
    decode_events,
    encode_chassis,
    encode_events,
    events_message,
    frame_has_discrete_content,
)
from teleop_joy_ctrl.joy_parser import (
    EVENT_ADJUST,
    EVENT_HOME,
    EVENT_LOCK,
    EVENT_MODE,
    EVENT_R3,
    EVENT_START,
    ChassisIntent,
    JoyParser,
    R3_HOLD,
    R3_IDLE,
    R3_PRESS,
    R3_RELEASE,
    R3_SYNC,
    SPEED_GEAR_HIGH,
    SPEED_GEAR_LOW,
    TARGET_EXPOSURE,
    TARGET_GAIN,
)
from teleop_joy_ctrl.teleop_executor import (
    VIDEO_COMMAND_PAUSE,
    VIDEO_COMMAND_START,
    VIDEO_COMMAND_STOP,
    VIDEO_PAUSED,
    VIDEO_RECORDING,
    TeleopExecutor,
)


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
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


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


def make_parser():
    return JoyParser(
        axes=AXES,
        buttons=BUTTONS,
        axis_deadzone=0.08,
        adjust_repeat_interval=0.20,
        left_trigger_pressed_threshold=0.95,
    )


def make_executor():
    pubs = {
        "cmd_vel": RecordingPublisher(),
        "state": RecordingPublisher(),
        "debug": RecordingPublisher(),
        "exposure": RecordingPublisher(),
        "gain": RecordingPublisher(),
        "light_power": RecordingPublisher(),
        "light_brightness": RecordingPublisher(),
    }
    logger = ListLogger()
    executor = TeleopExecutor(
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
        video={
            "enabled": True,
            "double_click_interval": 0.4,
            "long_press_duration": 1.5,
        },
        joy_timeout=0.5,
        debug_success_logs=True,
        chassis_debug_interval=1.0,
        cmd_vel_topic="/teleop_joy_ctrl/cmd_vel",
        publishers=pubs,
        sleeper=lambda _seconds: None,
        logger=logger,
        twist_cls=Twist,
        bool_cls=Message,
        float_cls=Message,
        string_cls=Message,
    )
    return executor, pubs, logger


def snapshot(executor, pubs):
    twists = [
        (message.linear.x, message.linear.y, message.angular.z)
        for message in pubs["cmd_vel"].messages
    ]
    return {
        "twists": twists,
        "states": [message.data.split(" ", 1)[0] for message in pubs["state"].messages],
        "power": [message.data for message in pubs["light_power"].messages],
        "brightness_cmd": [message.data for message in pubs["light_brightness"].messages],
        "gain_cmd": [message.data for message in pubs["gain"].messages],
        "exposure_cmd": [message.data for message in pubs["exposure"].messages],
        "linear_limit": executor.max_linear_speed,
        "angular_limit": executor.max_angular_speed,
        "gain": executor.gain,
        "exposure": executor.exposure,
        "brightness": executor.brightness,
        "light_on": executor.light_on,
        "active": executor.chassis_command_active,
        "locked": executor.locked,
        "video_queue": list(executor.video_command_queue.queue),
        "video_input": executor.video_input_state,
    }


class StepBIntentTopicTest(unittest.TestCase):
    def test_chassis_json_carries_normalized_axes_not_twist(self):
        raw = encode_chassis(ChassisIntent(True, SPEED_GEAR_HIGH, 0.0, 1.0), 12.5)
        payload = json.loads(raw)
        self.assertEqual(
            {
                "active": True,
                "axes": {"left_x": 0.0, "left_y": 1.0},
                "gear": "high",
                "left_x": 0.0,
                "left_y": 1.0,
                "speed_gear": "high",
                "time": 12.5,
            },
            payload,
        )
        self.assertNotIn("linear", raw)
        self.assertNotIn("angular", raw)
        self.assertNotIn("cmd_vel", raw)
        self.assertNotIn("max_linear", raw)

        intent, stamp = decode_chassis(raw)
        self.assertTrue(intent.active)
        self.assertEqual(SPEED_GEAR_HIGH, intent.speed_gear)
        self.assertEqual(0.0, intent.left_x)
        self.assertEqual(1.0, intent.left_y)
        self.assertEqual(12.5, stamp)

    def test_gear_alias_and_nested_axes_round_trip_into_executor_limits(self):
        executor, pubs, _logger = make_executor()
        intent, stamp = decode_chassis(
            '{"active":true,"gear":"low","axes":{"left_x":1,"left_y":1}}'
        )
        self.assertIsNone(stamp)
        self.assertEqual(SPEED_GEAR_LOW, intent.speed_gear)
        executor.apply_chassis(intent, 5.0)
        executor.publish_chassis(5.0)
        self.assertAlmostEqual(0.24, pubs["cmd_vel"].messages[-1].linear.x)
        self.assertAlmostEqual(0.48, pubs["cmd_vel"].messages[-1].angular.z)
        self.assertNotAlmostEqual(1.0, pubs["cmd_vel"].messages[-1].linear.x)

        overridden, overridden_stamp = decode_chassis(
            '{"active":true,"speed_gear":"high","gear":"low","left_y":1,"time":6}'
        )
        self.assertEqual(6.0, overridden_stamp)
        self.assertEqual(SPEED_GEAR_HIGH, overridden.speed_gear)
        twist = executor.make_chassis_twist(overridden)
        self.assertAlmostEqual(0.4, twist.linear.x)

    def test_event_channel_accepts_one_ordered_batch_or_a_single_object(self):
        batch = encode_events(
            [
                {"type": EVENT_MODE, "mode": "camera"},
                {"type": EVENT_ADJUST, "target": TARGET_GAIN, "direction": 1},
                {"type": EVENT_ADJUST, "target": TARGET_EXPOSURE, "direction": 1},
            ],
            10.5,
            seq=4,
        )
        payload = json.loads(batch)
        self.assertEqual(4, payload["seq"])
        self.assertEqual(
            [EVENT_MODE, EVENT_ADJUST, EVENT_ADJUST],
            [event["type"] for event in payload["events"]],
        )
        self.assertEqual(payload["events"], decode_events(batch))
        self.assertEqual([{"type": EVENT_HOME}], decode_events('{"type":"home"}'))
        self.assertEqual(
            [
                {"type": EVENT_LOCK, "locked": True},
                {"type": EVENT_START, "index": 7},
            ],
            decode_events(
                '[{"type":"lock","locked":true},{"type":"start","index":7}]'
            ),
        )
        self.assertIsNone(decode_events("not-json"))
        self.assertIsNone(decode_events('{"seq":1,"time":1}'))
        self.assertEqual([], decode_events('{"events":[]}'))
        self.assertIsNone(decode_chassis("not-json"))
        self.assertIsNone(decode_chassis('{"linear":{"x":1}}'))

    def test_executor_applies_one_event_message_in_array_order(self):
        executor, pubs, _logger = make_executor()
        events = decode_events(
            json.dumps(
                {
                    "time": 10.5,
                    "seq": 2,
                    "events": [
                        {"type": "mode", "mode": "camera"},
                        {"type": "adjust", "target": "gain", "direction": 1},
                        {"type": "adjust", "target": "exposure", "direction": 1},
                    ],
                }
            )
        )
        executor.apply_events(events)
        self.assertAlmostEqual(11.0, executor.gain)
        self.assertAlmostEqual(5600.0, executor.exposure)
        self.assertAlmostEqual(0.4, executor.max_linear_speed)
        self.assertEqual(
            ["事件=模式=相机参数", "事件=相机增益=11.000", "事件=相机曝光=5600.0"],
            [message.data.split(" ", 1)[0] for message in pubs["state"].messages],
        )
        self.assertEqual([], pubs["cmd_vel"].messages)

    def test_single_home_event_uses_yaml_restore_brightness(self):
        config = yaml.safe_load((ROOT / "config" / "default.yaml").read_text())
        self.assertEqual(0.2, config["light"]["home_restore_brightness"])
        executor, pubs, _logger = make_executor()
        self.assertAlmostEqual(0.2, executor.brightness)
        executor.apply_events(decode_events('{"type":"home"}'))
        self.assertEqual([0.0], [message.data for message in pubs["light_brightness"].messages])
        self.assertEqual([False, False], [message.data for message in pubs["light_power"].messages])
        self.assertFalse(executor.light_on)
        pubs["light_power"].messages[:] = []
        pubs["light_brightness"].messages[:] = []
        executor.apply_events(decode_events('{"type":"home"}'))
        self.assertEqual([True, True], [message.data for message in pubs["light_power"].messages])
        self.assertEqual([0.2], [message.data for message in pubs["light_brightness"].messages])
        self.assertAlmostEqual(0.2, executor.brightness)
        self.assertTrue(executor.light_on)

    def test_active_zero_axes_keep_the_channel_and_release_publishes_one_zero(self):
        executor, pubs, _logger = make_executor()
        held, held_stamp = decode_chassis(
            encode_chassis(ChassisIntent(True, SPEED_GEAR_HIGH, 0.0, 0.0), 10.0)
        )
        executor.apply_chassis(held, held_stamp)
        executor.publish_chassis(10.0)
        executor.publish_chassis(10.1)
        self.assertEqual(2, len(pubs["cmd_vel"].messages))
        self.assertEqual(0.0, pubs["cmd_vel"].messages[-1].linear.x)
        self.assertTrue(executor.chassis_command_active)

        released, released_stamp = decode_chassis(
            encode_chassis(ChassisIntent(False, "none", 0.0, 1.0), 10.2)
        )
        executor.apply_chassis(released, released_stamp)
        executor.publish_chassis(10.2)
        self.assertEqual(0.0, pubs["cmd_vel"].messages[-1].linear.x)
        self.assertFalse(executor.chassis_command_active)
        held_count = len(pubs["cmd_vel"].messages)
        executor.publish_chassis(10.3)
        self.assertEqual(held_count, len(pubs["cmd_vel"].messages))

    def test_stale_chassis_stamp_releases_without_a_joystick(self):
        executor, pubs, _logger = make_executor()
        intent, stamp = decode_chassis(
            encode_chassis(ChassisIntent(True, SPEED_GEAR_HIGH, 0.0, 1.0), 20.1)
        )
        executor.apply_chassis(intent, stamp)
        executor.publish_chassis(20.1)
        self.assertAlmostEqual(0.4, pubs["cmd_vel"].messages[-1].linear.x)
        executor.publish_chassis(20.7)
        self.assertEqual(0.0, pubs["cmd_vel"].messages[-1].linear.x)
        self.assertFalse(executor.chassis_command_active)
        count = len(pubs["cmd_vel"].messages)
        executor.publish_chassis(20.8)
        self.assertEqual(count, len(pubs["cmd_vel"].messages))

    def test_parser_frames_keep_order_and_skip_pure_idle(self):
        parser = make_parser()
        boot = parser.update(make_axes(), make_buttons(), 10.0)
        self.assertEqual(R3_SYNC, boot.events[0]["phase"])
        self.assertTrue(frame_has_discrete_content(boot.events))
        idle = parser.update(make_axes(left_y=1.0), make_buttons(right_shoulder=True), 10.1)
        self.assertEqual([EVENT_R3], [event["type"] for event in idle.events])
        self.assertEqual(R3_IDLE, idle.events[0]["phase"])
        self.assertFalse(frame_has_discrete_content(idle.events))
        self.assertIsNone(events_message(idle.events, 10.1, 2))

        switched = parser.update(
            make_axes(left_trigger=-1.0, dpad_x=-1.0, dpad_y=1.0),
            make_buttons(b=True),
            10.5,
        )
        self.assertTrue(frame_has_discrete_content(switched.events))
        raw = events_message(switched.events, 10.5, 2)
        decoded = decode_events(raw)
        self.assertEqual(switched.events, decoded)
        self.assertEqual(
            [EVENT_R3, EVENT_MODE, EVENT_ADJUST, EVENT_ADJUST],
            [event["type"] for event in decoded],
        )
        self.assertEqual(R3_IDLE, decoded[0]["phase"])
        self.assertEqual("camera", decoded[1]["mode"])
        self.assertEqual(
            [
                {"type": EVENT_ADJUST, "target": TARGET_GAIN, "direction": 1},
                {"type": EVENT_ADJUST, "target": TARGET_EXPOSURE, "direction": 1},
            ],
            decoded[2:],
        )
        self.assertEqual(2, json.loads(raw)["seq"])

    def test_topic_delivery_matches_in_process_executor(self):
        direct, direct_pubs, direct_logs = make_executor()
        remote, remote_pubs, remote_logs = make_executor()
        samples = [
            (10.0, make_buttons(), make_axes(), False),
            (10.2, make_buttons(a=True), make_axes(), False),
            (10.3, make_buttons(), make_axes(), False),
            (10.4, make_buttons(right_shoulder=True), make_axes(left_y=1.0), True),
            (
                10.8,
                make_buttons(right_shoulder=True),
                make_axes(left_y=1.0, left_trigger=-1.0, dpad_y=1.0),
                True,
            ),
            (
                10.9,
                make_buttons(right_shoulder=True),
                make_axes(left_y=1.0, left_trigger=-1.0, dpad_y=1.0),
                True,
            ),
            (11.2, make_buttons(right_shoulder=True), make_axes(), True),
            (11.4, make_buttons(), make_axes(left_y=1.0), True),
            (
                12.0,
                make_buttons(b=True),
                make_axes(left_trigger=-1.0, dpad_x=-1.0, dpad_y=1.0),
                False,
            ),
            (12.2, make_buttons(), make_axes(), False),
            (13.0, make_buttons(home=True), make_axes(), False),
            (13.1, make_buttons(), make_axes(), False),
            (13.2, make_buttons(home=True), make_axes(), False),
            (14.0, make_buttons(x=True), make_axes(), False),
            (
                14.2,
                make_buttons(b=True, home=True, start=True, right_shoulder=True),
                make_axes(left_y=1.0, left_trigger=-1.0, dpad_y=1.0),
                True,
            ),
        ]
        self._run_script(direct, samples, via_topic=False)
        self._run_script(remote, samples, via_topic=True)

        self.assertEqual(snapshot(direct, direct_pubs), snapshot(remote, remote_pubs))
        self.assertEqual(direct_logs.lines, remote_logs.lines)
        remote_shot = snapshot(remote, remote_pubs)
        self.assertAlmostEqual(0.44, remote_shot["linear_limit"])
        self.assertAlmostEqual(11.0, remote_shot["gain"])
        self.assertAlmostEqual(5600.0, remote_shot["exposure"])
        self.assertAlmostEqual(0.2, remote_shot["brightness"])
        self.assertTrue(remote_shot["light_on"])
        self.assertTrue(remote_shot["locked"])
        self.assertAlmostEqual(0.44, remote_pubs["cmd_vel"].messages[-1].linear.x)
        self.assertTrue(remote.chassis_command_active)
        self.assertTrue(any("已忽略" in text for _level, text in remote_logs.lines))
        self.assertTrue(
            any("START按键已识别 index=7 locked=是" in text for _level, text in remote_logs.lines)
        )

    def test_r3_hold_and_release_arrive_as_gesture_events(self):
        parser = make_parser()
        executor, _pubs, logger = make_executor()
        executor.video_state = VIDEO_RECORDING
        executor.video_input_state = VIDEO_RECORDING
        samples = [
            (2.0, make_buttons(), make_axes()),
            (2.10, make_buttons(right_stick=True), make_axes()),
            (3.70, make_buttons(right_stick=True), make_axes()),
        ]
        phases = []
        held_for = None
        for now, buttons, axes in samples:
            result = parser.update(axes, buttons, now)
            phases.append(result.events[0]["phase"])
            raw = events_message(result.events, now, len(phases))
            self.assertIsNotNone(raw)
            decoded = decode_events(raw)
            self.assertEqual(EVENT_R3, decoded[0]["type"])
            self.assertNotIn("buttons", raw)
            if decoded[0]["phase"] == R3_HOLD:
                held_for = decoded[0]["held_for"]
            executor.apply_events(decoded)
        self.assertEqual([R3_SYNC, R3_PRESS, R3_HOLD], phases)
        self.assertGreaterEqual(held_for, 1.5)
        self.assertEqual([VIDEO_COMMAND_STOP], list(executor.video_command_queue.queue))
        self.assertTrue(any("R3录像指令已识别" in text for _level, text in logger.lines))

        release_parser = make_parser()
        release_executor, _pubs, release_logger = make_executor()
        release_executor.video_state = VIDEO_RECORDING
        release_executor.video_input_state = VIDEO_RECORDING
        for now, buttons in ((1.0, make_buttons()), (1.10, make_buttons(right_stick=True)), (1.20, make_buttons())):
            result = release_parser.update(make_axes(), buttons, now)
            release_executor.apply_events(decode_events(events_message(result.events, now, 1)))
        self.assertEqual(R3_RELEASE, result.events[0]["phase"])
        self.assertFalse(result.events[0]["pressed"])
        self.assertEqual([VIDEO_COMMAND_PAUSE], list(release_executor.video_command_queue.queue))
        self.assertEqual(VIDEO_PAUSED, release_executor.video_input_state)
        self.assertTrue(any("R3录像指令已识别" in text for _level, text in release_logger.lines))

    def test_skipped_idle_frames_still_honor_double_click_window(self):
        inside, _pubs, _logger = make_executor()
        self.assertEqual(
            [VIDEO_COMMAND_START],
            self._apply_published_r3(
                inside,
                [
                    (0.00, False),
                    (0.10, True),
                    (0.18, False),
                    (0.22, False),
                    (0.30, True),
                ],
            ),
        )

        outside, _pubs, _logger = make_executor()
        self.assertEqual(
            [],
            self._apply_published_r3(
                outside,
                [
                    (0.00, False),
                    (0.10, True),
                    (0.18, False),
                    (0.70, False),
                    (0.80, True),
                    (0.90, False),
                ],
            ),
        )

    def test_arm_worker_keeps_chassis_publish_off_the_blocking_path(self):
        executor, pubs, _logger = make_executor()
        entered = threading.Event()
        release = threading.Event()
        finished = threading.Event()

        def slow():
            entered.set()
            release.wait(2.0)
            finished.set()

        executor.handle_arm_start = slow
        executor.start_arm_worker()
        try:
            executor._handle_start({"type": EVENT_START, "index": 7})
            self.assertTrue(entered.wait(1.0))
            self.assertFalse(finished.is_set())
            executor.apply_chassis(ChassisIntent(True, SPEED_GEAR_HIGH, 0.0, 1.0), 10.0)
            began = time.monotonic()
            executor.publish_chassis(10.0)
            self.assertLess(time.monotonic() - began, 0.2)
            self.assertFalse(finished.is_set())
            self.assertAlmostEqual(0.4, pubs["cmd_vel"].messages[-1].linear.x)
        finally:
            release.set()
            executor.stop_arm_worker()
        self.assertTrue(finished.wait(1.0))

    def test_yaml_topics_and_launch_keep_existing_defaults(self):
        config = yaml.safe_load((ROOT / "config" / "default.yaml").read_text())
        self.assertEqual("/teleop_joy_ctrl/chassis_axes", config["chassis_axes_topic"])
        self.assertEqual("/teleop_joy_ctrl/events", config["events_topic"])
        self.assertEqual("/teleop_joy_ctrl/cmd_vel", config["cmd_vel_topic"])
        self.assertEqual(0.4, config["chassis"]["max_linear_speed"])
        self.assertEqual(0.8, config["chassis"]["max_angular_speed"])
        self.assertEqual(0.60, config["chassis"]["low_speed_scale"])
        self.assertEqual(0.08, config["axis_deadzone"])
        self.assertEqual(0.20, config["adjust_repeat_interval"])
        self.assertEqual(
            {
                "a": 0,
                "b": 1,
                "x": 2,
                "y": 3,
                "left_shoulder": 4,
                "right_shoulder": 5,
                "start": 7,
                "home": 8,
                "right_stick": 10,
            },
            config["buttons"],
        )
        self.assertEqual(
            {"left_x": 0, "left_y": 1, "left_trigger": 2, "dpad_x": 6, "dpad_y": 7},
            config["axes"],
        )

        launch = (ROOT / "launch" / "teleop_joy_ctrl.launch").read_text()
        intent = (ROOT / "scripts" / "joy_intent_node.py").read_text()
        executor_node = (ROOT / "scripts" / "teleop_executor_node.py").read_text()
        codec = (ROOT / "src" / "teleop_joy_ctrl" / "intent_codec.py").read_text()
        mux = (ROOT / "config" / "twist_mux_takeover.yaml").read_text()

        self.assertIn('type="joy_intent_node.py"', launch)
        self.assertIn('type="teleop_executor_node.py"', launch)
        self.assertIn('legacy_single_node" default="false"', launch)
        self.assertEqual(1, launch.count("teleop_joy_ctrl_node.py"))
        self.assertNotIn("joy_parser", launch)
        self.assertIn("camera_workspace", launch)
        self.assertIn("python_executable", launch)
        self.assertIn("start_arm_bringup", launch)
        self.assertIn("priority: 254", mux)

        self.assertNotIn("import rospy", codec)
        self.assertNotIn("geometry_msgs", codec)
        self.assertNotIn("cmd_vel", codec)
        self.assertNotIn("max_linear_speed", codec)
        self.assertNotIn("geometry_msgs", intent)
        self.assertNotIn("Twist", intent)
        self.assertNotIn("TeleopExecutor", intent)
        self.assertNotIn("cmd_vel", intent)
        self.assertGreaterEqual(intent.count("latch=False"), 2)
        self.assertIn("events_message", intent)
        self.assertIn("encode_chassis", intent)

        self.assertIn("start_arm_worker", executor_node)
        self.assertIn("stop_arm_worker", executor_node)
        self.assertNotIn("init_arm_clients", executor_node)
        self.assertNotIn("wait_for_service", executor_node)
        self.assertNotIn("sensor_msgs", executor_node)
        self.assertIn("无法解析底盘意图", executor_node)
        self.assertIn("无法解析离散事件", executor_node)
        self.assertIn("if stamp is None", executor_node)

    def _apply_published_r3(self, executor, samples):
        parser = make_parser()
        for now, pressed in samples:
            result = parser.update(
                make_axes(),
                make_buttons(right_stick=pressed),
                now,
            )
            payload = events_message(result.events, now, 1)
            if result.events[0]["phase"] == R3_IDLE:
                self.assertIsNone(payload)
                continue
            self.assertIsNotNone(payload)
            executor.apply_events(decode_events(payload))
        return list(executor.video_command_queue.queue)

    def _run_script(self, executor, samples, via_topic):
        parser = make_parser()
        seq = 0
        for now, buttons, axes, publish in samples:
            result = parser.update(axes, buttons, now)
            if via_topic:
                intent, stamp = decode_chassis(encode_chassis(result.chassis, now))
                executor.apply_chassis(intent, stamp)
                payload = events_message(result.events, now, seq + 1)
                if payload is not None:
                    seq += 1
                    executor.apply_events(decode_events(payload))
            else:
                executor.apply(result, now)
            if publish:
                executor.publish_chassis(now)


if __name__ == "__main__":
    unittest.main()
