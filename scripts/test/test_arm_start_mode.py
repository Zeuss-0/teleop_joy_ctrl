#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""START 使能判断。

库存 dobot_v4_bringup 的 RobotMode 不写 mode，只把 TCP err_id 放进 res。
这些测试在 mode=0、res=0 时，要求用 FeedInfo / RobotStatus 区分上、下使能，
并在模式 9/11 时仍然清错。
"""

import json
import sys
import time
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from teleop_joy_ctrl.teleop_executor import (  # noqa: E402
    MODE_DISABLED,
    MODE_ENABLE_IDLE,
    MODE_ERROR,
    MODE_COLLISION,
    TeleopExecutor,
    resolve_robot_mode,
)


class ServiceException(Exception):
    pass


class ListLogger(object):
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warn(self, text):
        self.lines.append(("warn", text))

    def error(self, text):
        self.lines.append(("error", text))


class ModeResponse(object):
    def __init__(self, mode, res):
        self.mode = mode
        self.res = res


class ArmClients(object):
    def __init__(self, mode=0, res=0, fail=False):
        self.mode = mode
        self.res = res
        self.fail = fail
        self.enabled = 0
        self.disabled = 0
        self.cleared = 0

    def robot_mode(self):
        if self.fail:
            raise ServiceException("RobotMode down")
        return ModeResponse(self.mode, self.res)

    def enable(self):
        self.enabled += 1

    def disable(self):
        self.disabled += 1

    def clear(self):
        self.cleared += 1


def make_executor(clients, logger=None):
    executor = TeleopExecutor(
        arm={"enabled": True, "command_settle_time": 0},
        sleeper=lambda _seconds: None,
        logger=logger or ListLogger(),
        debug_success_logs=True,
    )
    executor.arm_clients_ready = True
    executor.arm_gateway.enable_client = clients.enable
    executor.arm_gateway.disable_client = clients.disable
    executor.arm_gateway.clear_error_client = clients.clear
    executor.arm_gateway.robot_mode_client = clients.robot_mode
    return executor


class ArmStartModeTest(unittest.TestCase):
    def test_resolve_prefers_filled_service_mode_then_feed_then_flags(self):
        self.assertEqual(9, resolve_robot_mode(9, feed_mode=5, feed_enable=1))
        self.assertEqual(5, resolve_robot_mode(0, feed_mode=5, feed_enable=1))
        self.assertEqual(11, resolve_robot_mode(0, feed_mode=11, feed_enable=0))
        self.assertEqual(MODE_ENABLE_IDLE, resolve_robot_mode(0, feed_enable=1))
        self.assertEqual(MODE_DISABLED, resolve_robot_mode(0, feed_enable=0))
        self.assertEqual(MODE_ENABLE_IDLE, resolve_robot_mode(0, status_enable=True))
        self.assertIsNone(resolve_robot_mode(0, status_enable=False))
        self.assertIsNone(resolve_robot_mode(0))
        self.assertEqual(5, resolve_robot_mode(None, feed_mode="5"))

    def test_broken_robot_mode_with_enabled_feed_disables(self):
        clients = ArmClients(mode=0, res=0)
        logger = ListLogger()
        executor = make_executor(clients, logger)
        payload = json.dumps({
            "robot_mode": 5,
            "EnableStatus": 1,
            "q_actual": [0, 0, 0, 0, 0, 0],
        })
        self.assertTrue(executor.ingest_feed_info(payload))
        self.assertEqual(MODE_ENABLE_IDLE, executor.query_robot_mode())
        executor.handle_arm_start()
        self.assertEqual(0, clients.enabled)
        self.assertEqual(1, clients.disabled)
        self.assertEqual(0, clients.cleared)
        self.assertTrue(any("FeedInfo/RobotStatus" in text for _level, text in logger.lines))

    def test_res_is_not_treated_as_mode(self):
        clients = ArmClients(mode=0, res=5)
        executor = make_executor(clients)
        executor.handle_arm_start()
        self.assertEqual(1, clients.enabled)
        self.assertEqual(0, clients.disabled)
        self.assertEqual(0, clients.cleared)

    def test_filled_service_mode_wins_over_feed_and_res(self):
        clients = ArmClients(mode=9, res=0)
        executor = make_executor(clients)
        executor.ingest_feed_info({"robot_mode": 5, "EnableStatus": 1})
        executor.handle_arm_start()
        self.assertEqual(1, clients.cleared)
        self.assertEqual(0, clients.enabled)
        self.assertEqual(0, clients.disabled)

        clients = ArmClients(mode=5, res=0)
        executor = make_executor(clients)
        executor.handle_arm_start()
        self.assertEqual(1, clients.disabled)
        self.assertEqual(0, clients.enabled)

    def test_start_toggles_enable_then_disable_from_feed(self):
        clients = ArmClients(mode=0, res=0)
        logger = ListLogger()
        executor = make_executor(clients, logger)

        def enable():
            clients.enabled += 1
            executor.ingest_feed_info({"robot_mode": 5, "EnableStatus": 1})

        def disable():
            clients.disabled += 1
            executor.ingest_feed_info({"robot_mode": 4, "EnableStatus": 0})

        executor.arm_gateway.enable_client = enable
        executor.arm_gateway.disable_client = disable
        executor.ingest_feed_info({"robot_mode": 4, "EnableStatus": 0})

        executor.handle_arm_start()
        self.assertEqual(1, clients.enabled)
        self.assertEqual(0, clients.disabled)
        self.assertTrue(any(
            "上使能" in text and "模式=5" in text for _level, text in logger.lines))
        self.assertFalse(any("未确认进入使能态" in text for _level, text in logger.lines))

        executor.handle_arm_start()
        self.assertEqual(1, clients.disabled)
        self.assertTrue(any(
            "下使能" in text and "模式=4" in text for _level, text in logger.lines))
        self.assertFalse(any("可能被控制器拒绝" in text for _level, text in logger.lines))

    def test_disable_confirmation_warns_while_feed_stays_enabled(self):
        clients = ArmClients(mode=0, res=0)
        logger = ListLogger()
        executor = make_executor(clients, logger)
        executor.ingest_feed_info({"robot_mode": 5, "EnableStatus": 1})
        executor.handle_arm_start()
        self.assertEqual(1, clients.disabled)
        self.assertTrue(any("可能被控制器拒绝" in text for _level, text in logger.lines))

    def test_enable_status_and_robot_status_can_show_enabled(self):
        clients = ArmClients(mode=0, res=0)
        executor = make_executor(clients)
        executor.ingest_feed_info({"EnableStatus": 1})
        executor.handle_arm_start()
        self.assertEqual(1, clients.disabled)

        clients = ArmClients(mode=0, res=0)
        executor = make_executor(clients)
        executor.ingest_robot_status(True)
        executor.handle_arm_start()
        self.assertEqual(1, clients.disabled)
        self.assertEqual(0, clients.enabled)

        clients = ArmClients(mode=0, res=0)
        executor = make_executor(clients)
        executor.ingest_robot_status(False)
        executor.handle_arm_start()
        self.assertEqual(1, clients.enabled)
        self.assertEqual(0, clients.disabled)

    def test_feed_error_and_collision_clear_instead_of_toggle(self):
        for mode in (MODE_ERROR, MODE_COLLISION):
            with self.subTest(mode=mode):
                clients = ArmClients(mode=0, res=0)
                executor = make_executor(clients)
                executor.ingest_feed_info({
                    "robot_mode": mode,
                    "EnableStatus": 1,
                })
                executor.ingest_robot_status(True)
                executor.handle_arm_start()
                self.assertEqual(1, clients.cleared)
                self.assertEqual(0, clients.enabled)
                self.assertEqual(0, clients.disabled)

    def test_running_and_single_move_from_feed_disable(self):
        for mode in (7, 8):
            with self.subTest(mode=mode):
                clients = ArmClients(mode=0, res=0)
                executor = make_executor(clients)
                executor.ingest_feed_info({"RobotMode": mode, "EnableStatus": 1})
                executor.handle_arm_start()
                self.assertEqual(1, clients.disabled)
                self.assertEqual(0, clients.enabled)

    def test_service_failure_still_follows_fresh_feed(self):
        clients = ArmClients(fail=True)
        logger = ListLogger()
        executor = make_executor(clients, logger)
        executor.ingest_feed_info({"robot_mode": 5, "EnableStatus": 1})
        self.assertEqual(MODE_ENABLE_IDLE, executor.query_robot_mode())
        executor.handle_arm_start()
        self.assertEqual(1, clients.disabled)
        self.assertTrue(any("RobotMode服务调用失败" in text for _level, text in logger.lines))

        clients = ArmClients(fail=True)
        executor = make_executor(clients)
        self.assertEqual(-1, executor.query_robot_mode())
        executor.handle_arm_start()
        self.assertEqual(1, clients.enabled)
        self.assertEqual(0, clients.disabled)

    def test_stale_feed_is_not_used_to_disable(self):
        clients = ArmClients(mode=0, res=0)
        executor = make_executor(clients)
        executor.ingest_feed_info(
            {"robot_mode": 5, "EnableStatus": 1},
            now=time.monotonic() - 5,
        )
        executor.ingest_robot_status(True, now=time.monotonic() - 5)
        executor.handle_arm_start()
        self.assertEqual(1, clients.enabled)
        self.assertEqual(0, clients.disabled)

    def test_topic_callback_reads_string_data(self):
        clients = ArmClients(mode=0, res=0)
        executor = make_executor(clients)

        class Feed(object):
            data = '{"robot_mode": 4, "EnableStatus": 0}'

        class Status(object):
            is_enable = True

        executor._on_feed_info(Feed())
        self.assertEqual(MODE_DISABLED, executor.query_robot_mode())
        executor._on_robot_status(Status())
        # 数字模式仍然优先于 is_enable。
        self.assertEqual(MODE_DISABLED, executor.query_robot_mode())

    def test_default_yaml_sets_feedback_stale_time(self):
        config = yaml.safe_load((ROOT / "config" / "default.yaml").read_text())
        self.assertEqual(1.0, config["arm"]["feedback_stale_time"])
        self.assertEqual("/dobot_v4_bringup", config["arm"]["service_prefix"])


if __name__ == "__main__":
    unittest.main()
