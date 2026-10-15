#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from moma_ws.src.teleop_joy_ctrl.src.teleop_joy_ctrl.video_process_manager import (
    ERROR,
    IDLE,
    PAUSED,
    RECORDING,
    VideoProcessManager,
    call_with_timeout,
)


class FakeProcess(object):
    def __init__(self, name):
        self.name = name
        self.returncode = None

    def poll(self):
        return self.returncode


class VideoProcessManagerTest(unittest.TestCase):
    def make_subject(self):
        self.operations = []
        self.ready_results = {"camera": True, "detector": True}
        self.detector_results = {
            "pause": (True, "视频录制已暂停"),
            "resume": (True, "视频录制已继续"),
            "stop": (True, "/tmp/recordings/20260723_153012"),
        }

        def launch(name):
            self.operations.append("launch:" + name)
            return FakeProcess(name)

        def wait_ready(name, process):
            self.operations.append("ready:" + name)
            self.assertEqual(name, process.name)
            return self.ready_results[name]

        def detector_command(name):
            self.operations.append("detector:" + name)
            return self.detector_results[name]

        def stop_process(process):
            self.operations.append("stop:" + process.name)

        return VideoProcessManager(
            launch_process=launch,
            wait_ready=wait_ready,
            detector_command=detector_command,
            stop_process=stop_process,
        )

    def test_start_launches_camera_before_detector(self):
        subject = self.make_subject()

        success, message = subject.start()

        self.assertTrue(success)
        self.assertIn("已开始", message)
        self.assertEqual(RECORDING, subject.state)
        self.assertEqual(
            [
                "launch:camera",
                "ready:camera",
                "launch:detector",
                "ready:detector",
            ],
            self.operations,
        )

    def test_camera_start_failure_only_cleans_owned_camera(self):
        subject = self.make_subject()
        self.ready_results["camera"] = False

        success, message = subject.start()

        self.assertFalse(success)
        self.assertIn("相机", message)
        self.assertEqual(ERROR, subject.state)
        self.assertEqual(
            ["launch:camera", "ready:camera", "stop:camera"],
            self.operations,
        )
        self.assertIsNone(subject.camera_process)
        self.assertIsNone(subject.detector_process)

    def test_detector_start_failure_cleans_detector_then_camera(self):
        subject = self.make_subject()
        self.ready_results["detector"] = False

        success, message = subject.start()

        self.assertFalse(success)
        self.assertIn("YOLO", message)
        self.assertEqual(
            [
                "launch:camera",
                "ready:camera",
                "launch:detector",
                "ready:detector",
                "stop:detector",
                "stop:camera",
            ],
            self.operations,
        )

    def test_pause_and_resume_use_explicit_detector_commands(self):
        subject = self.make_subject()
        subject.start()

        pause_success, _ = subject.pause()
        self.assertTrue(pause_success)
        self.assertEqual(PAUSED, subject.state)
        resume_success, _ = subject.resume()

        self.assertTrue(resume_success)
        self.assertEqual(RECORDING, subject.state)
        self.assertEqual(
            ["detector:pause", "detector:resume"],
            self.operations[-2:],
        )

    def test_failed_pause_does_not_change_state(self):
        subject = self.make_subject()
        subject.start()
        self.detector_results["pause"] = (False, "暂停服务失败")

        success, message = subject.pause()

        self.assertFalse(success)
        self.assertEqual("暂停服务失败", message)
        self.assertEqual(RECORDING, subject.state)

    def test_stop_finalizes_detector_before_closing_processes(self):
        subject = self.make_subject()
        subject.start()

        success, output_dir = subject.stop()

        self.assertTrue(success)
        self.assertEqual("/tmp/recordings/20260723_153012", output_dir)
        self.assertEqual(IDLE, subject.state)
        self.assertEqual(
            ["detector:stop", "stop:detector", "stop:camera"],
            self.operations[-3:],
        )
        self.assertIsNone(subject.camera_process)
        self.assertIsNone(subject.detector_process)

    def test_stop_failure_still_closes_owned_processes_and_sets_error(self):
        subject = self.make_subject()
        subject.start()
        self.detector_results["stop"] = (False, "视频封装失败")

        success, message = subject.stop()

        self.assertFalse(success)
        self.assertEqual("视频封装失败", message)
        self.assertEqual(ERROR, subject.state)
        self.assertEqual(
            ["detector:stop", "stop:detector", "stop:camera"],
            self.operations[-3:],
        )

    def test_stop_service_timeout_still_closes_owned_processes(self):
        subject = self.make_subject()
        subject.start()
        self.detector_results["stop"] = (False, "YOLO录像服务调用超时")

        success, message = subject.stop()

        self.assertFalse(success)
        self.assertIn("超时", message)
        self.assertEqual(ERROR, subject.state)
        self.assertEqual(
            ["detector:stop", "stop:detector", "stop:camera"],
            self.operations[-3:],
        )

    def test_process_stop_exception_keeps_ownership_and_sets_error(self):
        subject = self.make_subject()
        subject.start()
        original_stop = subject.stop_process

        def failing_stop(process):
            if process.name == "detector":
                self.operations.append("stop-error:detector")
                raise RuntimeError("无法终止进程组")
            original_stop(process)

        subject.stop_process = failing_stop

        success, message = subject.stop()

        self.assertFalse(success)
        self.assertIn("进程清理失败", message)
        self.assertEqual(ERROR, subject.state)
        self.assertIsNotNone(subject.detector_process)
        self.assertIsNone(subject.camera_process)

    def test_shutdown_from_recording_uses_safe_stop_sequence(self):
        subject = self.make_subject()
        subject.start()

        subject.shutdown()

        self.assertEqual(IDLE, subject.state)
        self.assertEqual(
            ["detector:stop", "stop:detector", "stop:camera"],
            self.operations[-3:],
        )

    def test_shutdown_during_partial_start_only_stops_owned_processes(self):
        subject = self.make_subject()
        subject.camera_process = FakeProcess("camera")

        subject.shutdown()

        self.assertEqual(["stop:camera"], self.operations)
        self.assertEqual(IDLE, subject.state)

    def test_abnormal_exit_reports_process_and_cleans_remaining_process(self):
        subject = self.make_subject()
        subject.start()
        subject.detector_process.returncode = 7

        failures = subject.find_exited_processes()
        subject.cleanup_after_failure()

        self.assertEqual([("YOLO", 7)], failures)
        self.assertEqual(ERROR, subject.state)
        self.assertEqual(
            ["stop:detector", "stop:camera"],
            self.operations[-2:],
        )

    def test_invalid_operations_return_chinese_reasons(self):
        subject = self.make_subject()

        self.assertIn("无法暂停", subject.pause()[1])
        self.assertIn("无法继续", subject.resume()[1])
        self.assertIn("没有可结束", subject.stop()[1])

    def test_call_with_timeout_returns_without_waiting_forever(self):
        release = threading.Event()

        completed, value, error = call_with_timeout(
            lambda: release.wait(), 0.01
        )

        self.assertFalse(completed)
        self.assertIsNone(value)
        self.assertIn("超时", str(error))
        release.set()

    def test_shutdown_retries_process_cleanup_after_stop_failure(self):
        subject = self.make_subject()
        subject.start()
        attempts = {"detector": 0}
        original_stop = subject.stop_process

        def fail_once(process):
            if process.name == "detector":
                attempts["detector"] += 1
                if attempts["detector"] == 1:
                    raise RuntimeError("第一次清理失败")
            original_stop(process)

        subject.stop_process = fail_once

        success, message = subject.shutdown()

        self.assertFalse(success)
        self.assertIn("第一次清理失败", message)
        self.assertIsNone(subject.detector_process)
        self.assertIsNone(subject.camera_process)


if __name__ == "__main__":
    unittest.main()
