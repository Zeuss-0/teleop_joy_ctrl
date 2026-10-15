#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""录像子进程生命周期核心。

该模块不依赖 ROS。进程启动、就绪等待、检测节点控制和进程停止均由
调用方注入，便于单元测试并确保所有权边界清晰。
"""

import threading


IDLE = "IDLE"
STARTING = "STARTING"
RECORDING = "RECORDING"
PAUSED = "PAUSED"
STOPPING = "STOPPING"
ERROR = "ERROR"


def call_with_timeout(function, timeout):
    completed = threading.Event()
    result = {}

    def invoke():
        try:
            result["value"] = function()
        except Exception as exc:
            result["error"] = exc
        finally:
            completed.set()

    worker = threading.Thread(target=invoke)
    worker.daemon = True
    worker.start()
    if not completed.wait(float(timeout)):
        return False, None, RuntimeError("服务调用超时")
    if "error" in result:
        return False, None, result["error"]
    return True, result.get("value"), None


class VideoProcessManager(object):
    def __init__(
        self,
        launch_process,
        wait_ready,
        detector_command,
        stop_process,
        wait_process_exit=None,
    ):
        self.launch_process = launch_process
        self.wait_ready = wait_ready
        self.detector_command = detector_command
        self.stop_process = stop_process
        self.wait_process_exit = wait_process_exit
        self.camera_process = None
        self.detector_process = None
        self.state = IDLE
        self.output_dir = ""
        self.external_detector = False
        self.last_cleanup_errors = []
        self.cancel_requested = threading.Event()

    def start(self):
        if self.state not in (IDLE, ERROR):
            return False, "录像管理器当前不是空闲状态，无法开始录像"

        if self.camera_process is not None or self.detector_process is not None:
            return False, "录像管理器仍持有子进程，拒绝重复启动"

        self.state = STARTING
        self.cancel_requested.clear()
        self.output_dir = ""
        self.external_detector = False
        try:
            self.camera_process = self._launch("camera")
            if not self.wait_ready("camera", self.camera_process):
                raise RuntimeError("等待相机图像话题超时")
            self._raise_if_cancelled()

            self.detector_process = self._launch("detector")
            if not self.wait_ready("detector", self.detector_process):
                raise RuntimeError("等待YOLO录像私有服务超时")
            self._raise_if_cancelled()

            self.state = RECORDING
            return True, "视频录制已开始"
        except Exception as exc:
            cleanup_errors = self._stop_owned_processes()
            self.state = ERROR
            message = "视频录制启动失败：%s" % exc
            if cleanup_errors:
                message += "；进程清理失败：%s" % "；".join(cleanup_errors)
            return False, message

    def pause(self):
        if self.state != RECORDING:
            return False, "当前状态无法暂停录像"
        success, message = self._run_detector_command("pause")
        if success:
            self.state = PAUSED
        return success, message

    def adopt_existing_detector(self):
        """Attach to a manually started YOLO recorder without taking its PID."""
        if self.state not in (IDLE, ERROR):
            return False, "录像管理器当前不是空闲状态，无法接管已有YOLO"
        if self.camera_process is not None or self.detector_process is not None:
            return False, "录像管理器仍持有子进程，拒绝接管已有YOLO"
        self.cancel_requested.clear()
        self.external_detector = True
        self.state = RECORDING
        return True, "已接管手动启动的YOLO录像会话"

    def resume(self):
        if self.state != PAUSED:
            return False, "当前状态无法继续录像"
        success, message = self._run_detector_command("resume")
        if success:
            self.state = RECORDING
        return success, message

    def stop(self):
        if self.state not in (RECORDING, PAUSED):
            return False, "当前没有可结束的录像"

        self.state = STOPPING
        success = False
        message = "YOLO录像停止服务未返回结果"
        cleanup_errors = []
        try:
            success, message = self._run_detector_command("stop")
            if success:
                self.output_dir = str(message)
        finally:
            if not self.external_detector:
                self._shutdown_detector_gracefully()
            cleanup_errors = self._stop_owned_processes()

        if cleanup_errors:
            success = False
            message = "进程清理失败：%s" % "；".join(cleanup_errors)

        self.state = IDLE if success else ERROR
        self.external_detector = False
        return success, message

    def shutdown(self):
        self.cancel_requested.set()
        success = True
        messages = []
        if self.state in (RECORDING, PAUSED):
            stopped, message = self.stop()
            success = success and stopped
            messages.append(message)

        if self.camera_process is not None or self.detector_process is not None:
            cleanup_errors = self._stop_owned_processes()
            if cleanup_errors:
                success = False
                messages.extend(cleanup_errors)
        handles_cleared = (
            self.camera_process is None
            and self.detector_process is None
        )
        success = success and handles_cleared
        self.state = IDLE if success else ERROR
        if success:
            return True, "自有录像子进程清理完成"
        return False, "；".join(str(item) for item in messages if item)

    def request_cancel(self):
        self.cancel_requested.set()

    def find_exited_processes(self):
        failures = []
        for display_name, process in (
            ("相机", self.camera_process),
            ("YOLO", self.detector_process),
        ):
            if process is None:
                continue
            try:
                returncode = process.poll()
            except Exception as exc:
                failures.append((display_name, "状态检查失败:%s" % exc))
                continue
            if returncode is not None:
                failures.append((display_name, returncode))
        return failures

    def cleanup_after_failure(self):
        self._stop_owned_processes()
        self.external_detector = False
        self.state = ERROR

    def _launch(self, name):
        self._raise_if_cancelled()
        process = self.launch_process(name)
        if process is None:
            raise RuntimeError("%s进程启动器未返回进程句柄" % name)
        return process

    def _raise_if_cancelled(self):
        if self.cancel_requested.is_set():
            raise RuntimeError("录像进程启动已取消")

    def _run_detector_command(self, command):
        try:
            result = self.detector_command(command)
        except Exception as exc:
            return False, "YOLO录像%s指令调用异常：%s" % (
                self._command_name(command),
                exc,
            )

        if not isinstance(result, tuple) or len(result) != 2:
            return False, "YOLO录像%s指令返回格式无效" % self._command_name(command)
        return bool(result[0]), str(result[1])

    def _stop_owned_processes(self):
        # 先关闭检测进程，避免相机先退出导致检测节点在封装后继续等待输入。
        cleanup_errors = []
        for attribute in ("detector_process", "camera_process"):
            process = getattr(self, attribute)
            if process is None:
                continue
            try:
                self.stop_process(process)
            except Exception as exc:
                # 一个进程清理失败时仍必须继续处理另一个自有进程。
                cleanup_errors.append("%s：%s" % (attribute, exc))
            else:
                setattr(self, attribute, None)
        self.last_cleanup_errors = cleanup_errors
        return cleanup_errors

    def _shutdown_detector_gracefully(self):
        """Ask YOLO to exit before process-group signals are considered."""
        process = self.detector_process
        if process is None:
            return

        success, _message = self._run_detector_command("shutdown")
        if not success:
            return

        if self._wait_for_process_exit(process):
            self.detector_process = None

    def _wait_for_process_exit(self, process):
        if self.wait_process_exit is not None:
            try:
                return bool(self.wait_process_exit(process))
            except Exception:
                return False
        try:
            process.wait(timeout=10.0)
            return True
        except Exception:
            return False

    @staticmethod
    def _command_name(command):
        return {
            "pause": "暂停",
            "resume": "继续",
            "stop": "结束",
            "shutdown": "正常退出",
        }.get(command, command)
