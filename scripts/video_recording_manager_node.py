#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS1录像进程管理节点。"""
# import sys
# import rospkg
import os
import shlex
import signal
import subprocess
import threading
import time
import xmlrpc.client

import rosgraph
import rosnode
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# package_path = rospkg.RosPack().get_path("teleop_joy_ctrl")
# module_parent = os.path.join(package_path, "scripts")
# if module_parent not in sys.path:
#     sys.path.insert(0, module_parent)

from teleop_joy_ctrl.video_process_manager import (
    ERROR,
    IDLE,
    PAUSED,
    RECORDING,
    VideoProcessManager,
    call_with_timeout,
)

class VideoRecordingManagerNode(object):
    def __init__(self):
        self.video = rospy.get_param("~video", {})
        self.camera_workspace = self.video.get(
            "camera_workspace", "/home/shansu/catkin_ws"
        )
        self.python_executable = self.video.get(
            "python_executable",
            "/home/shansu/miniconda3/envs/detection/bin/python",
        )
        self.image_topic = self.video.get(
            "image_topic", "/camera/image_raw"
        )
        self.camera_node_name = self.video.get(
            "camera_node_name", "/camera_stream_node"
        )
        self.detector_node_name = self.video.get(
            "detector_node_name", "/yolo_detector_node"
        )
        self.camera_ready_timeout = float(
            self.video.get("camera_ready_timeout", 10.0)
        )
        self.detector_ready_timeout = float(
            self.video.get("detector_ready_timeout", 30.0)
        )
        self.detector_exit_timeout = float(
            self.video.get("detector_exit_timeout", 12.0)
        )
        self.detector_headless = bool(
            self.video.get("detector_headless", False)
        )
        self.adopt_existing_detector = bool(
            self.video.get("adopt_existing_detector", True)
        )
        self.process_stop_timeout = float(
            self.video.get("process_stop_timeout", 5.0)
        )
        self.detector_service_timeout = float(
            self.video.get(
                "detector_service_timeout",
                self.video.get("detector_command_timeout", 10.0),
            )
        )
        self.cleanup_stale_recording_nodes = bool(
            self.video.get("cleanup_stale_recording_nodes", True)
        )
        self.stale_node_cleanup_timeout = float(
            self.video.get("stale_node_cleanup_timeout", 5.0)
        )
        self.force_kill_stale_recording_nodes = bool(
            self.video.get("force_kill_stale_recording_nodes", True)
        )
        self.force_kill_timeout = float(
            self.video.get("force_kill_timeout", 2.0)
        )
        self.cleanup_unreachable_stale_recording_nodes = bool(
            self.video.get(
                "cleanup_unreachable_stale_recording_nodes", True
            )
        )

        self.detector_services = {
            "pause": self.video.get(
                "detector_pause_service",
                "/yolo_detector_node/recording/pause",
            ),
            "resume": self.video.get(
                "detector_resume_service",
                "/yolo_detector_node/recording/resume",
            ),
            "stop": self.video.get(
                "detector_stop_service",
                "/yolo_detector_node/recording/stop",
            ),
            "shutdown": self.video.get(
                "detector_shutdown_service",
                "/yolo_detector_node/recording/shutdown",
            ),
        }
        self.detector_output_dir_topic = self.video.get(
            "detector_output_dir_topic",
            "/yolo_detector_node/recording/output_dir",
        )
        self.detector_state_topic = self.video.get(
            "detector_state_topic",
            "/yolo_detector_node/recording/state",
        )
        self.state_topic = self.video.get(
            "state_topic", "/teleop_joy_ctrl/video/state"
        )
        self.status_topic = self.video.get(
            "status_topic", "/teleop_joy_ctrl/video/status"
        )
        self.output_dir_topic = self.video.get(
            "output_dir_topic", "/teleop_joy_ctrl/video/output_dir"
        )
        self.public_services = {
            "start": self.video.get(
                "start_service", "/teleop_joy_ctrl/video/start"
            ),
            "pause": self.video.get(
                "pause_service", "/teleop_joy_ctrl/video/pause"
            ),
            "resume": self.video.get(
                "resume_service", "/teleop_joy_ctrl/video/resume"
            ),
            "stop": self.video.get(
                "stop_service", "/teleop_joy_ctrl/video/stop"
            ),
        }

        self.state_pub = rospy.Publisher(
            self.state_topic,
            String,
            queue_size=1,
            latch=True,
        )
        self.status_pub = rospy.Publisher(
            self.status_topic,
            String,
            queue_size=1,
            latch=True,
        )
        self.output_dir_pub = rospy.Publisher(
            self.output_dir_topic,
            String,
            queue_size=1,
            latch=True,
        )
        self.output_dir = ""
        self.output_dir_pub.publish(String(""))

        self.detector_clients = {
            command: rospy.ServiceProxy(service_name, Trigger)
            for command, service_name in self.detector_services.items()
        }
        self.core = VideoProcessManager(
            launch_process=self.launch_owned_process,
            wait_ready=self.wait_ready,
            detector_command=self.detector_command,
            stop_process=self.stop_owned_process,
            wait_process_exit=self.wait_for_detector_exit,
        )
        self.operation_lock = threading.Lock()
        self.detector_state_lock = threading.Lock()
        self.detector_state_event = threading.Event()
        self.detector_reported_state = IDLE
        self.shutting_down = False

        rospy.Subscriber(
            self.detector_output_dir_topic,
            String,
            self.detector_output_dir_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.detector_state_topic,
            String,
            self.detector_state_callback,
            queue_size=1,
        )
        self.start_service = rospy.Service(
            self.public_services["start"],
            Trigger,
            self.start_callback,
        )
        self.pause_service = rospy.Service(
            self.public_services["pause"],
            Trigger,
            self.pause_callback,
        )
        self.resume_service = rospy.Service(
            self.public_services["resume"],
            Trigger,
            self.resume_callback,
        )
        self.stop_service = rospy.Service(
            self.public_services["stop"],
            Trigger,
            self.stop_callback,
        )

        self.monitor_timer = rospy.Timer(
            rospy.Duration(1.0), self.monitor_processes
        )
        rospy.on_shutdown(self.shutdown)
        self.publish_core_state()
        self.publish_status(
            True, "录像进程管理节点已启动，当前处于空闲状态"
        )

    def start_callback(self, _request):
        return self.run_operation(
            self.start_with_conflict_check,
            "开始录像",
        )

    def pause_callback(self, _request):
        return self.run_operation(self.core.pause, "暂停录像")

    def resume_callback(self, _request):
        return self.run_operation(self.core.resume, "继续录像")

    def stop_callback(self, _request):
        return self.run_operation(self.core.stop, "结束录像")

    def run_operation(self, operation, operation_name):
        if self.shutting_down:
            return TriggerResponse(False, "录像管理节点正在退出")
        if not self.operation_lock.acquire(False):
            message = "上一条录像操作仍在执行，请稍候"
            self.publish_status(False, message)
            return TriggerResponse(False, message)

        try:
            success, message = operation()
            self.sync_detector_state_if_active()
            if success and operation_name == "结束录像":
                self.output_dir = str(message)
                self.output_dir_pub.publish(String(self.output_dir))
                status_message = "视频录制已结束，文件已保存，目录=%s" % message
            elif success:
                status_message = str(message)
            else:
                status_message = "%s失败：%s" % (operation_name, message)

            self.publish_core_state()
            self.publish_status(success, status_message)
            return TriggerResponse(bool(success), str(message))
        except Exception as exc:
            self.core.cleanup_after_failure()
            self.publish_core_state()
            message = "%s出现未处理异常：%s" % (operation_name, exc)
            self.publish_status(False, message)
            return TriggerResponse(False, message)
        finally:
            self.operation_lock.release()

    def start_with_conflict_check(self):
        if (
            self.core.state not in (IDLE, ERROR)
            or self.core.camera_process is not None
            or self.core.detector_process is not None
        ):
            return self.core.start()

        try:
            existing_nodes = set(rosnode.get_node_names())
        except Exception as exc:
            return False, "无法检查现有ROS节点：%s" % exc

        conflicts = existing_nodes.intersection(
            {self.camera_node_name, self.detector_node_name}
        )
        if conflicts:
            adopted, message = self.try_adopt_existing_detector(conflicts)
            if adopted:
                return True, message
            success, message = self.cleanup_stale_nodes(conflicts)
            if not success:
                return False, message
            rospy.sleep(0.2)
            try:
                existing_nodes = set(rosnode.get_node_names())
            except Exception as exc:
                return False, "遗留录像节点已关闭，但无法重新检查ROS节点：%s" % exc
            conflicts = existing_nodes.intersection(
                {self.camera_node_name, self.detector_node_name}
            )
            if not conflicts:
                rospy.loginfo(
                    "录像进程管理节点: [成功] %s",
                    message,
                )
            else:
                return (
                    False,
                    "遗留录像节点关闭后仍存在：%s"
                    % "、".join(sorted(conflicts)),
                )
        image_publishers = self.get_topic_publishers(self.image_topic)
        if image_publishers:
            return (
                False,
                "图像话题已有发布者，未启动第二个相机：%s"
                % "、".join(sorted(image_publishers)),
            )
        self.reset_detector_state()
        self.output_dir = ""
        self.output_dir_pub.publish(String(""))
        return self.core.start()

    def try_adopt_existing_detector(self, conflicts):
        """Attach only to a complete, actively recording manual YOLO session."""
        expected_nodes = {self.camera_node_name, self.detector_node_name}
        if not self.adopt_existing_detector or not expected_nodes.issubset(conflicts):
            return False, ""

        publishers = self.get_topic_publishers(self.image_topic)
        if self.camera_node_name not in publishers:
            return False, ""

        if not self.wait_for_existing_detector():
            return False, ""

        success, message = self.core.adopt_existing_detector()
        if success:
            rospy.loginfo("录像进程管理节点: [成功] %s", message)
        return success, message

    def reset_detector_state(self):
        """Discard a latched state from an earlier detector instance."""
        with self.detector_state_lock:
            self.detector_reported_state = IDLE
            self.detector_state_event.clear()

    def cleanup_stale_nodes(self, stale_nodes):
        stale_nodes = set(stale_nodes)
        if not self.cleanup_stale_recording_nodes:
            return (
                False,
                "检测到已有录像节点，自动回收已关闭：%s"
                % "、".join(sorted(stale_nodes)),
            )

        if self.detector_node_name in stale_nodes:
            try:
                completed, response, error = call_with_timeout(
                    self.detector_clients["shutdown"],
                    self.detector_service_timeout,
                )
                if not completed:
                    rospy.logwarn(
                        "录像进程管理节点: 遗留YOLO正常退出服务超时，继续请求节点退出"
                    )
                elif error is not None:
                    rospy.logwarn(
                        "录像进程管理节点: 遗留YOLO正常退出服务异常=%s，继续请求节点退出",
                        error,
                    )
                elif not response.success:
                    rospy.logwarn(
                        "录像进程管理节点: 遗留YOLO正常退出服务失败=%s，继续请求节点退出",
                        response.message,
                    )
            except Exception as exc:
                rospy.logwarn(
                    "录像进程管理节点: 遗留YOLO正常退出服务不可用=%s，继续请求节点退出",
                    exc,
                )

        try:
            rosnode.kill_nodes(sorted(stale_nodes))
        except Exception as exc:
            return False, "无法关闭遗留录像节点：%s" % exc

        remaining = self.wait_for_nodes_exit(
            stale_nodes, self.stale_node_cleanup_timeout
        )
        if not remaining:
            return (
                True,
                "已自动关闭遗留录像节点：%s"
                % "、".join(sorted(stale_nodes)),
            )

        if not self.force_kill_stale_recording_nodes:
            return self.stale_nodes_not_exited(remaining)

        self.force_stop_stale_nodes(remaining)
        remaining = self.wait_for_nodes_exit(
            remaining, self.force_kill_timeout
        )
        if not remaining:
            return (
                True,
                "已强制关闭卡住的遗留录像节点：%s"
                % "、".join(sorted(stale_nodes)),
            )
        return self.stale_nodes_not_exited(remaining)

    def wait_for_nodes_exit(self, node_names, timeout):
        deadline = time.monotonic() + max(0.0, float(timeout))
        remaining = set(node_names)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                remaining = remaining.intersection(
                    set(rosnode.get_node_names())
                )
            except Exception as exc:
                rospy.logwarn(
                    "录像进程管理节点: 关闭遗留录像节点后无法检查ROS节点=%s",
                    exc,
                )
                return remaining
            if not remaining:
                return set()
            rospy.sleep(0.1)
        return remaining

    @staticmethod
    def stale_nodes_not_exited(remaining):
        return (
            False,
            "遗留录像节点未在等待期限内退出：%s"
            % "、".join(sorted(remaining)),
        )

    def force_stop_stale_nodes(self, node_names):
        for node_name in sorted(node_names):
            try:
                pid = self.get_ros_node_pid(node_name)
            except Exception as exc:
                rospy.logerr(
                    "录像进程管理节点: 无法读取遗留节点PID 节点=%s 错误=%s",
                    node_name,
                    exc,
                )
                if self.cleanup_unreachable_stale_recording_nodes:
                    self.cleanup_unreachable_stale_nodes({node_name})
                continue
            if pid is None or pid == os.getpid():
                rospy.logerr(
                    "录像进程管理节点: 拒绝强制关闭无效或本节点PID 节点=%s PID=%s",
                    node_name,
                    pid,
                )
                continue
            try:
                os.kill(pid, signal.SIGTERM)
                rospy.logwarn(
                    "录像进程管理节点: 已向遗留录像节点发送SIGTERM 节点=%s PID=%s",
                    node_name,
                    pid,
                )
            except ProcessLookupError:
                continue
            except Exception as exc:
                rospy.logerr(
                    "录像进程管理节点: 发送SIGTERM失败 节点=%s PID=%s 错误=%s",
                    node_name,
                    pid,
                    exc,
                )
                continue

            if self.wait_for_nodes_exit({node_name}, self.force_kill_timeout):
                try:
                    os.kill(pid, signal.SIGKILL)
                    rospy.logerr(
                        "录像进程管理节点: 已向卡住的遗留录像节点发送SIGKILL 节点=%s PID=%s",
                        node_name,
                        pid,
                    )
                except ProcessLookupError:
                    pass
                except Exception as exc:
                    rospy.logerr(
                        "录像进程管理节点: 发送SIGKILL失败 节点=%s PID=%s 错误=%s",
                        node_name,
                        pid,
                        exc,
                    )

    @staticmethod
    def get_ros_node_pid(node_name):
        master = rosgraph.Master(rospy.get_name())
        api_uri = master.lookupNode(node_name)
        if not api_uri:
            return None
        proxy = xmlrpc.client.ServerProxy(api_uri)
        code, message, pid = proxy.getPid(rospy.get_name())
        if code != 1:
            raise RuntimeError(message)
        return int(pid)

    @staticmethod
    def cleanup_unreachable_stale_nodes(node_names):
        try:
            master = rosgraph.Master(rospy.get_name())
            rosnode.cleanup_master_blacklist(
                master, sorted(node_names)
            )
            rospy.logwarn(
                "录像进程管理节点: 已清理失联录像节点的ROS Master注册：%s",
                "、".join(sorted(node_names)),
            )
        except Exception as exc:
            rospy.logerr(
                "录像进程管理节点: 清理失联录像节点注册失败 节点=%s 错误=%s",
                "、".join(sorted(node_names)),
                exc,
            )

    @staticmethod
    def get_topic_publishers(topic_name):
        master = rosgraph.Master(rospy.get_name())
        publishers, _subscribers, _services = master.getSystemState()
        for topic, nodes in publishers:
            if topic == topic_name:
                return set(nodes)
        return set()

    def launch_owned_process(self, name):
        setup_file = os.path.join(
            self.camera_workspace, "devel", "setup.bash"
        )
        if not os.path.isdir(self.camera_workspace):
            raise RuntimeError(
                "相机工作空间不存在：%s" % self.camera_workspace
            )
        if not os.path.isfile(setup_file):
            raise RuntimeError(
                "相机工作空间环境文件不存在：%s" % setup_file
            )

        if name == "camera":
            command = (
                "source /opt/ros/noetic/setup.bash && "
                "source %s && "
                "exec rosrun mvs camera_stream"
            ) % shlex.quote(setup_file)
        elif name == "detector":
            if not os.path.isfile(self.python_executable):
                raise RuntimeError(
                    "检测环境Python不存在：%s" % self.python_executable
                )
            detector_script = os.path.join(
                self.camera_workspace,
                "src",
                "mvs",
                "scripts",
                "yolo_detector.py",
            )
            mvs_python_source = os.path.join(
                self.camera_workspace,
                "src",
                "mvs",
                "src",
            )
            if not os.path.isfile(detector_script):
                raise RuntimeError(
                    "YOLO检测脚本不存在：%s" % detector_script
                )
            if not os.path.isdir(mvs_python_source):
                raise RuntimeError(
                    "MVS Python模块目录不存在：%s" % mvs_python_source
                )
            detector_args = "--save-video"
            if self.detector_headless:
                detector_args += " --headless"
            command = (
                "source /opt/ros/noetic/setup.bash && "
                "source %s && "
                "export PYTHONPATH=%s:${PYTHONPATH} && "
                "exec %s %s %s"
            ) % (
                shlex.quote(setup_file),
                shlex.quote(mvs_python_source),
                shlex.quote(self.python_executable),
                shlex.quote(detector_script),
                detector_args,
            )
        else:
            raise ValueError("未知录像子进程类型：%s" % name)

        rospy.loginfo(
            "录像进程管理节点: 正在启动%s进程",
            "相机" if name == "camera" else "YOLO",
        )
        return subprocess.Popen(
            ["bash", "-lc", command],
            cwd=self.camera_workspace,
            start_new_session=True,
        )

    def wait_ready(self, name, process):
        if name == "camera":
            return self.wait_for_camera(process)
        return self.wait_for_detector(process)

    def wait_for_camera(self, process):
        deadline = time.monotonic() + self.camera_ready_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.core.cancel_requested.is_set():
                return False
            if process.poll() is not None:
                rospy.logerr(
                    "录像进程管理节点: 相机进程提前退出，退出码=%s",
                    process.returncode,
                )
                return False
            timeout = min(0.5, max(0.01, deadline - time.monotonic()))
            try:
                rospy.wait_for_message(
                    self.image_topic, Image, timeout=timeout
                )
                publishers = self.get_topic_publishers(self.image_topic)
                if self.camera_node_name not in publishers:
                    rospy.logwarn(
                        "录像进程管理节点: 图像已出现但发布者不是目标相机，发布者=%s",
                        "、".join(sorted(publishers)) or "未知",
                    )
                    continue
                rospy.loginfo(
                    "录像进程管理节点: [成功] 相机图像话题已就绪，话题=%s",
                    self.image_topic,
                )
                return True
            except rospy.ROSException:
                continue
        return False

    def wait_for_detector(self, process):
        deadline = time.monotonic() + self.detector_ready_timeout
        for command, service_name in self.detector_services.items():
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if self.core.cancel_requested.is_set():
                    return False
                camera_process = self.core.camera_process
                if (
                    camera_process is not None
                    and camera_process.poll() is not None
                ):
                    rospy.logerr(
                        "录像进程管理节点: 等待YOLO服务期间相机进程退出，退出码=%s",
                        camera_process.returncode,
                    )
                    return False
                if process.poll() is not None:
                    rospy.logerr(
                        "录像进程管理节点: YOLO进程提前退出，退出码=%s",
                        process.returncode,
                    )
                    return False
                timeout = min(
                    0.5, max(0.01, deadline - time.monotonic())
                )
                try:
                    rospy.wait_for_service(service_name, timeout=timeout)
                    break
                except rospy.ROSException:
                    continue
            else:
                rospy.logerr(
                    "录像进程管理节点: 等待YOLO私有服务超时，操作=%s 服务=%s",
                    command,
                    service_name,
                )
                return False

        rospy.loginfo(
            "录像进程管理节点: [成功] YOLO录像私有服务已全部就绪"
        )
        return self.wait_for_detector_recording_state(process, deadline)

    def wait_for_existing_detector(self):
        deadline = time.monotonic() + self.detector_ready_timeout
        for command, service_name in self.detector_services.items():
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                try:
                    rospy.wait_for_service(
                        service_name,
                        timeout=min(0.5, max(0.01, deadline - time.monotonic())),
                    )
                    break
                except rospy.ROSException:
                    continue
            else:
                return False
        return self.wait_for_detector_recording_state(None, deadline)

    def wait_for_detector_recording_state(self, process, deadline):
        """Wait on the persistent subscriber so a startup state is not missed."""
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.core.cancel_requested.is_set():
                return False
            if process is not None and process.poll() is not None:
                rospy.logerr(
                    "录像进程管理节点: 等待YOLO录像状态期间进程提前退出，退出码=%s",
                    process.returncode,
                )
                return False
            with self.detector_state_lock:
                state = self.detector_reported_state
            if state == RECORDING:
                rospy.loginfo(
                    "录像进程管理节点: [成功] YOLO录像状态已确认=%s",
                    state,
                )
                return True
            if state == ERROR:
                rospy.logerr(
                    "录像进程管理节点: YOLO录像状态为ERROR，未判定为启动成功"
                )
                return False
            timeout = min(0.5, max(0.01, deadline - time.monotonic()))
            self.detector_state_event.wait(timeout)
            self.detector_state_event.clear()

        rospy.logerr(
            "录像进程管理节点: 等待YOLO录像状态超时，话题=%s",
            self.detector_state_topic,
        )
        return False

    def detector_command(self, command):
        client = self.detector_clients.get(command)
        if client is None:
            return False, "未知YOLO录像操作：%s" % command
        completed, response, error = call_with_timeout(
            client, self.detector_service_timeout
        )
        if not completed:
            return (
                False,
                "YOLO录像服务调用超时，操作=%s，超时=%.1fs"
                % (command, self.detector_service_timeout),
            )
        if error is not None:
            return False, "YOLO录像服务调用失败：%s" % error
        return bool(response.success), str(response.message)

    def stop_owned_process(self, process):
        if process.poll() is not None:
            return

        try:
            process_group = os.getpgid(process.pid)
        except ProcessLookupError:
            return

        if self.signal_and_wait(
            process, process_group, signal.SIGINT, self.process_stop_timeout
        ):
            return
        rospy.logwarn(
            "录像进程管理节点: 子进程未响应SIGINT，升级为SIGTERM，PID=%s",
            process.pid,
        )
        if self.signal_and_wait(
            process, process_group, signal.SIGTERM, 1.0
        ):
            return
        rospy.logerr(
            "录像进程管理节点: 子进程未响应SIGTERM，升级为SIGKILL，PID=%s",
            process.pid,
        )
        if not self.signal_and_wait(
            process, process_group, signal.SIGKILL, 1.0
        ):
            raise RuntimeError(
                "SIGKILL后进程组仍未退出，PID=%s" % process.pid
            )

    def wait_for_detector_exit(self, process):
        deadline = time.monotonic() + self.detector_exit_timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                rospy.loginfo(
                    "录像进程管理节点: YOLO已通过正常退出服务结束，退出码=%s",
                    process.returncode,
                )
                return True
            time.sleep(0.1)
        rospy.logwarn(
            "录像进程管理节点: YOLO正常退出等待超时=%.1fs，转入信号清理",
            self.detector_exit_timeout,
        )
        return False

    @staticmethod
    def signal_and_wait(process, process_group, signal_number, timeout):
        try:
            os.killpg(process_group, signal_number)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            if (
                    process.poll() is not None
                    and not VideoRecordingManagerNode.process_group_exists(
                        process_group)):
                return True
            time.sleep(0.05)
        return (
            process.poll() is not None
            and not VideoRecordingManagerNode.process_group_exists(
                process_group))

    @staticmethod
    def process_group_exists(process_group):
        try:
            os.killpg(process_group, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def monitor_processes(self, _event):
        if self.shutting_down:
            return
        if self.core.state not in (RECORDING, PAUSED):
            return

        failures = self.core.find_exited_processes()
        if not failures or not self.operation_lock.acquire(False):
            return

        reason = "，".join(
            "%s退出码=%s" % (name, returncode)
            for name, returncode in failures
        )
        worker = threading.Thread(
            target=self.cleanup_after_process_failure,
            args=(reason,),
        )
        worker.daemon = True
        worker.start()

    def cleanup_after_process_failure(self, reason):
        try:
            self.core.cleanup_after_failure()
            self.publish_core_state()
            self.publish_status(
                False, "录像子进程异常退出：%s" % reason
            )
        finally:
            self.operation_lock.release()

    def detector_output_dir_callback(self, message):
        if self.core.state not in ("STARTING", RECORDING, PAUSED, "STOPPING"):
            return
        self.output_dir = str(message.data)
        self.output_dir_pub.publish(String(self.output_dir))

    def detector_state_callback(self, message):
        state = str(message.data).strip().upper()
        if state not in (RECORDING, PAUSED, ERROR):
            return
        with self.detector_state_lock:
            self.detector_reported_state = state
            self.detector_state_event.set()

        if self.shutting_down:
            return
        if state in (RECORDING, PAUSED):
            if not self.operation_lock.acquire(False):
                return
            try:
                self.sync_detector_state_if_active()
            finally:
                self.operation_lock.release()
            return
        if (
                state != ERROR
                or self.shutting_down
                or self.core.state not in (RECORDING, PAUSED)
                or not self.operation_lock.acquire(False)):
            return
        worker = threading.Thread(
            target=self.cleanup_after_process_failure,
            args=("YOLO录像状态=ERROR",),
        )
        worker.daemon = True
        worker.start()

    def sync_detector_state_if_active(self):
        with self.detector_state_lock:
            reported_state = self.detector_reported_state
        if (
                self.core.state in (RECORDING, PAUSED)
                and reported_state in (RECORDING, PAUSED)
                and reported_state != self.core.state):
            self.core.state = reported_state
            rospy.logwarn(
                "录像进程管理节点: 已根据YOLO状态回传同步录像状态=%s",
                reported_state,
            )

    def publish_core_state(self):
        self.state_pub.publish(String(self.core.state))

    def publish_status(self, success, message):
        prefix = "[成功]" if success else "[失败]"
        text = "%s %s" % (prefix, message)
        self.status_pub.publish(String(text))
        if success:
            rospy.loginfo("录像进程管理节点: %s", text)
        else:
            rospy.logerr("录像进程管理节点: %s", text)

    def shutdown(self):
        self.shutting_down = True
        self.core.request_cancel()
        try:
            self.monitor_timer.shutdown()
        except Exception:
            pass

        acquired = self.operation_lock.acquire(
            timeout=max(
                1.0,
                self.detector_service_timeout
                + self.process_stop_timeout
                + 3.0,
            )
        )
        if not acquired:
            self.publish_status(
                False,
                "录像操作在等待期限内未结束，未并发清理子进程",
            )
            return
        try:
            try:
                success, message = self.core.shutdown()
            except Exception as exc:
                success = False
                message = "退出清理异常：%s" % exc
                self.core.cleanup_after_failure()
            self.publish_core_state()
            self.publish_status(success, message)
        finally:
            self.operation_lock.release()


def main():
    rospy.init_node("video_recording_manager_node")
    VideoRecordingManagerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
