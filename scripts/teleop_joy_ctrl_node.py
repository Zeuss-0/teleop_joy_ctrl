#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

# import sys,os,rospkg
import math
import queue
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String

# package_path = rospkg.RosPack().get_path("teleop_joy_ctrl")
# module_parent = os.path.join(package_path, "scripts")
# if module_parent not in sys.path:
#     sys.path.insert(0, module_parent)

MODE_INIT = 1
MODE_BRAKE_OPEN = 2
MODE_POWEROFF = 3
MODE_DISABLED = 4
MODE_ENABLE_IDLE = 5
MODE_BACKDRIVE = 6
MODE_RUNNING = 7
MODE_SINGLE_MOVE = 8
MODE_ERROR = 9
MODE_PAUSE = 10
MODE_COLLISION = 11

ACTIVE_MODE_NONE = 'none'
ACTIVE_MODE_CHASSIS = 'chassis'
ACTIVE_MODE_CAMERA = 'camera'
ACTIVE_MODE_LIGHT = 'light'

VIDEO_IDLE = 'IDLE'
VIDEO_STARTING = 'STARTING'
VIDEO_RECORDING = 'RECORDING'
VIDEO_PAUSED = 'PAUSED'
VIDEO_STOPPING = 'STOPPING'
VIDEO_ERROR = 'ERROR'

VIDEO_COMMAND_START = 'start'
VIDEO_COMMAND_PAUSE = 'pause'
VIDEO_COMMAND_RESUME = 'resume'
VIDEO_COMMAND_STOP = 'stop'


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


class R3GestureInterpreter(object):
    def __init__(self, double_click_interval, long_press_duration):
        self.double_click_interval = float(double_click_interval)
        self.long_press_duration = float(long_press_duration)
        self.was_pressed = False
        self.press_started_at = 0.0
        self.long_press_fired = False
        self.pending_idle_click_at = None
        self.double_click_fired_for_press = False

    def reset(self, pressed, now):
        self.was_pressed = bool(pressed)
        self.press_started_at = float(now)
        self.long_press_fired = False
        self.pending_idle_click_at = None
        self.double_click_fired_for_press = False

    def update(self, pressed, now, recording_state):
        pressed = bool(pressed)
        now = float(now)
        events = []
        active = recording_state in (VIDEO_RECORDING, VIDEO_PAUSED)
        idle_like = recording_state in (VIDEO_IDLE, VIDEO_ERROR)

        if (
                idle_like
                and self.pending_idle_click_at is not None
                and now - self.pending_idle_click_at > self.double_click_interval):
            self.pending_idle_click_at = None

        if pressed and not self.was_pressed:
            self.press_started_at = now
            self.long_press_fired = False
            self.double_click_fired_for_press = False
            if (
                    idle_like
                    and self.pending_idle_click_at is not None
                    and now - self.pending_idle_click_at
                    <= self.double_click_interval):
                self.pending_idle_click_at = None
                self.double_click_fired_for_press = True
                events.append(VIDEO_COMMAND_START)

        if (
                pressed
                and self.was_pressed
                and active
                and not self.long_press_fired
                and now - self.press_started_at >= self.long_press_duration):
            self.long_press_fired = True
            self.pending_idle_click_at = None
            events.append(VIDEO_COMMAND_STOP)

        if not pressed and self.was_pressed:
            held_for = now - self.press_started_at
            if (
                    active
                    and not self.long_press_fired
                    and held_for >= self.long_press_duration):
                self.long_press_fired = True
                events.append(VIDEO_COMMAND_STOP)
            elif self.long_press_fired:
                pass
            elif self.double_click_fired_for_press:
                pass
            elif recording_state == VIDEO_RECORDING:
                events.append(VIDEO_COMMAND_PAUSE)
            elif recording_state == VIDEO_PAUSED:
                events.append(VIDEO_COMMAND_RESUME)
            elif idle_like and held_for < self.long_press_duration:
                self.pending_idle_click_at = now

        self.was_pressed = pressed
        return events


class TeleopJoyCtrl(object):
    def __init__(self):
        self.joy_topic = rospy.get_param('~joy_topic', '/joy')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.state_topic = rospy.get_param('~state_topic', '/teleop_joy_ctrl/state')
        self.debug_topic = rospy.get_param('~debug_topic', '/teleop_joy_ctrl/debug')

        self.publish_rate = float(rospy.get_param('~publish_rate', 20.0))
        self.joy_timeout = float(rospy.get_param('~joy_timeout', 0.5))
        self.axis_deadzone = float(rospy.get_param('~axis_deadzone', 0.08))
        self.adjust_repeat_interval = float(rospy.get_param('~adjust_repeat_interval', 0.25))
        self.left_trigger_pressed_threshold = clamp(
            float(rospy.get_param('~left_trigger_pressed_threshold', 0.95)),
            -1.0,
            1.0)
        self.debug_success_logs = bool(rospy.get_param('~debug_success_logs', True))
        self.chassis_debug_interval = float(rospy.get_param('~chassis_debug_interval', 1.0))

        self.axes = rospy.get_param('~axes', {})
        self.buttons = rospy.get_param('~buttons', {})
        self.chassis = rospy.get_param('~chassis', {})
        self.camera = rospy.get_param('~camera', {})
        self.arm = rospy.get_param('~arm', {})
        self.light = rospy.get_param('~light', {})
        self.video = rospy.get_param('~video', {})

        self.active_mode = ACTIVE_MODE_NONE
        self.locked = False
        self.last_joy = None
        self.last_buttons = None
        self.last_adjust_time = rospy.Time(0)
        self.last_joy_time = rospy.Time(0)
        self.last_chassis_debug_time = rospy.Time(0)
        self.chassis_command_active = False
        self.light_on = bool(self.light.get('assume_initial_on', False))
        self.video_enabled = bool(self.video.get('enabled', True))
        self.video_state = VIDEO_IDLE
        self.video_input_state = VIDEO_IDLE
        self.video_gesture = R3GestureInterpreter(
            self.video.get('double_click_interval', 0.4),
            self.video.get('long_press_duration', 1.5))
        self.video_command_queue = queue.Queue(maxsize=8)
        self.video_worker_stop = threading.Event()
        self.video_operation_lock = threading.Lock()
        self.video_operation_pending = False
        self.video_worker = None
        self.video_clients = {}
        self.video_service_names = {}
        self.video_service_type = None
        self.video_call_with_timeout = None
        self.video_command_timeout = float(
            self.video.get('manager_command_timeout', 35.0))

        self.max_linear_speed = float(self.chassis.get('max_linear_speed', 0.4))
        self.max_angular_speed = float(self.chassis.get('max_angular_speed', 0.8))
        self.exposure = float(self.camera.get('exposure_default', 5000.0))
        self.gain = float(self.camera.get('gain_default', 0.0))
        self.brightness = float(self.light.get('brightness_default', 0.5))

        self.arm_config_enabled = bool(self.arm.get('enabled', True))
        self.arm_clients_ready = False
        self.enable_client = None
        self.disable_client = None
        self.clear_error_client = None
        self.robot_mode_client = None

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=5, latch=True)
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=20)
        self.exposure_pub = rospy.Publisher(
            self.camera.get('exposure_topic', '/camera/exposure_cmd'), Float32, queue_size=5)
        self.gain_pub = rospy.Publisher(
            self.camera.get('gain_topic', '/camera/gain_cmd'), Float32, queue_size=5)
        self.light_power_pub = rospy.Publisher(
            self.light.get('power_topic', '/light/power_cmd'), Bool, queue_size=5, latch=True)
        self.light_brightness_pub = rospy.Publisher(
            self.light.get('brightness_topic', '/light/brightness_cmd'), Float32, queue_size=5, latch=True)

        rospy.Subscriber(
            self.camera.get('exposure_state_topic', '/camera/exposure_state'),
            Float32,
            self.exposure_state_callback,
            queue_size=1)
        rospy.Subscriber(
            self.camera.get('gain_state_topic', '/camera/gain_state'),
            Float32,
            self.gain_state_callback,
            queue_size=1)
        rospy.Subscriber(
            self.video.get(
                'state_topic', '/teleop_joy_ctrl/video/state'),
            String,
            self.video_state_callback,
            queue_size=1)
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback, queue_size=10)

        self.init_video_clients()
        self.init_arm_clients()
        self.publish_state('节点已启动')

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(period), self.publish_loop)

        rospy.loginfo('手柄总控节点: 手柄话题=%s 底盘速度话题=%s 状态话题=%s',
                      self.joy_topic, self.cmd_vel_topic, self.state_topic)

    def init_video_clients(self):
        if not self.video_enabled:
            rospy.loginfo('手柄总控节点: 视频录制控制已通过参数关闭')
            return

        try:
            from std_srvs.srv import Trigger
            from teleop_joy_ctrl.video_process_manager import call_with_timeout
        except ImportError as exc:
            self.video_enabled = False
            self.video_state = VIDEO_ERROR
            self.video_input_state = VIDEO_ERROR
            rospy.logerr(
                '手柄总控节点: 无法导入std_srvs/Trigger，视频录制控制不可用，错误=%s',
                exc)
            return

        service_names = {
            VIDEO_COMMAND_START: self.video.get(
                'start_service', '/teleop_joy_ctrl/video/start'),
            VIDEO_COMMAND_PAUSE: self.video.get(
                'pause_service', '/teleop_joy_ctrl/video/pause'),
            VIDEO_COMMAND_RESUME: self.video.get(
                'resume_service', '/teleop_joy_ctrl/video/resume'),
            VIDEO_COMMAND_STOP: self.video.get(
                'stop_service', '/teleop_joy_ctrl/video/stop'),
        }
        self.video_service_names = service_names
        self.video_service_type = Trigger
        self.video_call_with_timeout = call_with_timeout
        self.rebuild_video_clients()
        self.video_worker = threading.Thread(
            target=self.video_command_worker,
            name='teleop_video_command_worker')
        self.video_worker.daemon = True
        self.video_worker.start()
        rospy.on_shutdown(self.stop_video_worker)

    def rebuild_video_clients(self):
        if self.video_service_type is None:
            return
        self.video_clients = {
            command: rospy.ServiceProxy(name, self.video_service_type)
            for command, name in self.video_service_names.items()
        }

    def init_arm_clients(self):
        if not self.arm_config_enabled:
            rospy.loginfo('手柄总控节点: 机械臂控制已通过参数关闭')
            return False

        try:
            from dobot_v4_bringup.srv import ClearError, DisableRobot, EnableRobot, RobotMode
        except ImportError as exc:
            self.arm_clients_ready = False
            rospy.logerr(
                '手柄总控节点: 无法导入 dobot_v4_bringup 服务定义，请确认已 source 机械臂工作空间，错误=%s',
                exc)
            return False

        prefix = self.arm.get('service_prefix', '/dobot_v4_bringup').rstrip('/')
        robot_namespace = self.arm.get('robot_namespace', '').strip('/')
        if robot_namespace:
            prefix = '%s/%s' % (prefix, robot_namespace)
        srv_prefix = '%s/srv' % prefix

        service_names = {
            'enable': srv_prefix + '/EnableRobot',
            'disable': srv_prefix + '/DisableRobot',
            'clear_error': srv_prefix + '/ClearError',
            'robot_mode': srv_prefix + '/RobotMode',
        }

        rospy.loginfo('手柄总控节点: 等待 dobot_v4_bringup 机械臂服务...')
        for name in service_names.values():
            rospy.wait_for_service(name)

        self.enable_client = rospy.ServiceProxy(service_names['enable'], EnableRobot)
        self.disable_client = rospy.ServiceProxy(service_names['disable'], DisableRobot)
        self.clear_error_client = rospy.ServiceProxy(service_names['clear_error'], ClearError)
        self.robot_mode_client = rospy.ServiceProxy(service_names['robot_mode'], RobotMode)
        self.arm_clients_ready = True
        rospy.loginfo('手柄总控节点: 机械臂服务已就绪，服务前缀=%s', srv_prefix)
        return True

    def joy_callback(self, msg):
        now = rospy.Time.now()
        self.last_joy = msg
        self.last_joy_time = now

        if self.last_buttons is None:
            self.last_buttons = list(msg.buttons)
            self.video_gesture.reset(
                self.button_held(msg, 'right_stick'),
                now.to_sec())
            return

        self.handle_video_button(msg, now.to_sec())

        if self.rising_button(msg, 'x'):
            self.locked = not self.locked
            self.active_mode = ACTIVE_MODE_NONE
            self.publish_state('锁定=%s' % self.cn_bool(self.locked))

        if self.rising_button(msg, 'start'):
            start_index = int(self.buttons.get('start', -1))
            self.log_success('START按键已识别 index=%d locked=%s arm_ready=%s' % (
                start_index,
                self.cn_bool(self.locked),
                self.cn_bool(self.arm_clients_ready)))
            self.handle_arm_start()

        if not self.locked:
            if self.rising_button(msg, 'a'):
                self.active_mode = ACTIVE_MODE_CHASSIS
                self.publish_state('模式=底盘参数')
            if self.rising_button(msg, 'b'):
                self.active_mode = ACTIVE_MODE_CAMERA
                self.publish_state('模式=相机参数')
            if self.rising_button(msg, 'y'):
                self.active_mode = ACTIVE_MODE_LIGHT
                self.publish_state('模式=光源亮度')
            if self.rising_button(msg, 'home'):
                self.log_success('HOME按键触发，准备切换光源开关')
                self.toggle_light()

            self.handle_adjustments(msg, now)

        self.last_buttons = list(msg.buttons)

    def handle_video_button(self, msg, now_sec):
        if not self.video_enabled:
            return

        index = int(self.buttons.get('right_stick', -1))
        pressed = (
            0 <= index < len(msg.buttons)
            and msg.buttons[index] == 1)
        commands = self.video_gesture.update(
            pressed, now_sec, self.video_input_state)
        for command in commands:
            with self.video_operation_lock:
                if self.video_operation_pending:
                    rospy.logwarn(
                        '手柄总控节点: 录像操作仍在执行，已忽略R3操作=%s',
                        self.cn_video_command(command))
                    continue
                self.video_operation_pending = True
            try:
                self.video_command_queue.put_nowait(command)
            except queue.Full:
                with self.video_operation_lock:
                    self.video_operation_pending = False
                rospy.logwarn(
                    '手柄总控节点: 录像指令队列已满，已忽略操作=%s',
                    self.cn_video_command(command))
                continue

            if command == VIDEO_COMMAND_START:
                self.video_input_state = VIDEO_STARTING
            elif command == VIDEO_COMMAND_PAUSE:
                self.video_input_state = VIDEO_PAUSED
            elif command == VIDEO_COMMAND_RESUME:
                self.video_input_state = VIDEO_RECORDING
            elif command == VIDEO_COMMAND_STOP:
                self.video_input_state = VIDEO_STOPPING
            self.log_success(
                'R3录像指令已识别，操作=%s'
                % self.cn_video_command(command))

    def video_state_callback(self, msg):
        state = str(msg.data).strip().upper()
        valid_states = (
            VIDEO_IDLE,
            VIDEO_STARTING,
            VIDEO_RECORDING,
            VIDEO_PAUSED,
            VIDEO_STOPPING,
            VIDEO_ERROR,
        )
        if state not in valid_states:
            rospy.logwarn(
                '手柄总控节点: 收到未知录像状态=%s', state)
            return
        if state != self.video_state:
            self.video_state = state
            self.video_input_state = state
            self.publish_state(
                '录像状态更新=%s' % self.cn_video_state(state))

    def video_command_worker(self):
        while (
                not rospy.is_shutdown()
                and not self.video_worker_stop.is_set()):
            try:
                command = self.video_command_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                client = self.video_clients.get(command)
                if client is None:
                    raise RuntimeError('录像服务客户端不存在')
                completed, response, error = self.video_call_with_timeout(
                    client,
                    self.video_command_timeout,
                )
                if not completed:
                    self.video_input_state = self.video_state
                    self.rebuild_video_clients()
                    rospy.logerr(
                        '手柄总控节点: 录像服务调用超时=%s 超时=%.1fs',
                        self.cn_video_command(command),
                        self.video_command_timeout)
                    self.publish_state('录像服务调用超时')
                    continue
                if error is not None:
                    raise error
                if response.success:
                    self.log_success(str(response.message))
                else:
                    self.video_input_state = self.video_state
                    rospy.logerr(
                        '手柄总控节点: 录像操作失败=%s 原因=%s',
                        self.cn_video_command(command),
                        response.message)
            except Exception as exc:
                self.video_input_state = self.video_state
                rospy.logerr(
                    '手柄总控节点: 录像服务调用异常=%s 错误=%s',
                    self.cn_video_command(command),
                    exc)
                self.publish_state('录像服务调用异常')
            finally:
                with self.video_operation_lock:
                    self.video_operation_pending = False
                self.video_command_queue.task_done()

    def stop_video_worker(self):
        self.video_worker_stop.set()

    def publish_loop(self, _event):
        joy_is_fresh = self.last_joy is not None and not self.joy_timed_out()
        control_requested = (
            joy_is_fresh and self.chassis_control_requested(self.last_joy))

        if control_requested:
            twist = self.make_chassis_twist(self.last_joy)
            self.cmd_vel_pub.publish(twist)
            self.chassis_command_active = True
            self.maybe_log_chassis_success(twist)
            return

        if self.chassis_command_active:
            self.cmd_vel_pub.publish(Twist())
            self.chassis_command_active = False
            self.log_success(
                '底盘手柄控制已释放，已发送一次零速度并停止发布，等待导航接管')

    def maybe_log_chassis_success(self, twist):
        if not self.debug_success_logs:
            return
        if self.is_zero_twist(twist):
            return
        now = rospy.Time.now()
        if (now - self.last_chassis_debug_time).to_sec() < self.chassis_debug_interval:
            return
        self.last_chassis_debug_time = now
        self.log_success(
            '底盘速度指令 话题=%s 线速度=(%.3f,%.3f,%.3f) 角速度=(%.3f,%.3f,%.3f)'
            % (
                self.cmd_vel_topic,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )

    def is_zero_twist(self, twist):
        return (
            abs(twist.linear.x) < 1e-6
            and abs(twist.linear.y) < 1e-6
            and abs(twist.linear.z) < 1e-6
            and abs(twist.angular.x) < 1e-6
            and abs(twist.angular.y) < 1e-6
            and abs(twist.angular.z) < 1e-6
        )

    def joy_timed_out(self):
        if self.last_joy_time == rospy.Time(0):
            return True
        return (rospy.Time.now() - self.last_joy_time).to_sec() > self.joy_timeout

    def make_chassis_twist(self, msg):
        twist = Twist()
        low_held = self.button_held(msg, 'left_shoulder')
        high_held = self.button_held(msg, 'right_shoulder')
        if not low_held and not high_held:
            return twist

        speed_scale = float(self.chassis.get('high_speed_scale', 1.0))
        if low_held and not high_held:
            speed_scale = float(self.chassis.get('low_speed_scale', 0.35))

        left_y = self.apply_deadzone(self.axis_value(msg, 'left_y'))
        left_x = self.apply_deadzone(self.axis_value(msg, 'left_x'))

        twist.linear.x = left_y * self.max_linear_speed * speed_scale
        if bool(self.chassis.get('holonomic', False)):
            twist.linear.y = left_x * self.max_linear_speed * speed_scale
        else:
            twist.angular.z = left_x * self.max_angular_speed * speed_scale
        return twist

    def chassis_control_requested(self, msg):
        return (
            self.button_held(msg, 'left_shoulder') or
            self.button_held(msg, 'right_shoulder'))

    def handle_adjustments(self, msg, now):
        if not self.left_trigger_held(msg):
            return
        if (now - self.last_adjust_time).to_sec() < self.adjust_repeat_interval:
            return

        dpad_x = -self.dpad_direction(self.axis_value(msg, 'dpad_x'))
        dpad_y = self.dpad_direction(self.axis_value(msg, 'dpad_y'))
        if dpad_x == 0 and dpad_y == 0:
            return

        if self.active_mode == ACTIVE_MODE_CHASSIS:
            self.adjust_chassis_limits(dpad_x, dpad_y)
        elif self.active_mode == ACTIVE_MODE_CAMERA:
            self.adjust_camera(dpad_x, dpad_y)
        elif self.active_mode == ACTIVE_MODE_LIGHT:
            self.adjust_light(dpad_y)

        self.last_adjust_time = now

    def adjust_chassis_limits(self, dpad_x, dpad_y):
        if dpad_y != 0:
            step = max(float(self.chassis.get('linear_speed_step_min', 0.02)),
                       abs(self.max_linear_speed) * float(self.chassis.get('linear_speed_step_ratio', 0.10)))
            self.max_linear_speed = clamp(
                self.max_linear_speed + dpad_y * step,
                float(self.chassis.get('min_linear_speed', 0.05)),
                float(self.chassis.get('max_linear_speed_limit', 1.5)))
            self.publish_state('底盘线速度上限=%.3f' % self.max_linear_speed)

        if dpad_x != 0:
            step = max(float(self.chassis.get('angular_speed_step_min', 0.05)),
                       abs(self.max_angular_speed) * float(self.chassis.get('angular_speed_step_ratio', 0.10)))
            self.max_angular_speed = clamp(
                self.max_angular_speed + dpad_x * step,
                float(self.chassis.get('min_angular_speed', 0.05)),
                float(self.chassis.get('max_angular_speed_limit', 2.5)))
            self.publish_state('底盘角速度上限=%.3f' % self.max_angular_speed)

    def adjust_camera(self, dpad_x, dpad_y):
        if dpad_y != 0:
            step = max(float(self.camera.get('gain_step_min', 0.5)),
                       abs(self.gain) * float(self.camera.get('gain_step_ratio', 0.10)))
            self.gain = clamp(
                self.gain + dpad_y * step,
                float(self.camera.get('gain_min', 0.0)),
                float(self.camera.get('gain_max', 24.0)))
            self.gain_pub.publish(Float32(self.gain))
            self.log_success('相机增益指令 话题=%s 数值=%.3f' %
                             (self.camera.get('gain_topic', '/camera/gain_cmd'), self.gain))
            self.publish_state('相机增益=%.3f' % self.gain)

        if dpad_x != 0:
            step = max(float(self.camera.get('exposure_step_min', 100.0)),
                       abs(self.exposure) * float(self.camera.get('exposure_step_ratio', 0.12)))
            self.exposure = clamp(
                self.exposure + dpad_x * step,
                float(self.camera.get('exposure_min', 20.0)),
                float(self.camera.get('exposure_max', 50000.0)))
            self.exposure_pub.publish(Float32(self.exposure))
            self.log_success('相机曝光指令 话题=%s 数值=%.1f' %
                             (self.camera.get('exposure_topic', '/camera/exposure_cmd'), self.exposure))
            self.publish_state('相机曝光=%.1f' % self.exposure)

    def adjust_light(self, dpad_y):
        if dpad_y == 0:
            return
        was_light_on = self.light_on
        was_off_by_brightness = self.is_light_off_by_brightness()
        self.brightness = clamp(
            self.brightness + dpad_y * float(self.light.get('brightness_step', 0.05)),
            float(self.light.get('brightness_min', 0.0)),
            float(self.light.get('brightness_max', 1.0)))
        self.light_on = not self.is_light_off_by_brightness()
        if self.light_on and (was_off_by_brightness or not was_light_on):
            self.publish_light_power()
            rospy.sleep(float(self.light.get('power_brightness_delay', 0.05)))
            self.publish_light_brightness()
        elif not self.light_on and was_light_on:
            self.publish_light_brightness()
            rospy.sleep(float(self.light.get('power_brightness_delay', 0.05)))
            self.publish_light_power()
        else:
            self.publish_light_brightness()
        self.publish_state('光源亮度=%.2f' % self.brightness)

    def toggle_light(self):
        if not bool(self.light.get('enabled', True)):
            rospy.logwarn('手柄总控节点: 光源控制已通过参数关闭')
            return

        if self.is_light_off_by_brightness():
            self.brightness = clamp(
                float(self.light.get('home_restore_brightness',
                                     self.light.get('brightness_default', 0.5))),
                float(self.light.get('brightness_min', 0.0)),
                float(self.light.get('brightness_max', 1.0)))
            self.light_on = True
            self.publish_light_power()
            rospy.sleep(float(self.light.get('power_brightness_delay', 0.05)))
            self.publish_light_brightness()
            self.publish_state('HOME恢复光源固定亮度=%.2f' % self.brightness)
            return

        self.brightness = float(self.light.get('brightness_min', 0.0))
        self.publish_light_brightness()
        rospy.sleep(float(self.light.get('power_brightness_delay', 0.05)))
        self.light_on = False
        self.publish_light_power()
        self.publish_state('HOME关闭光源，亮度=0')

    def publish_light_power(self):
        if not bool(self.light.get('enabled', True)):
            return
        repeat = max(1, int(self.light.get('power_publish_repeat', 1)))
        interval = max(0.0, float(self.light.get('power_publish_repeat_interval', 0.05)))
        for index in range(repeat):
            self.light_power_pub.publish(Bool(self.light_on))
            if index + 1 < repeat and interval > 0.0:
                rospy.sleep(interval)
        self.log_success('光源开关指令 话题=%s 状态=%s' %
                         (
                             self.light.get('power_topic', '/light/power_cmd'),
                             '开启' if self.light_on else '关闭',
                         ))

    def publish_light_brightness(self):
        if not bool(self.light.get('enabled', True)):
            return
        self.light_brightness_pub.publish(Float32(self.brightness))
        self.log_success('光源亮度指令 话题=%s 亮度=%.3f 光源状态=%s' %
                         (
                             self.light.get('brightness_topic', '/light/brightness_cmd'),
                             self.brightness,
                             '开启' if self.light_on else '关闭',
                         ))

    def is_light_off_by_brightness(self):
        epsilon = float(self.light.get('off_brightness_epsilon', 0.001))
        return self.brightness <= float(self.light.get('brightness_min', 0.0)) + epsilon

    def ensure_arm_ready(self, action_name):
        if not self.arm_config_enabled:
            rospy.logwarn('手柄总控节点: %s 已忽略，因为配置中已关闭机械臂控制', action_name)
            return False

        if not self.arm_clients_ready:
            rospy.logwarn('手柄总控节点: 机械臂服务未连接，正在重新连接')
            if not self.init_arm_clients():
                rospy.logwarn(
                    '手柄总控节点: %s 暂不能控制机械臂，请检查 Dobot bringup 是否已启动、当前终端是否已 source 机械臂工作空间',
                    action_name)
                self.publish_state('机械臂服务未连接')
                return False

        if (
            self.enable_client is None
            or self.disable_client is None
            or self.clear_error_client is None
            or self.robot_mode_client is None
        ):
            self.arm_clients_ready = False
            rospy.logwarn('手柄总控节点: 机械臂服务客户端为空，%s 下次会重新连接', action_name)
            return False

        return True

    def handle_arm_start(self):
        rospy.loginfo('手柄总控节点: START进入机械臂状态机处理 locked=%s arm_ready=%s' % (
            self.cn_bool(self.locked),
            self.cn_bool(self.arm_clients_ready)))
        if not self.ensure_arm_ready('START机械臂状态机'):
            return

        mode = self.query_robot_mode()
        if mode in (MODE_ERROR, MODE_COLLISION):
            rospy.logwarn('手柄总控节点: START检测到机械臂报错/碰撞状态(mode=%d)，执行清错', mode)
            self.call_clear_error()
            return

        if mode in (MODE_ENABLE_IDLE, MODE_RUNNING, MODE_SINGLE_MOVE):
            rospy.loginfo('手柄总控节点: START -> 下使能机械臂，当前模式=%d', mode)
            self.call_disable_robot()
            return

        rospy.loginfo('手柄总控节点: START -> 上使能机械臂，当前模式=%d', mode)
        self.call_enable_robot()
    def query_robot_mode(self):
        try:
            resp = self.robot_mode_client()
        except rospy.ServiceException as exc:
            rospy.logwarn('手柄总控节点: RobotMode服务调用失败: %s', exc)
            return -1

        mode = getattr(resp, 'res', 0)
        return int(mode)

    def call_clear_error(self):
        try:
            self.clear_error_client()
        except rospy.ServiceException as exc:
            rospy.logerr('手柄总控节点: ClearError服务调用失败: %s', exc)
            return
        rospy.sleep(float(self.arm.get('command_settle_time', 0.3)))
        mode = self.query_robot_mode()
        if mode == MODE_ERROR:
            rospy.logwarn('手柄总控节点: ClearError已发送，但机械臂仍处于ERROR(9)')
        else:
            self.log_success('机械臂清错指令 服务=ClearError 执行后模式=%d' % mode)
            self.publish_state('机械臂模式=%d 清错后' % mode)

    def call_enable_robot(self):
        try:
            self.enable_client()
        except rospy.ServiceException as exc:
            rospy.logerr('手柄总控节点: EnableRobot服务调用失败: %s', exc)
            return
        rospy.sleep(float(self.arm.get('command_settle_time', 0.3)))
        mode = self.query_robot_mode()
        if mode in (MODE_ENABLE_IDLE, MODE_RUNNING, MODE_SINGLE_MOVE):
            self.log_success('机械臂上使能指令 服务=EnableRobot 执行后模式=%d' % mode)
            self.publish_state('机械臂模式=%d 上使能后' % mode)
        else:
            rospy.logwarn('手柄总控节点: EnableRobot已发送，但机械臂当前模式=%d，未确认进入使能态', mode)
            self.publish_state('机械臂上使能后模式=%d' % mode)

    def call_disable_robot(self):
        try:
            self.disable_client()
        except rospy.ServiceException as exc:
            rospy.logerr('手柄总控节点: DisableRobot服务调用失败: %s', exc)
            return
        rospy.sleep(float(self.arm.get('command_settle_time', 0.3)))
        mode = self.query_robot_mode()
        if mode == MODE_DISABLED:
            self.log_success('机械臂下使能指令 服务=DisableRobot 执行后模式=%d' % mode)
            self.publish_state('机械臂模式=%d 下使能后' % mode)
        else:
            rospy.logwarn('手柄总控节点: DisableRobot已发送，但机械臂当前模式=%d，可能被控制器拒绝', mode)
            self.publish_state('机械臂下使能后模式=%d' % mode)

    def exposure_state_callback(self, msg):
        self.exposure = clamp(
            float(msg.data),
            float(self.camera.get('exposure_min', 20.0)),
            float(self.camera.get('exposure_max', 50000.0)))

    def gain_state_callback(self, msg):
        self.gain = clamp(
            float(msg.data),
            float(self.camera.get('gain_min', 0.0)),
            float(self.camera.get('gain_max', 24.0)))

    def rising_button(self, msg, name):
        index = int(self.buttons.get(name, -1))
        if index < 0 or index >= len(msg.buttons) or index >= len(self.last_buttons):
            return False
        return msg.buttons[index] == 1 and self.last_buttons[index] == 0

    def button_held(self, msg, name):
        index = int(self.buttons.get(name, -1))
        if index < 0 or index >= len(msg.buttons):
            return False
        return msg.buttons[index] == 1

    def axis_value(self, msg, name):
        index = int(self.axes.get(name, -1))
        if index < 0 or index >= len(msg.axes):
            return 0.0
        value = float(msg.axes[index])
        if not math.isfinite(value):
            return 0.0
        return value

    def left_trigger_held(self, msg):
        value = self.axis_value(msg, 'left_trigger')
        return value < self.left_trigger_pressed_threshold

    def dpad_direction(self, value):
        if abs(value) < 0.5:
            return 0
        return int(sign(value))

    def apply_deadzone(self, value):
        if abs(value) < self.axis_deadzone:
            return 0.0
        return value

    def publish_state(self, text):
        state = (
            '事件=%s 锁定=%s 当前模式=%s 线速度上限=%.3f 角速度上限=%.3f '
            '曝光=%.1f 增益=%.3f 光源=%s 亮度=%.2f 录像=%s'
            % (
                text,
                self.cn_bool(self.locked),
                self.cn_mode(self.active_mode),
                self.max_linear_speed,
                self.max_angular_speed,
                self.exposure,
                self.gain,
                '开启' if self.light_on else '关闭',
                self.brightness,
                self.cn_video_state(self.video_state),
            )
        )
        self.state_pub.publish(String(state))
        rospy.loginfo('手柄总控节点: %s', state)

    def log_success(self, text):
        if not self.debug_success_logs:
            return
        msg = '[成功] %s' % text
        self.debug_pub.publish(String(msg))
        rospy.loginfo('手柄总控节点: %s', msg)

    def cn_bool(self, value):
        return '是' if value else '否'

    def cn_mode(self, mode):
        names = {
            ACTIVE_MODE_NONE: '未选择',
            ACTIVE_MODE_CHASSIS: '底盘参数',
            ACTIVE_MODE_CAMERA: '相机参数',
            ACTIVE_MODE_LIGHT: '光源亮度',
        }
        return names.get(mode, mode)

    def cn_video_state(self, state):
        names = {
            VIDEO_IDLE: '空闲',
            VIDEO_STARTING: '启动中',
            VIDEO_RECORDING: '录制中',
            VIDEO_PAUSED: '已暂停',
            VIDEO_STOPPING: '结束中',
            VIDEO_ERROR: '故障',
        }
        return names.get(state, state)

    def cn_video_command(self, command):
        names = {
            VIDEO_COMMAND_START: '开始',
            VIDEO_COMMAND_PAUSE: '暂停',
            VIDEO_COMMAND_RESUME: '继续',
            VIDEO_COMMAND_STOP: '结束',
        }
        return names.get(command, command)


def main():
    rospy.init_node('teleop_joy_ctrl_node')
    TeleopJoyCtrl()
    rospy.spin()


if __name__ == '__main__':
    main()
