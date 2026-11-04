#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""手柄意图执行。

限速、Twist、相机、光源、机械臂和录像都在这里完成。JoyParser 的结果通过
apply() 以内存对象传入，publish_chassis() 按肩键接管状态发布速度。

机械臂的 wait_for_service 与同步调用仍在本模块，但集中在 ArmServiceGateway，
底盘发布循环不调用它。Step A 仍在手柄回调线程里同步执行 START，后续可以把
gateway.connect / handle_arm_start 挪走，而不改速度发布。
"""

import queue
import threading

from teleop_joy_ctrl.joy_parser import (
    ACTIVE_MODE_CAMERA,
    ACTIVE_MODE_CHASSIS,
    ACTIVE_MODE_LIGHT,
    ACTIVE_MODE_NONE,
    EVENT_ADJUST,
    EVENT_HOME,
    EVENT_LOCK,
    EVENT_MODE,
    EVENT_R3,
    EVENT_START,
    R3_SYNC,
    SPEED_GEAR_HIGH,
    SPEED_GEAR_LOW,
    TARGET_ANGULAR_SPEED,
    TARGET_BRIGHTNESS,
    TARGET_EXPOSURE,
    TARGET_GAIN,
    TARGET_LINEAR_SPEED,
    clamp,
)


MODE_DISABLED = 4
MODE_ENABLE_IDLE = 5
MODE_RUNNING = 7
MODE_SINGLE_MOVE = 8
MODE_ERROR = 9
MODE_COLLISION = 11

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


class R3GestureInterpreter(object):
    """录像状态机：把按下/时间解释成开始、暂停、继续或结束。

    解析器只报告手势相位。是否双击开始、短按暂停或长按结束，由这里结合
    当前录像状态决定。
    """

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


class ArmServiceGateway(object):
    """Dobot 服务访问边界。

    wait_for_service 只出现在 connect()。底盘速度循环不进入这个类。
    """

    def __init__(self, arm_config):
        self.arm = arm_config or {}
        self.config_enabled = bool(self.arm.get('enabled', True))
        self.ready = False
        self.enable_client = None
        self.disable_client = None
        self.clear_error_client = None
        self.robot_mode_client = None

    def service_names(self):
        prefix = self.arm.get('service_prefix', '/dobot_v4_bringup').rstrip('/')
        robot_namespace = self.arm.get('robot_namespace', '').strip('/')
        if robot_namespace:
            prefix = '%s/%s' % (prefix, robot_namespace)
        srv_prefix = '%s/srv' % prefix
        return srv_prefix, {
            'enable': srv_prefix + '/EnableRobot',
            'disable': srv_prefix + '/DisableRobot',
            'clear_error': srv_prefix + '/ClearError',
            'robot_mode': srv_prefix + '/RobotMode',
        }

    def connect(self, service_types):
        """阻塞直到机械臂服务可用，然后建立代理。调用方不要放在速度循环里。"""
        import rospy

        _srv_prefix, names = self.service_names()
        for name in names.values():
            rospy.wait_for_service(name)

        self.enable_client = rospy.ServiceProxy(names['enable'], service_types['enable'])
        self.disable_client = rospy.ServiceProxy(names['disable'], service_types['disable'])
        self.clear_error_client = rospy.ServiceProxy(
            names['clear_error'], service_types['clear_error'])
        self.robot_mode_client = rospy.ServiceProxy(
            names['robot_mode'], service_types['robot_mode'])
        self.ready = True
        return True

    def clients_present(self):
        return (
            self.enable_client is not None
            and self.disable_client is not None
            and self.clear_error_client is not None
            and self.robot_mode_client is not None
        )


class TeleopExecutor(object):
    def __init__(
            self,
            chassis=None,
            camera=None,
            light=None,
            arm=None,
            video=None,
            joy_timeout=0.5,
            debug_success_logs=True,
            chassis_debug_interval=1.0,
            cmd_vel_topic='/cmd_vel',
            state_topic='/teleop_joy_ctrl/state',
            debug_topic='/teleop_joy_ctrl/debug',
            publishers=None,
            sleeper=None,
            logger=None,
            twist_cls=None,
            bool_cls=None,
            float_cls=None,
            string_cls=None):
        self.chassis = chassis if chassis is not None else {}
        self.camera = camera if camera is not None else {}
        self.light = light if light is not None else {}
        self.arm = arm if arm is not None else {}
        self.video = video if video is not None else {}

        self.joy_timeout = float(joy_timeout)
        self.debug_success_logs = bool(debug_success_logs)
        self.chassis_debug_interval = float(chassis_debug_interval)
        self.cmd_vel_topic = cmd_vel_topic
        self.state_topic = state_topic
        self.debug_topic = debug_topic

        self._sleeper = sleeper
        self._logger = logger
        self._Twist = twist_cls
        self._Bool = bool_cls
        self._Float32 = float_cls
        self._String = string_cls

        pubs = publishers or {}
        self.cmd_vel_pub = pubs.get('cmd_vel')
        self.state_pub = pubs.get('state')
        self.debug_pub = pubs.get('debug')
        self.exposure_pub = pubs.get('exposure')
        self.gain_pub = pubs.get('gain')
        self.light_power_pub = pubs.get('light_power')
        self.light_brightness_pub = pubs.get('light_brightness')

        self.locked = False
        self.active_mode = ACTIVE_MODE_NONE
        self._chassis_intent = None
        self._last_joy_time = None
        self._last_chassis_debug_time = 0.0
        self.chassis_command_active = False

        self.max_linear_speed = float(self.chassis.get('max_linear_speed', 0.4))
        self.max_angular_speed = float(self.chassis.get('max_angular_speed', 0.8))
        self.exposure = float(self.camera.get('exposure_default', 5000.0))
        self.gain = float(self.camera.get('gain_default', 0.0))
        self.brightness = float(self.light.get('brightness_default', 0.5))
        self.light_on = bool(self.light.get('assume_initial_on', False))

        self.arm_gateway = ArmServiceGateway(self.arm)
        self.arm_config_enabled = self.arm_gateway.config_enabled
        self.arm_clients_ready = False

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

    def bind_ros_interfaces(self):
        import rospy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, Float32, String

        self._Twist = Twist
        self._Bool = Bool
        self._Float32 = Float32
        self._String = String
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=5, latch=True)
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=20)
        self.exposure_pub = rospy.Publisher(
            self.camera.get('exposure_topic', '/camera/exposure_cmd'),
            Float32,
            queue_size=5)
        self.gain_pub = rospy.Publisher(
            self.camera.get('gain_topic', '/camera/gain_cmd'),
            Float32,
            queue_size=5)
        self.light_power_pub = rospy.Publisher(
            self.light.get('power_topic', '/light/power_cmd'),
            Bool,
            queue_size=5,
            latch=True)
        self.light_brightness_pub = rospy.Publisher(
            self.light.get('brightness_topic', '/light/brightness_cmd'),
            Float32,
            queue_size=5,
            latch=True)

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
            self.video.get('state_topic', '/teleop_joy_ctrl/video/state'),
            String,
            self.video_state_callback,
            queue_size=1)

    def apply(self, result, now):
        self._chassis_intent = result.chassis
        self._last_joy_time = float(now)
        for event in result.events:
            self._dispatch(event)

    def publish_chassis(self, now):
        now = float(now)
        control_requested = self._joy_is_fresh(now) and self._control_active()

        if control_requested:
            twist = self.make_chassis_twist(self._chassis_intent)
            self.cmd_vel_pub.publish(twist)
            self.chassis_command_active = True
            self._maybe_log_chassis_success(twist, now)
            return

        if self.chassis_command_active:
            self.cmd_vel_pub.publish(self._new_twist())
            self.chassis_command_active = False
            self.log_success(
                '底盘手柄控制已释放，已发送一次零速度并停止发布，等待导航接管')

    def make_chassis_twist(self, intent):
        twist = self._new_twist()
        if intent is None or not intent.active:
            return twist
        if intent.speed_gear == SPEED_GEAR_LOW:
            speed_scale = float(self.chassis.get('low_speed_scale', 0.35))
        elif intent.speed_gear == SPEED_GEAR_HIGH:
            speed_scale = float(self.chassis.get('high_speed_scale', 1.0))
        else:
            return twist

        twist.linear.x = intent.left_y * self.max_linear_speed * speed_scale
        if bool(self.chassis.get('holonomic', False)):
            twist.linear.y = intent.left_x * self.max_linear_speed * speed_scale
        else:
            twist.angular.z = intent.left_x * self.max_angular_speed * speed_scale
        return twist

    def _control_active(self):
        return self._chassis_intent is not None and self._chassis_intent.active

    def _joy_is_fresh(self, now):
        if self._last_joy_time is None:
            return False
        return (now - self._last_joy_time) <= self.joy_timeout

    def _maybe_log_chassis_success(self, twist, now):
        if not self.debug_success_logs:
            return
        if self._is_zero_twist(twist):
            return
        if (now - self._last_chassis_debug_time) < self.chassis_debug_interval:
            return
        self._last_chassis_debug_time = now
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

    @staticmethod
    def _is_zero_twist(twist):
        return (
            abs(twist.linear.x) < 1e-6
            and abs(twist.linear.y) < 1e-6
            and abs(twist.linear.z) < 1e-6
            and abs(twist.angular.x) < 1e-6
            and abs(twist.angular.y) < 1e-6
            and abs(twist.angular.z) < 1e-6
        )

    def _dispatch(self, event):
        event_type = event.get('type')
        if event_type == EVENT_R3:
            self._handle_r3(event)
        elif event_type == EVENT_LOCK:
            self._handle_lock(event)
        elif event_type == EVENT_START:
            self._handle_start(event)
        elif event_type == EVENT_MODE:
            self._handle_mode(event)
        elif event_type == EVENT_HOME:
            self.toggle_light()
        elif event_type == EVENT_ADJUST:
            self._handle_adjust(event)

    def _handle_lock(self, event):
        self.locked = bool(event.get('locked'))
        self.active_mode = ACTIVE_MODE_NONE
        self.publish_state('锁定=%s' % self.cn_bool(self.locked))

    def _handle_mode(self, event):
        self.active_mode = event.get('mode', ACTIVE_MODE_NONE)
        labels = {
            ACTIVE_MODE_CHASSIS: '模式=底盘参数',
            ACTIVE_MODE_CAMERA: '模式=相机参数',
            ACTIVE_MODE_LIGHT: '模式=光源亮度',
        }
        self.publish_state(labels.get(
            self.active_mode, '模式=%s' % self.active_mode))

    def _handle_start(self, event):
        self.log_success('START按键已识别 index=%d locked=%s arm_ready=%s' % (
            int(event.get('index', -1)),
            self.cn_bool(self.locked),
            self.cn_bool(self.arm_clients_ready)))
        self.handle_arm_start()

    def _handle_adjust(self, event):
        target = event.get('target')
        direction = int(event.get('direction', 0))
        if direction == 0:
            return
        if target == TARGET_LINEAR_SPEED:
            self._adjust_linear_speed(direction)
        elif target == TARGET_ANGULAR_SPEED:
            self._adjust_angular_speed(direction)
        elif target == TARGET_GAIN:
            self._adjust_gain(direction)
        elif target == TARGET_EXPOSURE:
            self._adjust_exposure(direction)
        elif target == TARGET_BRIGHTNESS:
            self._adjust_brightness(direction)

    def _adjust_linear_speed(self, direction):
        step = max(
            float(self.chassis.get('linear_speed_step_min', 0.02)),
            abs(self.max_linear_speed) * float(
                self.chassis.get('linear_speed_step_ratio', 0.10)))
        self.max_linear_speed = clamp(
            self.max_linear_speed + direction * step,
            float(self.chassis.get('min_linear_speed', 0.05)),
            float(self.chassis.get('max_linear_speed_limit', 1.5)))
        self.publish_state('底盘线速度上限=%.3f' % self.max_linear_speed)

    def _adjust_angular_speed(self, direction):
        step = max(
            float(self.chassis.get('angular_speed_step_min', 0.05)),
            abs(self.max_angular_speed) * float(
                self.chassis.get('angular_speed_step_ratio', 0.10)))
        self.max_angular_speed = clamp(
            self.max_angular_speed + direction * step,
            float(self.chassis.get('min_angular_speed', 0.05)),
            float(self.chassis.get('max_angular_speed_limit', 2.5)))
        self.publish_state('底盘角速度上限=%.3f' % self.max_angular_speed)

    def _adjust_gain(self, direction):
        step = max(
            float(self.camera.get('gain_step_min', 0.5)),
            abs(self.gain) * float(self.camera.get('gain_step_ratio', 0.10)))
        self.gain = clamp(
            self.gain + direction * step,
            float(self.camera.get('gain_min', 0.0)),
            float(self.camera.get('gain_max', 24.0)))
        self.gain_pub.publish(self._float32(self.gain))
        self.log_success('相机增益指令 话题=%s 数值=%.3f' % (
            self.camera.get('gain_topic', '/camera/gain_cmd'), self.gain))
        self.publish_state('相机增益=%.3f' % self.gain)

    def _adjust_exposure(self, direction):
        step = max(
            float(self.camera.get('exposure_step_min', 100.0)),
            abs(self.exposure) * float(
                self.camera.get('exposure_step_ratio', 0.12)))
        self.exposure = clamp(
            self.exposure + direction * step,
            float(self.camera.get('exposure_min', 20.0)),
            float(self.camera.get('exposure_max', 50000.0)))
        self.exposure_pub.publish(self._float32(self.exposure))
        self.log_success('相机曝光指令 话题=%s 数值=%.1f' % (
            self.camera.get('exposure_topic', '/camera/exposure_cmd'),
            self.exposure))
        self.publish_state('相机曝光=%.1f' % self.exposure)

    def _adjust_brightness(self, direction):
        if direction == 0:
            return
        was_light_on = self.light_on
        was_off_by_brightness = self.is_light_off_by_brightness()
        self.brightness = clamp(
            self.brightness + direction * float(self.light.get('brightness_step', 0.05)),
            float(self.light.get('brightness_min', 0.0)),
            float(self.light.get('brightness_max', 1.0)))
        self.light_on = not self.is_light_off_by_brightness()
        if self.light_on and (was_off_by_brightness or not was_light_on):
            self.publish_light_power()
            self._sleep(float(self.light.get('power_brightness_delay', 0.05)))
            self.publish_light_brightness()
        elif not self.light_on and was_light_on:
            self.publish_light_brightness()
            self._sleep(float(self.light.get('power_brightness_delay', 0.05)))
            self.publish_light_power()
        else:
            self.publish_light_brightness()
        self.publish_state('光源亮度=%.2f' % self.brightness)

    def toggle_light(self):
        if not bool(self.light.get('enabled', True)):
            self._logwarn('手柄总控节点: 光源控制已通过参数关闭')
            return

        if self.is_light_off_by_brightness():
            self.brightness = clamp(
                float(self.light.get(
                    'home_restore_brightness',
                    self.light.get('brightness_default', 0.5))),
                float(self.light.get('brightness_min', 0.0)),
                float(self.light.get('brightness_max', 1.0)))
            self.light_on = True
            self.publish_light_power()
            self._sleep(float(self.light.get('power_brightness_delay', 0.05)))
            self.publish_light_brightness()
            self.publish_state('HOME恢复光源固定亮度=%.2f' % self.brightness)
            return

        self.brightness = float(self.light.get('brightness_min', 0.0))
        self.publish_light_brightness()
        self._sleep(float(self.light.get('power_brightness_delay', 0.05)))
        self.light_on = False
        self.publish_light_power()
        self.publish_state('HOME关闭光源，亮度=0')

    def publish_light_power(self):
        if not bool(self.light.get('enabled', True)):
            return
        repeat = max(1, int(self.light.get('power_publish_repeat', 1)))
        interval = max(0.0, float(self.light.get('power_publish_repeat_interval', 0.05)))
        for index in range(repeat):
            self.light_power_pub.publish(self._bool(self.light_on))
            if index + 1 < repeat and interval > 0.0:
                self._sleep(interval)
        self.log_success('光源开关指令 话题=%s 状态=%s' % (
            self.light.get('power_topic', '/light/power_cmd'),
            '开启' if self.light_on else '关闭',
        ))

    def publish_light_brightness(self):
        if not bool(self.light.get('enabled', True)):
            return
        self.light_brightness_pub.publish(self._float32(self.brightness))
        self.log_success('光源亮度指令 话题=%s 亮度=%.3f 光源状态=%s' % (
            self.light.get('brightness_topic', '/light/brightness_cmd'),
            self.brightness,
            '开启' if self.light_on else '关闭',
        ))

    def is_light_off_by_brightness(self):
        epsilon = float(self.light.get('off_brightness_epsilon', 0.001))
        return self.brightness <= float(self.light.get('brightness_min', 0.0)) + epsilon

    def _handle_r3(self, event):
        if not self.video_enabled:
            return

        pressed = bool(event.get('pressed'))
        when = float(event.get('time', 0.0))
        if event.get('phase') == R3_SYNC:
            self.video_gesture.reset(pressed, when)
            return

        commands = self.video_gesture.update(
            pressed, when, self.video_input_state)
        for command in commands:
            with self.video_operation_lock:
                if self.video_operation_pending:
                    self._logwarn(
                        '手柄总控节点: 录像操作仍在执行，已忽略R3操作=%s'
                        % self.cn_video_command(command))
                    continue
                self.video_operation_pending = True
            try:
                self.video_command_queue.put_nowait(command)
            except queue.Full:
                with self.video_operation_lock:
                    self.video_operation_pending = False
                self._logwarn(
                    '手柄总控节点: 录像指令队列已满，已忽略操作=%s'
                    % self.cn_video_command(command))
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
                'R3录像指令已识别，操作=%s' % self.cn_video_command(command))

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
            self._logwarn('手柄总控节点: 收到未知录像状态=%s' % state)
            return
        if state != self.video_state:
            self.video_state = state
            self.video_input_state = state
            self.publish_state('录像状态更新=%s' % self.cn_video_state(state))

    def init_video_clients(self):
        if not self.video_enabled:
            self._loginfo('手柄总控节点: 视频录制控制已通过参数关闭')
            return

        try:
            from std_srvs.srv import Trigger
            from teleop_joy_ctrl.video_process_manager import call_with_timeout
        except ImportError as exc:
            self.video_enabled = False
            self.video_state = VIDEO_ERROR
            self.video_input_state = VIDEO_ERROR
            self._logerr(
                '手柄总控节点: 无法导入std_srvs/Trigger，视频录制控制不可用，错误=%s'
                % exc)
            return

        import rospy

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
        import rospy
        self.video_clients = {
            command: rospy.ServiceProxy(name, self.video_service_type)
            for command, name in self.video_service_names.items()
        }

    def video_command_worker(self):
        import rospy
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
                    self._logerr(
                        '手柄总控节点: 录像服务调用超时=%s 超时=%.1fs' % (
                            self.cn_video_command(command),
                            self.video_command_timeout))
                    self.publish_state('录像服务调用超时')
                    continue
                if error is not None:
                    raise error
                if response.success:
                    self.log_success(str(response.message))
                else:
                    self.video_input_state = self.video_state
                    self._logerr(
                        '手柄总控节点: 录像操作失败=%s 原因=%s' % (
                            self.cn_video_command(command),
                            response.message))
            except Exception as exc:
                self.video_input_state = self.video_state
                self._logerr(
                    '手柄总控节点: 录像服务调用异常=%s 错误=%s' % (
                        self.cn_video_command(command),
                        exc))
                self.publish_state('录像服务调用异常')
            finally:
                with self.video_operation_lock:
                    self.video_operation_pending = False
                self.video_command_queue.task_done()

    def stop_video_worker(self):
        self.video_worker_stop.set()

    def init_arm_clients(self):
        if not self.arm_config_enabled:
            self._loginfo('手柄总控节点: 机械臂控制已通过参数关闭')
            return False

        try:
            from dobot_v4_bringup.srv import ClearError, DisableRobot, EnableRobot, RobotMode
        except ImportError as exc:
            self.arm_clients_ready = False
            self.arm_gateway.ready = False
            self._logerr(
                '手柄总控节点: 无法导入 dobot_v4_bringup 服务定义，请确认已 source 机械臂工作空间，错误=%s'
                % exc)
            return False

        srv_prefix, _names = self.arm_gateway.service_names()
        self._loginfo('手柄总控节点: 等待 dobot_v4_bringup 机械臂服务...')
        self.arm_gateway.connect({
            'enable': EnableRobot,
            'disable': DisableRobot,
            'clear_error': ClearError,
            'robot_mode': RobotMode,
        })
        self.arm_clients_ready = True
        self._loginfo('手柄总控节点: 机械臂服务已就绪，服务前缀=%s' % srv_prefix)
        return True

    def ensure_arm_ready(self, action_name):
        if not self.arm_config_enabled:
            self._logwarn(
                '手柄总控节点: %s 已忽略，因为配置中已关闭机械臂控制' % action_name)
            return False

        if not self.arm_clients_ready:
            self._logwarn('手柄总控节点: 机械臂服务未连接，正在重新连接')
            if not self.init_arm_clients():
                self._logwarn(
                    '手柄总控节点: %s 暂不能控制机械臂，请检查 Dobot bringup 是否已启动、当前终端是否已 source 机械臂工作空间'
                    % action_name)
                self.publish_state('机械臂服务未连接')
                return False

        if not self.arm_gateway.clients_present():
            self.arm_clients_ready = False
            self.arm_gateway.ready = False
            self._logwarn(
                '手柄总控节点: 机械臂服务客户端为空，%s 下次会重新连接' % action_name)
            return False

        return True

    def handle_arm_start(self):
        self._loginfo(
            '手柄总控节点: START进入机械臂状态机处理 locked=%s arm_ready=%s' % (
                self.cn_bool(self.locked),
                self.cn_bool(self.arm_clients_ready)))
        if not self.ensure_arm_ready('START机械臂状态机'):
            return

        mode = self.query_robot_mode()
        if mode in (MODE_ERROR, MODE_COLLISION):
            self._logwarn(
                '手柄总控节点: START检测到机械臂报错/碰撞状态(mode=%d)，执行清错' % mode)
            self.call_clear_error()
            return

        if mode in (MODE_ENABLE_IDLE, MODE_RUNNING, MODE_SINGLE_MOVE):
            self._loginfo('手柄总控节点: START -> 下使能机械臂，当前模式=%d' % mode)
            self.call_disable_robot()
            return

        self._loginfo('手柄总控节点: START -> 上使能机械臂，当前模式=%d' % mode)
        self.call_enable_robot()

    def query_robot_mode(self):
        import rospy
        try:
            resp = self.arm_gateway.robot_mode_client()
        except rospy.ServiceException as exc:
            self._logwarn('手柄总控节点: RobotMode服务调用失败: %s' % exc)
            return -1

        mode = getattr(resp, 'res', 0)
        return int(mode)

    def call_clear_error(self):
        import rospy
        try:
            self.arm_gateway.clear_error_client()
        except rospy.ServiceException as exc:
            self._logerr('手柄总控节点: ClearError服务调用失败: %s' % exc)
            return
        self._sleep(float(self.arm.get('command_settle_time', 0.3)))
        mode = self.query_robot_mode()
        if mode == MODE_ERROR:
            self._logwarn('手柄总控节点: ClearError已发送，但机械臂仍处于ERROR(9)')
        else:
            self.log_success('机械臂清错指令 服务=ClearError 执行后模式=%d' % mode)
            self.publish_state('机械臂模式=%d 清错后' % mode)

    def call_enable_robot(self):
        import rospy
        try:
            self.arm_gateway.enable_client()
        except rospy.ServiceException as exc:
            self._logerr('手柄总控节点: EnableRobot服务调用失败: %s' % exc)
            return
        self._sleep(float(self.arm.get('command_settle_time', 0.3)))
        mode = self.query_robot_mode()
        if mode in (MODE_ENABLE_IDLE, MODE_RUNNING, MODE_SINGLE_MOVE):
            self.log_success('机械臂上使能指令 服务=EnableRobot 执行后模式=%d' % mode)
            self.publish_state('机械臂模式=%d 上使能后' % mode)
        else:
            self._logwarn(
                '手柄总控节点: EnableRobot已发送，但机械臂当前模式=%d，未确认进入使能态' % mode)
            self.publish_state('机械臂上使能后模式=%d' % mode)

    def call_disable_robot(self):
        import rospy
        try:
            self.arm_gateway.disable_client()
        except rospy.ServiceException as exc:
            self._logerr('手柄总控节点: DisableRobot服务调用失败: %s' % exc)
            return
        self._sleep(float(self.arm.get('command_settle_time', 0.3)))
        mode = self.query_robot_mode()
        if mode == MODE_DISABLED:
            self.log_success('机械臂下使能指令 服务=DisableRobot 执行后模式=%d' % mode)
            self.publish_state('机械臂模式=%d 下使能后' % mode)
        else:
            self._logwarn(
                '手柄总控节点: DisableRobot已发送，但机械臂当前模式=%d，可能被控制器拒绝' % mode)
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
        if self.state_pub is not None:
            self.state_pub.publish(self._string(state))
        self._loginfo('手柄总控节点: %s' % state)

    def log_success(self, text):
        if not self.debug_success_logs:
            return
        msg = '[成功] %s' % text
        if self.debug_pub is not None:
            self.debug_pub.publish(self._string(msg))
        self._loginfo('手柄总控节点: %s' % msg)

    def _sleep(self, seconds):
        if self._sleeper is not None:
            self._sleeper(seconds)
            return
        import rospy
        rospy.sleep(seconds)

    def _emit(self, level, text):
        if self._logger is not None:
            getattr(self._logger, level)(text)
            return
        import rospy
        log_fn = {
            'info': rospy.loginfo,
            'warn': rospy.logwarn,
            'error': rospy.logerr,
        }[level]
        log_fn(text)

    def _loginfo(self, text):
        self._emit('info', text)

    def _logwarn(self, text):
        self._emit('warn', text)

    def _logerr(self, text):
        self._emit('error', text)

    def _new_twist(self):
        if self._Twist is None:
            from geometry_msgs.msg import Twist
            self._Twist = Twist
        return self._Twist()

    def _bool(self, value):
        if self._Bool is None:
            from std_msgs.msg import Bool
            self._Bool = Bool
        return self._Bool(value)

    def _float32(self, value):
        if self._Float32 is None:
            from std_msgs.msg import Float32
            self._Float32 = Float32
        return self._Float32(value)

    def _string(self, value):
        if self._String is None:
            from std_msgs.msg import String
            self._String = String
        return self._String(value)

    @staticmethod
    def cn_bool(value):
        return '是' if value else '否'

    @staticmethod
    def cn_mode(mode):
        names = {
            ACTIVE_MODE_NONE: '未选择',
            ACTIVE_MODE_CHASSIS: '底盘参数',
            ACTIVE_MODE_CAMERA: '相机参数',
            ACTIVE_MODE_LIGHT: '光源亮度',
        }
        return names.get(mode, mode)

    @staticmethod
    def cn_video_state(state):
        names = {
            VIDEO_IDLE: '空闲',
            VIDEO_STARTING: '启动中',
            VIDEO_RECORDING: '录制中',
            VIDEO_PAUSED: '已暂停',
            VIDEO_STOPPING: '结束中',
            VIDEO_ERROR: '故障',
        }
        return names.get(state, state)

    @staticmethod
    def cn_video_command(command):
        names = {
            VIDEO_COMMAND_START: '开始',
            VIDEO_COMMAND_PAUSE: '暂停',
            VIDEO_COMMAND_RESUME: '继续',
            VIDEO_COMMAND_STOP: '结束',
        }
        return names.get(command, command)
