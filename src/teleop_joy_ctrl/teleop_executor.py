#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""手柄意图执行。

限速、Twist、相机、光源和机械臂都在这里完成。同进程路径用 apply()
一次送入解析结果。两个节点时，执行器入口用 apply_chassis() 和
apply_event_message()：轴夹到 [-1, 1]，报文时间只用来丢掉超前或过期的包，
存活时间以本机接收时钟为准。离散事件还要带会话和递增序号，重复包和旧会话
不会再次执行。锁定以执行器为准，锁定期间模式、HOME 和调参直接丢弃。
右摇杆按下不再进入本模块，类型为 r3 的事件直接忽略。

机械臂的 wait_for_service 与同步调用集中在 ArmServiceGateway，底盘发布循环
不进入这个类。执行器节点调用 start_arm_worker() 后，连接和 START 在后台线程
运行。服务未就绪或上一条 START 还没做完时直接拒绝，不把按键积压到恢复之后。
未启动该线程时（同进程旧节点和单元测试），START 仍在调用线程里同步执行。
"""

import math
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
    EVENT_START,
    ChassisIntent,
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

# 报文时间最多比本机接收时间超前这么多，避免把时钟抖动当成超前攻击。
STAMP_FUTURE_SKEW = 0.05
_LOCK_GATED_EVENTS = (EVENT_MODE, EVENT_HOME, EVENT_ADJUST)
_RETIRED_EVENT_SESSIONS = 8


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

    def connect(self, service_types, wait_slice=None, should_stop=None):
        """阻塞直到机械臂服务可用，然后建立代理。调用方不要放在速度循环里。

        ``wait_slice`` 为秒时，按片等待并在 ``should_stop`` 为真时返回 False，
        方便后台线程在关机时退出。默认仍一次等到服务出现。
        """
        import rospy

        _srv_prefix, names = self.service_names()
        for name in names.values():
            if wait_slice is None:
                rospy.wait_for_service(name)
                continue
            while True:
                if should_stop is not None and should_stop():
                    return False
                try:
                    rospy.wait_for_service(name, timeout=float(wait_slice))
                    break
                except rospy.ROSInterruptException:
                    return False
                except rospy.ROSException:
                    continue

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
        self._arm_gate = threading.Lock()
        self._arm_job_pending = False
        self.arm_clients_ready = False
        self._event_session = None
        self._event_seq = 0
        self._retired_event_sessions = []

        self._arm_async = False
        self._arm_queue = None
        self._arm_worker_stop = None
        self._arm_worker = None

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

    def apply(self, result, now):
        self.apply_chassis(result.chassis, now)
        self.apply_events(result.events)

    def apply_chassis(self, intent, now, stamp=None):
        """记录一帧归一化底盘意图。

        ``now`` 是本机接收时间，也是超时时钟。``stamp`` 是报文时间，只用来
        丢掉超前或过期的包，不能把存活时间推到未来。轴在这里夹到 [-1, 1]。
        拒绝时不刷新 ``_last_joy_time``。返回是否接受。
        """
        now = float(now)
        if not self._chassis_stamp_ok(stamp, now):
            return False
        sanitized = self._sanitize_chassis(intent)
        if sanitized is None:
            self._logwarn('手柄总控节点: 底盘轴无效，已忽略')
            return False
        self._chassis_intent = sanitized
        self._last_joy_time = now
        return True

    def apply_events(self, events):
        """按给定顺序处理离散事件。同一条消息里的顺序就是同一帧的顺序。"""
        for event in events or []:
            self._dispatch(event)

    def apply_event_message(self, message, now):
        """话题入口。校验会话、序号和报文时间后再按原顺序执行。"""
        if message is None or not self._accept_event_header(message, now):
            return False
        self.apply_events(message.events)
        return True

    def _chassis_stamp_ok(self, stamp, now):
        if stamp is None:
            return True
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            self._logwarn('手柄总控节点: 底盘意图时间无效，已忽略')
            return False
        if not math.isfinite(stamp):
            self._logwarn('手柄总控节点: 底盘意图时间无效，已忽略')
            return False
        if stamp > now + STAMP_FUTURE_SKEW:
            self._logwarn('手柄总控节点: 底盘意图时间超前，已忽略')
            return False
        if (now - stamp) > self.joy_timeout:
            self._logwarn('手柄总控节点: 底盘意图已过期，已忽略')
            return False
        return True

    def _sanitize_chassis(self, intent):
        if intent is None:
            return None
        left_x = self._finite_axis(intent.left_x)
        left_y = self._finite_axis(intent.left_y)
        if left_x is None or left_y is None:
            return None
        return ChassisIntent(
            intent.active,
            intent.speed_gear,
            left_x,
            left_y,
        )

    @staticmethod
    def _finite_axis(value):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        return clamp(number, -1.0, 1.0)

    @staticmethod
    def _bounded_axis(value):
        number = TeleopExecutor._finite_axis(value)
        if number is None:
            return 0.0
        return number

    def _accept_event_header(self, message, now):
        now = float(now)
        session = message.session
        seq = message.seq
        stamp = message.stamp
        if not isinstance(session, str) or not session or isinstance(seq, bool):
            self._logwarn('手柄总控节点: 离散事件缺少会话或序号，已忽略')
            return False
        try:
            seq = int(seq)
            stamp = float(stamp)
        except (TypeError, ValueError):
            self._logwarn('手柄总控节点: 离散事件缺少会话或序号，已忽略')
            return False
        if seq < 1 or not math.isfinite(stamp):
            self._logwarn('手柄总控节点: 离散事件序号或时间无效，已忽略')
            return False
        if stamp > now + STAMP_FUTURE_SKEW:
            self._logwarn('手柄总控节点: 离散事件时间超前，已忽略')
            return False
        if (now - stamp) > self.joy_timeout:
            self._logwarn('手柄总控节点: 离散事件已过期，已忽略')
            return False
        if session in self._retired_event_sessions:
            self._logwarn('手柄总控节点: 离散事件属于已结束的会话，已忽略')
            return False
        if session != self._event_session:
            if self._event_session:
                self._retired_event_sessions.append(self._event_session)
                if len(self._retired_event_sessions) > _RETIRED_EVENT_SESSIONS:
                    del self._retired_event_sessions[0]
            self._event_session = session
            self._event_seq = seq
            return True
        if seq <= self._event_seq:
            self._logwarn('手柄总控节点: 离散事件序号重复，已忽略')
            return False
        self._event_seq = seq
        return True

    def _refuse_start(self, reason):
        self._logwarn('手柄总控节点: %s' % reason)
        self.publish_state(reason)

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

        left_x = self._bounded_axis(intent.left_x)
        left_y = self._bounded_axis(intent.left_y)
        twist.linear.x = left_y * self.max_linear_speed * speed_scale
        if bool(self.chassis.get('holonomic', False)):
            twist.linear.y = left_x * self.max_linear_speed * speed_scale
        else:
            twist.angular.z = left_x * self.max_angular_speed * speed_scale
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
        if self.locked and event_type in _LOCK_GATED_EVENTS:
            self._logwarn('手柄总控节点: 参数已锁定，已忽略事件=%s' % event_type)
            self.publish_state('锁定中忽略事件=%s' % event_type)
            return
        if event_type == 'r3':
            return
        if event_type == EVENT_LOCK:
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
        if not self._arm_async:
            self.handle_arm_start()
            return
        with self._arm_gate:
            if not self.arm_clients_ready:
                self._refuse_start('机械臂服务未连接，已忽略本次START，恢复后请再按一次')
                return
            if self._arm_job_pending:
                self._refuse_start('机械臂指令仍在执行，已忽略本次START')
                return
            self._arm_job_pending = True
        try:
            self._arm_queue.put_nowait(True)
        except queue.Full:
            with self._arm_gate:
                self._arm_job_pending = False
            self._refuse_start('机械臂指令仍在执行，已忽略本次START')

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

    def init_arm_clients(self, wait_slice=None, should_stop=None):
        if not self.arm_config_enabled:
            self._loginfo('手柄总控节点: 机械臂控制已通过参数关闭')
            return False

        try:
            from dobot_v4_bringup.srv import ClearError, DisableRobot, EnableRobot, RobotMode
        except ImportError as exc:
            self._set_arm_clients_ready(False)
            self._logerr(
                '手柄总控节点: 无法导入 dobot_v4_bringup 服务定义，请确认已 source 机械臂工作空间，错误=%s'
                % exc)
            return False

        srv_prefix, _names = self.arm_gateway.service_names()
        self._loginfo('手柄总控节点: 等待 dobot_v4_bringup 机械臂服务...')
        connected = self.arm_gateway.connect(
            {
                'enable': EnableRobot,
                'disable': DisableRobot,
                'clear_error': ClearError,
                'robot_mode': RobotMode,
            },
            wait_slice=wait_slice,
            should_stop=should_stop,
        )
        if not connected:
            self._set_arm_clients_ready(False)
            self._loginfo('手柄总控节点: 机械臂服务连接已中止')
            return False
        self._set_arm_clients_ready(True)
        self._loginfo('手柄总控节点: 机械臂服务已就绪，服务前缀=%s' % srv_prefix)
        return True

    def _set_arm_clients_ready(self, ready):
        with self._arm_gate:
            self.arm_clients_ready = bool(ready)
            if not ready:
                self.arm_gateway.ready = False

    def start_arm_worker(self):
        """把机械臂连接和 START 状态机挪出底盘/事件回调线程。"""
        if self._arm_async:
            return
        self._arm_queue = queue.Queue(maxsize=1)
        self._arm_worker_stop = threading.Event()
        self._arm_async = True
        self._arm_worker = threading.Thread(
            target=self._arm_worker_loop,
            name='teleop_arm_worker')
        self._arm_worker.daemon = True
        self._arm_worker.start()

    def stop_arm_worker(self):
        if self._arm_worker_stop is not None:
            self._arm_worker_stop.set()
        worker = self._arm_worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.5)
        if worker is not None and worker.is_alive():
            return
        self._arm_async = False
        self._arm_worker = None

    def _arm_worker_loop(self):
        if self.arm_config_enabled:
            try:
                self.init_arm_clients(
                    wait_slice=0.5,
                    should_stop=self._arm_worker_stop.is_set)
            except Exception as exc:
                if exc.__class__.__name__ == 'ROSInterruptException':
                    return
                self._logerr('手柄总控节点: 机械臂后台连接失败: %s' % exc)
        while not self._arm_worker_stop.is_set():
            try:
                self._arm_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.handle_arm_start()
            except Exception as exc:
                if exc.__class__.__name__ == 'ROSInterruptException':
                    return
                self._logerr('手柄总控节点: START机械臂处理异常: %s' % exc)
            finally:
                with self._arm_gate:
                    self._arm_job_pending = False
                self._arm_queue.task_done()

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
            self._set_arm_clients_ready(False)
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
            '曝光=%.1f 增益=%.3f 光源=%s 亮度=%.2f'
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
