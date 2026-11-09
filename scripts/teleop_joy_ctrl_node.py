#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from sensor_msgs.msg import Joy

from teleop_joy_ctrl.joy_parser import JoyParser, clamp
from teleop_joy_ctrl.teleop_executor import TeleopExecutor

__all__ = [
    'JoyParser',
    'TeleopExecutor',
    'TeleopJoyCtrl',
]


class TeleopJoyCtrl(object):
    """同一进程内的薄节点：Joy 回调进解析器，再把意图交给执行器。"""

    def __init__(self):
        self.joy_topic = rospy.get_param('~joy_topic', '/joy')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.state_topic = rospy.get_param('~state_topic', '/teleop_joy_ctrl/state')
        self.debug_topic = rospy.get_param('~debug_topic', '/teleop_joy_ctrl/debug')

        publish_rate = float(rospy.get_param('~publish_rate', 20.0))
        joy_timeout = float(rospy.get_param('~joy_timeout', 0.5))
        axis_deadzone = float(rospy.get_param('~axis_deadzone', 0.08))
        adjust_repeat_interval = float(rospy.get_param('~adjust_repeat_interval', 0.25))
        left_trigger_pressed_threshold = clamp(
            float(rospy.get_param('~left_trigger_pressed_threshold', 0.95)),
            -1.0,
            1.0)
        debug_success_logs = bool(rospy.get_param('~debug_success_logs', True))
        chassis_debug_interval = float(rospy.get_param('~chassis_debug_interval', 1.0))

        self.parser = JoyParser(
            axes=rospy.get_param('~axes', {}),
            buttons=rospy.get_param('~buttons', {}),
            axis_deadzone=axis_deadzone,
            adjust_repeat_interval=adjust_repeat_interval,
            left_trigger_pressed_threshold=left_trigger_pressed_threshold,
        )
        self.executor = TeleopExecutor(
            chassis=rospy.get_param('~chassis', {}),
            camera=rospy.get_param('~camera', {}),
            light=rospy.get_param('~light', {}),
            arm=rospy.get_param('~arm', {}),
            joy_timeout=joy_timeout,
            debug_success_logs=debug_success_logs,
            chassis_debug_interval=chassis_debug_interval,
            cmd_vel_topic=self.cmd_vel_topic,
            state_topic=self.state_topic,
            debug_topic=self.debug_topic,
        )
        self.executor.bind_ros_interfaces()
        self.executor.init_arm_clients()
        self.executor.publish_state('节点已启动')

        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback, queue_size=10)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(period), self.publish_loop)
        rospy.loginfo(
            '手柄总控节点: 手柄话题=%s 底盘速度话题=%s 状态话题=%s',
            self.joy_topic,
            self.cmd_vel_topic,
            self.state_topic)

    def joy_callback(self, msg):
        now = rospy.Time.now().to_sec()
        result = self.parser.update(msg.axes, msg.buttons, now)
        self.executor.apply(result, now)

    def publish_loop(self, _event):
        self.executor.publish_chassis(rospy.Time.now().to_sec())


def main():
    rospy.init_node('teleop_joy_ctrl_node')
    TeleopJoyCtrl()
    rospy.spin()


if __name__ == '__main__':
    main()
