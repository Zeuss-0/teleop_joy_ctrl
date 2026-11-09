#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""执行器节点：订阅归一化意图，拥有限速、状态机和对外硬件话题。

底盘速度仍发布到 ``/teleop_joy_ctrl/cmd_vel``。机械臂连接和 START 在后台线程
执行，不占用本节点的回调线程。
"""

from __future__ import print_function

import rospy
from std_msgs.msg import String

from teleop_joy_ctrl.intent_codec import (
    DEFAULT_CHASSIS_AXES_TOPIC,
    DEFAULT_EVENTS_TOPIC,
    decode_chassis,
    decode_events,
)
from teleop_joy_ctrl.teleop_executor import TeleopExecutor


class TeleopExecutorNode(object):
    def __init__(self):
        self.chassis_axes_topic = rospy.get_param(
            '~chassis_axes_topic', DEFAULT_CHASSIS_AXES_TOPIC)
        self.events_topic = rospy.get_param('~events_topic', DEFAULT_EVENTS_TOPIC)
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.state_topic = rospy.get_param('~state_topic', '/teleop_joy_ctrl/state')
        self.debug_topic = rospy.get_param('~debug_topic', '/teleop_joy_ctrl/debug')

        publish_rate = float(rospy.get_param('~publish_rate', 20.0))
        joy_timeout = float(rospy.get_param('~joy_timeout', 0.5))
        debug_success_logs = bool(rospy.get_param('~debug_success_logs', True))
        chassis_debug_interval = float(rospy.get_param('~chassis_debug_interval', 1.0))

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
        self.executor.start_arm_worker()
        rospy.on_shutdown(self.executor.stop_arm_worker)
        self.executor.publish_state('执行器节点已启动')

        rospy.Subscriber(
            self.chassis_axes_topic,
            String,
            self.chassis_callback,
            queue_size=1)
        rospy.Subscriber(
            self.events_topic,
            String,
            self.events_callback,
            queue_size=100)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(period), self.publish_loop)
        rospy.loginfo(
            '手柄执行器节点: 底盘意图=%s 离散事件=%s 速度话题=%s 状态话题=%s',
            self.chassis_axes_topic,
            self.events_topic,
            self.cmd_vel_topic,
            self.state_topic)

    def chassis_callback(self, msg):
        decoded = decode_chassis(msg.data)
        if decoded is None:
            rospy.logwarn('手柄执行器节点: 无法解析底盘意图')
            return
        intent, stamp = decoded
        now = rospy.Time.now().to_sec()
        if not self.executor.apply_chassis(intent, now, stamp=stamp):
            rospy.logwarn('手柄执行器节点: 已忽略无效底盘意图')

    def events_callback(self, msg):
        message = decode_events(msg.data)
        if message is None:
            rospy.logwarn('手柄执行器节点: 无法解析离散事件')
            return
        now = rospy.Time.now().to_sec()
        if not self.executor.apply_event_message(message, now):
            rospy.logwarn('手柄执行器节点: 已忽略无效离散事件')

    def publish_loop(self, _event):
        self.executor.publish_chassis(rospy.Time.now().to_sec())


def main():
    rospy.init_node('teleop_executor_node')
    TeleopExecutorNode()
    rospy.spin()


if __name__ == '__main__':
    main()
