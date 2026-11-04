#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""意图节点：订阅 /joy，只发布归一化底盘意图和有序离散事件。

不发布速度指令，也不调用相机、光源、机械臂或录像。
"""

from __future__ import print_function

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from teleop_joy_ctrl.intent_codec import (
    DEFAULT_CHASSIS_AXES_TOPIC,
    DEFAULT_EVENTS_TOPIC,
    encode_chassis,
    events_message,
)
from teleop_joy_ctrl.joy_parser import JoyParser, clamp


class JoyIntentNode(object):
    def __init__(self):
        self.joy_topic = rospy.get_param('~joy_topic', '/joy')
        self.chassis_axes_topic = rospy.get_param(
            '~chassis_axes_topic', DEFAULT_CHASSIS_AXES_TOPIC)
        self.events_topic = rospy.get_param('~events_topic', DEFAULT_EVENTS_TOPIC)

        axis_deadzone = float(rospy.get_param('~axis_deadzone', 0.08))
        adjust_repeat_interval = float(rospy.get_param('~adjust_repeat_interval', 0.25))
        left_trigger_pressed_threshold = clamp(
            float(rospy.get_param('~left_trigger_pressed_threshold', 0.95)),
            -1.0,
            1.0)

        self.parser = JoyParser(
            axes=rospy.get_param('~axes', {}),
            buttons=rospy.get_param('~buttons', {}),
            axis_deadzone=axis_deadzone,
            adjust_repeat_interval=adjust_repeat_interval,
            left_trigger_pressed_threshold=left_trigger_pressed_threshold,
        )
        self._event_seq = 0
        self.chassis_pub = rospy.Publisher(
            self.chassis_axes_topic, String, queue_size=1, latch=False)
        self.events_pub = rospy.Publisher(
            self.events_topic, String, queue_size=100, latch=False)
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback, queue_size=10)
        rospy.loginfo(
            '手柄意图节点: 手柄话题=%s 底盘意图=%s 离散事件=%s',
            self.joy_topic,
            self.chassis_axes_topic,
            self.events_topic)

    def joy_callback(self, msg):
        now = rospy.Time.now().to_sec()
        result = self.parser.update(msg.axes, msg.buttons, now)
        self.chassis_pub.publish(String(data=encode_chassis(result.chassis, now)))
        event_payload = events_message(result.events, now, self._event_seq + 1)
        if event_payload is None:
            return
        self._event_seq += 1
        self.events_pub.publish(String(data=event_payload))


def main():
    rospy.init_node('joy_intent_node')
    JoyIntentNode()
    rospy.spin()


if __name__ == '__main__':
    main()
