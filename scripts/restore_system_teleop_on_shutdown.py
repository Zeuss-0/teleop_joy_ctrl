#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import subprocess
import time

import rospy
from std_msgs.msg import String


class SystemTeleopRestorer(object):
    def __init__(self):
        self.enabled = bool(rospy.get_param('~enabled', True))
        self.package = rospy.get_param('~package', 'robot_driver')
        self.launch_file = rospy.get_param('~launch_file', 'teleop.launch')
        self.delay = float(rospy.get_param('~delay', 1.0))
        self.log_path = rospy.get_param(
            '~log_path',
            '/tmp/teleop_joy_ctrl_restore_system_teleop.log')
        self.debug_topic = rospy.get_param('~debug_topic', '/teleop_joy_ctrl/debug')
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=10)

        rospy.on_shutdown(self.restore)
        rospy.loginfo(
            '系统手柄恢复节点: 已启动 enabled=%s package=%s launch=%s delay=%.2f',
            self.enabled,
            self.package,
            self.launch_file,
            self.delay)

    def restore(self):
        if not self.enabled:
            rospy.loginfo('系统手柄恢复节点: 已禁用，退出时不恢复系统 teleop.launch')
            return

        if self.delay > 0.0:
            time.sleep(self.delay)

        cmd = ['roslaunch', self.package, self.launch_file]
        try:
            log_dir = os.path.dirname(self.log_path)
            if log_dir and not os.path.isdir(log_dir):
                os.makedirs(log_dir)
            log_file = open(self.log_path, 'a')
            log_file.write('\n===== restore system teleop: %s =====\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
            log_file.flush()
            subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=log_file,
                close_fds=True,
                start_new_session=True)
            msg = '[成功] 已在退出时恢复系统手柄控制: roslaunch %s %s' % (
                self.package,
                self.launch_file)
            self.debug_pub.publish(String(msg))
            rospy.loginfo('系统手柄恢复节点: %s', msg)
        except Exception as exc:
            rospy.logerr('系统手柄恢复节点: 恢复系统手柄控制失败: %s', exc)


def main():
    rospy.init_node('restore_system_teleop_on_shutdown')
    SystemTeleopRestorer()
    rospy.spin()


if __name__ == '__main__':
    main()