#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import signal
import subprocess
import time

import rospy
from std_msgs.msg import String


class ArmBringupGuard(object):
    def __init__(self):
        self.robot_ip = rospy.get_param('~robot_ip', '192.168.1.80')
        self.robot_namespace = rospy.get_param('~robot_namespace', '')
        self.service_prefix = rospy.get_param('~service_prefix', '/dobot_v4_bringup')
        self.launch_package = rospy.get_param('~launch_package', 'dobot_v4_bringup')
        self.launch_file = rospy.get_param('~launch_file', 'bringup_v4.launch')
        self.service_wait_timeout = float(rospy.get_param('~service_wait_timeout', 1.0))
        self.bringup_wait_timeout = float(rospy.get_param('~bringup_wait_timeout', 8.0))
        self.debug_topic = rospy.get_param('~debug_topic', '/teleop_joy_ctrl/debug')
        self.log_path = rospy.get_param('~log_path', '/tmp/teleop_joy_ctrl_arm_bringup_guard.log')
        self.proc = None
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=10)
        rospy.on_shutdown(self.shutdown)

        if self.services_ready(self.service_wait_timeout):
            self.log('检测到已有Dobot服务，复用当前dobot_v4_bringup，不重复拉起')
            return

        self.start_bringup()
        if self.services_ready(self.bringup_wait_timeout):
            self.log('Dobot bringup已由总控launch拉起，服务已就绪')
        else:
            rospy.logwarn('机械臂bringup守护节点: 已尝试拉起Dobot bringup，但服务仍未就绪，请检查29999/30004网络或控制器状态')

    def service_base(self):
        prefix = self.service_prefix.rstrip('/')
        namespace = self.robot_namespace.strip('/')
        if namespace:
            prefix = '%s/%s' % (prefix, namespace)
        return '%s/srv' % prefix

    def services_ready(self, timeout):
        names = [
            self.service_base() + '/EnableRobot',
            self.service_base() + '/DisableRobot',
            self.service_base() + '/ClearError',
            self.service_base() + '/RobotMode',
        ]
        deadline = time.time() + max(0.0, timeout)
        for name in names:
            remaining = max(0.05, deadline - time.time())
            try:
                rospy.wait_for_service(name, timeout=remaining)
            except rospy.ROSException:
                return False
        return True

    def start_bringup(self):
        cmd = [
            'roslaunch',
            self.launch_package,
            self.launch_file,
            'robotIp:=%s' % self.robot_ip,
            'robotName:=%s' % self.robot_namespace,
        ]
        log_dir = os.path.dirname(self.log_path)
        if log_dir and not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        log_file = open(self.log_path, 'a')
        log_file.write('\n===== start dobot bringup: %s =====\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
        log_file.write('cmd=%s\n' % ' '.join(cmd))
        log_file.flush()
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=log_file,
            stderr=log_file,
            close_fds=True,
            start_new_session=True)
        self.log('未检测到已有Dobot服务，已拉起: %s' % ' '.join(cmd))

    def log(self, text):
        msg = '[成功] %s' % text
        self.debug_pub.publish(String(msg))
        rospy.loginfo('机械臂bringup守护节点: %s', text)

    def shutdown(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except Exception:
            try:
                self.proc.terminate()
            except Exception:
                pass


def main():
    rospy.init_node('arm_bringup_guard_node')
    ArmBringupGuard()
    rospy.spin()


if __name__ == '__main__':
    main()