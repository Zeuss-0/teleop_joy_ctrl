#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import termios
import time

import rospy
from std_msgs.msg import Bool, Float32, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class LightSerialController(object):
    def __init__(self):
        self.light = rospy.get_param('~light', {})
        self.enabled = bool(self.light.get('enabled', True))

        self.power_topic = self.light.get('power_topic', '/light/power_cmd')
        self.brightness_topic = self.light.get('brightness_topic', '/light/brightness_cmd')
        self.state_topic = self.light.get('serial_state_topic', '/light/serial_state')
        self.debug_topic = self.light.get('serial_debug_topic', '/light/serial_debug')

        self.port = self.light.get('serial_port', '/dev/light_port')
        self.baud_rate = int(self.light.get('baud_rate', 9600))
        self.line_ending = self.light.get('line_ending', '\n')
        self.command_delay = float(self.light.get('command_delay', 0.02))

        self.on_cmd = self.light.get('on_cmd', 'set,onall')
        self.off_cmd = self.light.get('off_cmd', 'set,offall')
        self.brightness_cmd_template = self.light.get('brightness_cmd_template', 'set,stepall,{value}')

        self.brightness_min = float(self.light.get('brightness_min', 0.0))
        self.brightness_max = float(self.light.get('brightness_max', 1.0))
        self.output_min = int(self.light.get('brightness_output_min', 0))
        self.output_max = int(self.light.get('brightness_output_max', 100))
        self.send_brightness_on_power_on = bool(self.light.get('send_brightness_on_power_on', False))
        self.send_brightness_when_off = bool(self.light.get('send_brightness_when_off', False))

        self.serial_fd = None
        self.light_on = bool(self.light.get('assume_initial_on', False))
        self.brightness = float(self.light.get('brightness_default', 0.5))
        self.debug_success_logs = bool(self.light.get('debug_success_logs', True))

        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=5, latch=True)
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=20)
        rospy.Subscriber(self.power_topic, Bool, self.power_callback, queue_size=10)
        rospy.Subscriber(self.brightness_topic, Float32, self.brightness_callback, queue_size=10)

        # if self.enabled:
        #     self.open_serial()
        # else:
        #     self.publish_state('已禁用')

        if self.enabled:
            if self.open_serial() and self.light_on:
                self.send_command(self.on_cmd)
                self.send_brightness()
        else:
            self.publish_state('已禁用')

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('光源串口节点: 开关话题=%s 亮度话题=%s 串口=%s',
                      self.power_topic, self.brightness_topic, self.port)

    def open_serial(self):
        try:
            self.serial_fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
            self.configure_serial(self.serial_fd)
        except OSError as exc:
            self.serial_fd = None
            self.publish_state('打开串口失败:%s' % exc)
            rospy.logerr('光源串口节点: 打开串口失败 %s: %s', self.port, exc)
            return False

        self.publish_state('串口已就绪')
        rospy.loginfo('光源串口节点: 串口已就绪 %s', self.port)
        return True

    def configure_serial(self, fd):
        attrs = termios.tcgetattr(fd)
        baud = self.baud_constant(self.baud_rate)

        attrs[4] = baud
        attrs[5] = baud

        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8
        attrs[2] &= ~termios.PARENB
        attrs[2] &= ~termios.CSTOPB
        if hasattr(termios, 'CRTSCTS'):
            attrs[2] &= ~termios.CRTSCTS
        attrs[2] |= termios.CREAD | termios.CLOCAL

        attrs[3] = 0
        attrs[1] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 10

        termios.tcflush(fd, termios.TCIOFLUSH)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def baud_constant(self, baud_rate):
        mapping = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
        }
        return mapping.get(baud_rate, termios.B9600)

    def power_callback(self, msg):
        self.light_on = bool(msg.data)
        if self.light_on:
            if self.send_command(self.on_cmd):
                self.log_success('光源开关指令 话题=%s 状态=开启 串口命令=%s' %
                                 (self.power_topic, self.on_cmd))
            if self.send_brightness_on_power_on:
                self.send_brightness()
        else:
            if self.send_command(self.off_cmd):
                self.log_success('光源开关指令 话题=%s 状态=关闭 串口命令=%s' %
                                 (self.power_topic, self.off_cmd))
        self.publish_state('光源=%s' % ('开启' if self.light_on else '关闭'))

    def brightness_callback(self, msg):
        self.brightness = clamp(float(msg.data), self.brightness_min, self.brightness_max)
        if self.brightness <= self.brightness_min + 1e-6:
            self.send_brightness()
        elif self.light_on or self.send_brightness_when_off:
            self.send_brightness()
        else:
            self.log_success('光源亮度已缓存 话题=%s 亮度=%.3f 未写串口原因=光源关闭' %
                             (self.brightness_topic, self.brightness))
        self.publish_state('亮度=%.3f' % self.brightness)

    def send_brightness(self):
        value = self.brightness_output_value()
        cmd = self.brightness_cmd_template.format(value=value)
        if self.send_command(cmd):
            self.log_success('光源亮度指令 话题=%s 亮度=%.3f 输出=%d 串口命令=%s' %
                             (self.brightness_topic, self.brightness, value, cmd))

    def brightness_output_value(self):
        if self.brightness_max <= self.brightness_min:
            return self.output_min
        ratio = clamp(
            (self.brightness - self.brightness_min) / (self.brightness_max - self.brightness_min),
            0.0,
            1.0)
        return int(round(self.output_min + ratio * (self.output_max - self.output_min)))

    def send_command(self, cmd):
        if not self.enabled:
            return False
        if self.serial_fd is None and not self.open_serial():
            return False

        data = (cmd + self.line_ending).encode('ascii')
        try:
            os.write(self.serial_fd, data)
            try:
                os.fsync(self.serial_fd)
            except OSError:
                pass
            time.sleep(self.command_delay)
        except OSError as exc:
            rospy.logerr('光源串口节点: 串口发送失败，命令=%s 错误=%s', cmd, exc)
            self.publish_state('发送失败:%s' % exc)
            self.close_serial()
            return False

        rospy.loginfo('光源串口节点: 串口写入成功: %s', cmd)
        return True

    def publish_state(self, event):
        text = '事件=%s 串口=%s 光源=%s 亮度=%.3f 输出=%d' % (
            event,
            self.port,
            '开启' if self.light_on else '关闭',
            self.brightness,
            self.brightness_output_value(),
        )
        self.state_pub.publish(String(text))

    def log_success(self, text):
        if not self.debug_success_logs:
            return
        msg = '[成功] %s' % text
        self.debug_pub.publish(String(msg))
        rospy.loginfo('光源串口节点: %s', msg)

    def close_serial(self):
        if self.serial_fd is not None:
            os.close(self.serial_fd)
            self.serial_fd = None

    def shutdown(self):
        self.close_serial()


def main():
    rospy.init_node('light_serial_controller_node')
    LightSerialController()
    rospy.spin()


if __name__ == '__main__':
    main()
