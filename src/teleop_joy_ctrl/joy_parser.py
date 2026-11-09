#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""手柄意图解析。

本模块不依赖 rospy，也不合成 Twist。每一帧只产出：

- 底盘意图：是否接管（active）、肩键档位（low/high）、死区后的摇杆轴
- 一条有序离散事件列表：锁定、START、模式、HOME、调参

active 为真且轴为 0 表示肩键按住、摇杆居中，调用方应继续占住速度通道。
active 为假表示释放控制权。限速、倍率和 Twist 由执行器计算。
"""

import math


ACTIVE_MODE_NONE = 'none'
ACTIVE_MODE_CHASSIS = 'chassis'
ACTIVE_MODE_CAMERA = 'camera'
ACTIVE_MODE_LIGHT = 'light'

SPEED_GEAR_NONE = 'none'
SPEED_GEAR_LOW = 'low'
SPEED_GEAR_HIGH = 'high'

EVENT_LOCK = 'lock'
EVENT_START = 'start'
EVENT_MODE = 'mode'
EVENT_HOME = 'home'
EVENT_ADJUST = 'adjust'

TARGET_LINEAR_SPEED = 'linear_speed'
TARGET_ANGULAR_SPEED = 'angular_speed'
TARGET_GAIN = 'gain'
TARGET_EXPOSURE = 'exposure'
TARGET_BRIGHTNESS = 'brightness'


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _sign(value):
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


class ChassisIntent(object):
    """归一化底盘意图。轴值只做过死区，没有乘速度上限。"""

    def __init__(self, active, speed_gear, left_x, left_y):
        self.active = bool(active)
        self.speed_gear = speed_gear
        self.left_x = float(left_x)
        self.left_y = float(left_y)


class ParseResult(object):
    def __init__(self, chassis, events):
        self.chassis = chassis
        self.events = events


class JoyParser(object):
    def __init__(
            self,
            axes,
            buttons,
            axis_deadzone=0.08,
            adjust_repeat_interval=0.25,
            left_trigger_pressed_threshold=0.95):
        self.axes = axes or {}
        self.buttons = buttons or {}
        self.axis_deadzone = float(axis_deadzone)
        self.adjust_repeat_interval = float(adjust_repeat_interval)
        self.left_trigger_pressed_threshold = float(left_trigger_pressed_threshold)

        self.locked = False
        self.active_mode = ACTIVE_MODE_NONE
        self._last_buttons = None
        self._last_adjust_time = 0.0

    def update(self, axes, buttons, now):
        axes = list(axes or [])
        buttons = list(buttons or [])
        now = float(now)
        chassis = self._chassis_intent(axes, buttons)

        if self._last_buttons is None:
            self._last_buttons = buttons
            return ParseResult(chassis, [])

        events = []

        if self._rising(buttons, 'x'):
            self.locked = not self.locked
            self.active_mode = ACTIVE_MODE_NONE
            events.append({
                'type': EVENT_LOCK,
                'locked': self.locked,
            })

        if self._rising(buttons, 'start'):
            events.append({
                'type': EVENT_START,
                'index': self._index(self.buttons, 'start'),
            })

        if not self.locked:
            if self._rising(buttons, 'a'):
                self.active_mode = ACTIVE_MODE_CHASSIS
                events.append({'type': EVENT_MODE, 'mode': self.active_mode})
            if self._rising(buttons, 'b'):
                self.active_mode = ACTIVE_MODE_CAMERA
                events.append({'type': EVENT_MODE, 'mode': self.active_mode})
            if self._rising(buttons, 'y'):
                self.active_mode = ACTIVE_MODE_LIGHT
                events.append({'type': EVENT_MODE, 'mode': self.active_mode})
            if self._rising(buttons, 'home'):
                events.append({'type': EVENT_HOME})
            self._append_adjustments(axes, now, events)

        self._last_buttons = buttons
        return ParseResult(chassis, events)

    def _chassis_intent(self, axes, buttons):
        low_held = self._held(buttons, 'left_shoulder')
        high_held = self._held(buttons, 'right_shoulder')
        if low_held and not high_held:
            active = True
            gear = SPEED_GEAR_LOW
        elif high_held:
            active = True
            gear = SPEED_GEAR_HIGH
        else:
            active = False
            gear = SPEED_GEAR_NONE
        return ChassisIntent(
            active,
            gear,
            self._apply_deadzone(self._axis(axes, 'left_x')),
            self._apply_deadzone(self._axis(axes, 'left_y')),
        )

    def _append_adjustments(self, axes, now, events):
        if not self._trigger_held(axes):
            return
        if (now - self._last_adjust_time) < self.adjust_repeat_interval:
            return

        dpad_x = -self._dpad_direction(self._axis(axes, 'dpad_x'))
        dpad_y = self._dpad_direction(self._axis(axes, 'dpad_y'))
        if dpad_x == 0 and dpad_y == 0:
            return

        if self.active_mode == ACTIVE_MODE_CHASSIS:
            if dpad_y != 0:
                events.append(self._adjust(TARGET_LINEAR_SPEED, dpad_y))
            if dpad_x != 0:
                events.append(self._adjust(TARGET_ANGULAR_SPEED, dpad_x))
        elif self.active_mode == ACTIVE_MODE_CAMERA:
            if dpad_y != 0:
                events.append(self._adjust(TARGET_GAIN, dpad_y))
            if dpad_x != 0:
                events.append(self._adjust(TARGET_EXPOSURE, dpad_x))
        elif self.active_mode == ACTIVE_MODE_LIGHT:
            if dpad_y != 0:
                events.append(self._adjust(TARGET_BRIGHTNESS, dpad_y))

        self._last_adjust_time = now

    @staticmethod
    def _adjust(target, direction):
        return {
            'type': EVENT_ADJUST,
            'target': target,
            'direction': int(direction),
        }

    def _rising(self, buttons, name):
        index = self._index(self.buttons, name)
        if index < 0 or index >= len(buttons) or index >= len(self._last_buttons):
            return False
        return buttons[index] == 1 and self._last_buttons[index] == 0

    def _held(self, buttons, name):
        index = self._index(self.buttons, name)
        if index < 0 or index >= len(buttons):
            return False
        return buttons[index] == 1

    def _axis(self, axes, name):
        index = self._index(self.axes, name)
        if index < 0 or index >= len(axes):
            return 0.0
        value = float(axes[index])
        if not math.isfinite(value):
            return 0.0
        return value

    def _trigger_held(self, axes):
        return self._axis(axes, 'left_trigger') < self.left_trigger_pressed_threshold

    def _apply_deadzone(self, value):
        if abs(value) < self.axis_deadzone:
            return 0.0
        return value

    @staticmethod
    def _dpad_direction(value):
        if abs(value) < 0.5:
            return 0
        return _sign(value)

    @staticmethod
    def _index(mapping, name):
        try:
            return int(mapping.get(name, -1))
        except (TypeError, ValueError):
            return -1
