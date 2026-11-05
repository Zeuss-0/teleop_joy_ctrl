#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""意图话题的 JSON 编码。

不依赖 rospy，也不生成速度指令。两个节点和以后的网页控制端共用这里的字段。

底盘话题 ``/teleop_joy_ctrl/chassis_axes``（std_msgs/String，不锁存）::

    {"active": true, "speed_gear": "high", "gear": "high",
     "left_x": 0.0, "left_y": 1.0,
     "axes": {"left_x": 0.0, "left_y": 1.0}, "time": 12.5}

轴只做过死区，没有乘速度上限。``gear`` 与 ``speed_gear`` 相同；解码时
``speed_gear`` 优先。执行器收到后再把轴夹到 [-1, 1]，并用本机接收时间
判断超时；报文里的 ``time`` 不能把存活时间推到未来。消息不锁存。

离散事件话题 ``/teleop_joy_ctrl/events``（std_msgs/String，不锁存）。
一帧里的事件放在同一条消息的 ``events`` 数组里，顺序与解析器一致
（R3、锁定、START、模式、HOME、调参）。报头带 ``session``、``seq`` 和
``time``。意图节点每次启动使用新的 ``session``，``seq`` 从 1 递增::

    {"time": 12.5, "seq": 3, "session": "intent-session", "events": [
        {"type": "adjust", "target": "exposure", "direction": 1}
    ]}

纯 R3 idle 且没有其他事件时可以不发；press/hold/release/sync 必须发，
长按结束依赖按住期间的 hold 采样。
"""

import json

from teleop_joy_ctrl.joy_parser import ChassisIntent


DEFAULT_CHASSIS_AXES_TOPIC = '/teleop_joy_ctrl/chassis_axes'
DEFAULT_EVENTS_TOPIC = '/teleop_joy_ctrl/events'


class EventMessage(object):
    """一条离散事件报文。``events`` 保持发布顺序。"""

    def __init__(self, events, stamp, seq, session):
        self.events = events
        self.stamp = stamp
        self.seq = seq
        self.session = session

_JSON_KWARGS = {
    'ensure_ascii': True,
    'separators': (',', ':'),
    'sort_keys': True,
    'allow_nan': False,
}


def encode_chassis(intent, now):
    """把归一化底盘意图编成一条 JSON 字符串。"""
    left_x = float(intent.left_x)
    left_y = float(intent.left_y)
    gear = str(intent.speed_gear)
    payload = {
        'active': bool(intent.active),
        'speed_gear': gear,
        'gear': gear,
        'left_x': left_x,
        'left_y': left_y,
        'axes': {
            'left_x': left_x,
            'left_y': left_y,
        },
        'time': float(now),
    }
    return json.dumps(payload, **_JSON_KWARGS)


def decode_chassis(text):
    """解析底盘意图。成功时返回 ``(ChassisIntent, stamp_or_None)``。"""
    payload = _load_object(text)
    if not isinstance(payload, dict) or 'active' not in payload:
        return None
    if 'speed_gear' in payload and payload['speed_gear'] is not None:
        gear = payload['speed_gear']
    elif 'gear' in payload and payload['gear'] is not None:
        gear = payload['gear']
    else:
        gear = 'none'
    try:
        intent = ChassisIntent(
            _as_bool(payload.get('active')),
            str(gear),
            float(_axis_value(payload, 'left_x')),
            float(_axis_value(payload, 'left_y')),
        )
    except (TypeError, ValueError):
        return None
    if 'time' not in payload or payload['time'] is None:
        return intent, None
    try:
        return intent, float(payload['time'])
    except (TypeError, ValueError):
        return None


def encode_events(events, now, seq=None, session=None):
    """把同一帧的离散事件按原顺序编成一条 JSON 字符串。"""
    payload = {
        'time': float(now),
        'events': [dict(event) for event in events],
    }
    if seq is not None:
        payload['seq'] = int(seq)
    if session is not None:
        payload['session'] = str(session)
    return json.dumps(payload, **_JSON_KWARGS)


def decode_events(text):
    """解析离散事件。

    返回 ``EventMessage``。结构无法识别时返回 None。
    接受带 ``events`` 的报头、单条 ``{"type": ...}``，以及数组。
    缺报头时 ``seq`` / ``session`` / ``stamp`` 为 None，执行器会拒绝执行。
    """
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    stamp = None
    seq = None
    session = None
    if isinstance(payload, dict) and isinstance(payload.get('events'), list):
        events = payload['events']
        stamp = payload.get('time')
        seq = payload.get('seq')
        session = payload.get('session')
    elif isinstance(payload, dict) and 'type' in payload:
        events = [payload]
    elif isinstance(payload, list):
        events = payload
    else:
        return None
    cleaned = []
    for event in events:
        if isinstance(event, dict) and isinstance(event.get('type'), str):
            cleaned.append(event)
    return EventMessage(cleaned, stamp, seq, session)


def frame_has_discrete_content(events):
    """纯 R3 idle 不算需要发布的离散帧。"""
    for event in events or []:
        if not isinstance(event, dict):
            continue
        if event.get('type') != 'r3':
            return True
        if event.get('phase') != 'idle':
            return True
    return False


def events_message(events, now, seq, session=None):
    """需要发布时返回 JSON，否则返回 None。"""
    if not frame_has_discrete_content(events):
        return None
    return encode_events(events, now, seq=seq, session=session)


def _axis_value(payload, name):
    if name in payload and payload[name] is not None:
        return payload[name]
    axes = payload.get('axes')
    if isinstance(axes, dict) and name in axes and axes[name] is not None:
        return axes[name]
    return 0.0


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _load_object(text):
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return None
