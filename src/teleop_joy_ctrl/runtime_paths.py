#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Resolve camera-workspace and detection-Python paths without a fixed home directory.

Precedence for each value:
1. An explicit configured value (launch arg or YAML), when non-empty.
2. The matching environment variable.
3. Local discovery (the sourced ``mvs`` workspace, ``~/catkin_ws``, or a
   conda env named ``detection``).
"""

import os
import re
import shutil


CAMERA_WORKSPACE_ENV = "TELEOP_CAMERA_WORKSPACE"
DETECTION_PYTHON_ENV = "TELEOP_DETECTION_PYTHON"

CAMERA_WORKSPACE_HELP = (
    "未配置相机工作空间。请设置 ROS 参数 video/camera_workspace、"
    "launch 参数 camera_workspace:=<工作空间>，或环境变量 %s。"
    % CAMERA_WORKSPACE_ENV
)
DETECTION_PYTHON_HELP = (
    "未配置检测环境 Python。请设置 ROS 参数 video/python_executable、"
    "launch 参数 python_executable:=<解释器>，或环境变量 %s。"
    % DETECTION_PYTHON_ENV
)

_ENV_VAR_PATTERN = re.compile(r"\$\{([^}]+)\}|\$([A-Za-z_][A-Za-z0-9_]*)")
_CONDA_ENV_RELATIVE_PATHS = (
    os.path.join("miniconda3", "envs", "detection", "bin", "python"),
    os.path.join("anaconda3", "envs", "detection", "bin", "python"),
    os.path.join("miniforge3", "envs", "detection", "bin", "python"),
    os.path.join("mambaforge", "envs", "detection", "bin", "python"),
    os.path.join("micromamba", "envs", "detection", "bin", "python"),
)


def ros_setup_bash(environ=None):
    """Return the ROS setup file for the active distro, defaulting to Noetic."""
    environ = os.environ if environ is None else environ
    distro = str(environ.get("ROS_DISTRO", "") or "").strip() or "noetic"
    return os.path.join("/opt/ros", distro, "setup.bash")


def resolve_camera_workspace(configured, environ=None, mvs_package_path=None, home=None):
    """Return a camera workspace path, or ``""`` when nothing can be resolved."""
    environ = os.environ if environ is None else environ
    configured_path = normalize_directory(configured, environ)
    if configured_path:
        return configured_path

    env_path = normalize_directory(environ.get(CAMERA_WORKSPACE_ENV, ""), environ)
    if env_path:
        return env_path

    discovered = workspace_from_package(mvs_package_path)
    if discovered:
        return discovered

    if home is None:
        home = environ.get("HOME", "")
    conventional = os.path.join(home, "catkin_ws") if home else ""
    setup_file = os.path.join(conventional, "devel", "setup.bash") if conventional else ""
    if setup_file and os.path.isfile(setup_file):
        return os.path.abspath(conventional)
    return ""


def resolve_python_executable(configured, environ=None, home=None):
    """Return a detection interpreter path, or ``""`` when nothing can be resolved.

    An explicit configured value or environment variable is returned even when
    the file is missing, so the caller can report that exact path.
    """
    environ = os.environ if environ is None else environ
    if _as_text(configured):
        return normalize_executable(configured, environ)

    if _as_text(environ.get(DETECTION_PYTHON_ENV, "")):
        return normalize_executable(environ.get(DETECTION_PYTHON_ENV, ""), environ)

    conda_prefix = _as_text(environ.get("CONDA_PREFIX", ""))
    conda_env = _as_text(environ.get("CONDA_DEFAULT_ENV", ""))
    if conda_prefix and conda_env == "detection":
        candidate = os.path.join(conda_prefix, "bin", "python")
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)

    if home is None:
        home = environ.get("HOME", "")
    if home:
        for relative in _CONDA_ENV_RELATIVE_PATHS:
            candidate = os.path.join(home, relative)
            if os.path.isfile(candidate):
                return os.path.abspath(candidate)
    return ""


def workspace_from_package(package_path):
    """Walk parents of a ROS package until a catkin ``devel/setup.bash`` exists."""
    if not _as_text(package_path):
        return ""
    current = os.path.realpath(_as_text(package_path))
    for _ in range(8):
        parent = os.path.dirname(current)
        if parent == current:
            return ""
        current = parent
        setup_file = os.path.join(current, "devel", "setup.bash")
        if os.path.isfile(setup_file):
            return current
    return ""


def normalize_directory(value, environ):
    text = expand_text(value, environ)
    if not text:
        return ""
    return os.path.abspath(text)


def normalize_executable(value, environ):
    text = expand_text(value, environ)
    if not text:
        return ""
    if os.path.isfile(text):
        return os.path.abspath(text)
    found = shutil.which(text, path=environ.get("PATH"))
    if found:
        return os.path.abspath(found)
    return text


def expand_text(value, environ):
    text = _as_text(value)
    if not text:
        return ""
    home = environ.get("HOME", "")
    if home and (text == "~" or text.startswith("~/") or text.startswith("~" + os.sep)):
        text = home + text[1:]

    def replace(match):
        name = match.group(1) or match.group(2)
        return environ.get(name, match.group(0))

    return _ENV_VAR_PATTERN.sub(replace, text)


def _as_text(value):
    if value is None:
        return ""
    return str(value).strip()
