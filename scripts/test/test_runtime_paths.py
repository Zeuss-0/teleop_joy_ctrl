#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import stat
import tempfile
import unittest
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from teleop_joy_ctrl.runtime_paths import (  # noqa: E402
    resolve_camera_workspace,
    resolve_python_executable,
    ros_setup_bash,
)


class RuntimePathsTest(unittest.TestCase):
    def test_configured_workspace_wins_over_env_and_discovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            configured = self.write_workspace(tmp, "configured")
            env_workspace = self.write_workspace(tmp, "from-env")
            discovered = self.write_workspace(tmp, "discovered")
            home = self.write_workspace(tmp, "home")
            resolved = resolve_camera_workspace(
                configured,
                environ={
                    "HOME": home,
                    "TELEOP_CAMERA_WORKSPACE": env_workspace,
                },
                mvs_package_path=os.path.join(discovered, "src", "mvs"),
                home=home,
            )
            self.assertEqual(os.path.abspath(configured), resolved)

    def test_blank_configured_value_uses_environment(self):
        with tempfile.TemporaryDirectory() as tmp:
            env_workspace = self.write_workspace(tmp, "from-env")
            resolved = resolve_camera_workspace(
                "  ",
                environ={"TELEOP_CAMERA_WORKSPACE": env_workspace, "HOME": tmp},
                mvs_package_path="",
                home=tmp,
            )
            self.assertEqual(os.path.abspath(env_workspace), resolved)

    def test_mvs_package_discovers_catkin_workspace(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = self.write_workspace(tmp, "camera")
            resolved = resolve_camera_workspace(
                "",
                environ={"HOME": os.path.join(tmp, "missing-home")},
                mvs_package_path=os.path.join(workspace, "src", "mvs"),
                home=os.path.join(tmp, "missing-home"),
            )
            self.assertEqual(os.path.abspath(workspace), resolved)

    def test_home_catkin_ws_is_the_last_resort(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = os.path.join(tmp, "robot")
            workspace = self.write_workspace(home, "catkin_ws")
            resolved = resolve_camera_workspace(
                None,
                environ={"HOME": home},
                mvs_package_path="",
                home=home,
            )
            self.assertEqual(os.path.abspath(workspace), resolved)

    def test_unresolved_workspace_is_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            resolved = resolve_camera_workspace(
                "",
                environ={"HOME": tmp},
                mvs_package_path=os.path.join(tmp, "src", "mvs"),
                home=tmp,
            )
            self.assertEqual("", resolved)

    def test_workspace_expands_tilde_and_environment_variables(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = self.write_workspace(tmp, "camera")
            resolved = resolve_camera_workspace(
                "~/camera",
                environ={"HOME": tmp, "CAMERA_ROOT": workspace},
            )
            self.assertEqual(os.path.abspath(os.path.join(tmp, "camera")), resolved)

            resolved = resolve_camera_workspace(
                "${CAMERA_ROOT}",
                environ={"HOME": tmp, "CAMERA_ROOT": workspace},
            )
            self.assertEqual(os.path.abspath(workspace), resolved)

    def test_explicit_python_is_not_replaced_when_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            discovered = self.write_executable(
                os.path.join(tmp, "miniconda3", "envs", "detection", "bin", "python")
            )
            resolved = resolve_python_executable(
                os.path.join(tmp, "missing-python"),
                environ={"HOME": tmp, "PATH": ""},
                home=tmp,
            )
            self.assertEqual(os.path.join(tmp, "missing-python"), resolved)
            self.assertNotEqual(discovered, resolved)

    def test_python_environment_wins_over_conda_discovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            configured = self.write_executable(os.path.join(tmp, "env-python"))
            conda_python = self.write_executable(
                os.path.join(tmp, "detection", "bin", "python")
            )
            resolved = resolve_python_executable(
                "",
                environ={
                    "HOME": tmp,
                    "PATH": "",
                    "TELEOP_DETECTION_PYTHON": configured,
                    "CONDA_PREFIX": os.path.join(tmp, "detection"),
                    "CONDA_DEFAULT_ENV": "detection",
                },
                home=tmp,
            )
            self.assertEqual(os.path.abspath(configured), resolved)
            self.assertNotEqual(os.path.abspath(conda_python), resolved)

    def test_active_detection_conda_env(self):
        with tempfile.TemporaryDirectory() as tmp:
            conda_python = self.write_executable(
                os.path.join(tmp, "detection", "bin", "python")
            )
            resolved = resolve_python_executable(
                "",
                environ={
                    "HOME": os.path.join(tmp, "empty-home"),
                    "PATH": "",
                    "CONDA_PREFIX": os.path.join(tmp, "detection"),
                    "CONDA_DEFAULT_ENV": "detection",
                },
                home=os.path.join(tmp, "empty-home"),
            )
            self.assertEqual(os.path.abspath(conda_python), resolved)

    def test_other_conda_env_is_not_used(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.write_executable(os.path.join(tmp, "base", "bin", "python"))
            resolved = resolve_python_executable(
                "",
                environ={
                    "HOME": os.path.join(tmp, "empty-home"),
                    "PATH": "",
                    "CONDA_PREFIX": os.path.join(tmp, "base"),
                    "CONDA_DEFAULT_ENV": "base",
                },
                home=os.path.join(tmp, "empty-home"),
            )
            self.assertEqual("", resolved)

    def test_home_miniconda_detection_env(self):
        with tempfile.TemporaryDirectory() as tmp:
            python_path = self.write_executable(
                os.path.join(tmp, "miniconda3", "envs", "detection", "bin", "python")
            )
            resolved = resolve_python_executable(
                None,
                environ={"HOME": tmp, "PATH": ""},
                home=tmp,
            )
            self.assertEqual(os.path.abspath(python_path), resolved)

    def test_command_name_resolves_from_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = self.write_executable(os.path.join(tmp, "detection-python"))
            resolved = resolve_python_executable(
                "detection-python",
                environ={"HOME": tmp, "PATH": tmp},
                home=tmp,
            )
            self.assertEqual(os.path.abspath(binary), resolved)

    def test_ros_setup_defaults_to_noetic(self):
        self.assertEqual(
            "/opt/ros/noetic/setup.bash",
            ros_setup_bash(environ={}),
        )
        self.assertEqual(
            "/opt/ros/noetic/setup.bash",
            ros_setup_bash(environ={"ROS_DISTRO": "noetic"}),
        )

    def test_repository_has_no_machine_specific_paths(self):
        root = Path(__file__).resolve().parents[2]
        banned = ("/home/" + "shansu", "todo@" + "example.com")
        offenders = []
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if any(part in {".git", "__pycache__"} for part in path.parts):
                continue
            if path.suffix in {".pyc", ".png", ".jpg"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if any(token in text for token in banned):
                offenders.append(str(path.relative_to(root)))
        self.assertEqual([], offenders)

    def write_workspace(self, root, name):
        workspace = os.path.join(root, name)
        os.makedirs(os.path.join(workspace, "devel"))
        setup_file = os.path.join(workspace, "devel", "setup.bash")
        with open(setup_file, "w") as handle:
            handle.write("# test setup\n")
        return workspace

    def write_executable(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as handle:
            handle.write("#!/bin/sh\n")
        os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)
        return path


if __name__ == "__main__":
    unittest.main()
