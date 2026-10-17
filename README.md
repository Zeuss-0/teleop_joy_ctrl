# teleop_joy_ctrl

ROS1 gamepad control for the chassis, Dobot arm, camera, light, and video recording.

[中文版](README_ZH.md)

## Quick Start

Requires working base/joystick drivers, `dobot_v4_bringup` in an arm workspace, `mvs` in a camera workspace, and `teleop_joy_ctrl` plus `hybrid_video_recorder` in this package's workspace. The `~/test_ws`, `~/catkin_ws`, and `~/moma_ws` paths below are examples; replace them with the directories on the robot. Recording requires FFmpeg. See [Paths on a new machine](#paths-on-a-new-machine).

Start the camera in terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun mvs camera_stream
```

Start the controller in terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source ~/test_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/moma_ws/devel/setup.bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch
```

If Dobot bringup is already running, use `start_arm_bringup:=false` to reuse it and avoid a duplicate node:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch start_arm_bringup:=false
```

After installing or changing the packages, build `catkin_ws` first, then `moma_ws`, and source the updated workspace environments.

## Controls

| Input | Action |
| --- | --- |
| Left shoulder + left stick | Lower-speed chassis motion |
| Right shoulder + left stick | Motion at the current speed limits |
| A, then left trigger + D-pad | Up/down: linear-speed limit; right/left: angular-speed limit |
| B, then left trigger + D-pad | Up/down: gain; right/left: exposure |
| Y, then left trigger + D-pad up/down | Light brightness, including increasing from zero |
| START | Clear arm errors/collision, disable enabled states, or enable other states |
| HOME | Turn the light off or restore the configured brightness |
| X | Toggle parameter lock; motion, START, and R3 remain available |
| R3 double-click while idle | Start recording |
| R3 short-click while recording/paused | Pause/resume |
| R3 hold for at least 1.5 seconds | Save and stop recording |

Up/right increases; down/left decreases. A/B/Y selects the parameter mode; hold the left trigger while adjusting. R3 is right-stick press (`button 10`); mappings are configurable.

Manual motion publishes `/teleop_joy_ctrl/cmd_vel` while a shoulder button is held. Releasing it sends one stop command, then stops publishing; navigation can regain control after the mux timeout.

## Recording

The default backend is `hybrid_video_recorder`. Start `camera_stream` separately before recording. YOLO is optional and runs independently.

Videos are saved under the camera package recordings directory:

```text
<camera_ws>/src/mvs/scripts/recordings/<timestamp>/
```

`<camera_ws>` is the catkin workspace that contains `mvs`, for example `~/catkin_ws`.

| File | Content | During pause |
| --- | --- | --- |
| `<timestamp>.mp4` | Original camera frames | Keeps recording |
| `aimed_<timestamp>.mp4` | Original camera frames | Stops writing |

Both workers keep subscribing to `/camera/image_raw`; pause affects only aimed-video writing. Ending an R3 session leaves the separately started camera running.

To save a complete YOLO-annotated video, run this in another terminal with the camera workspace sourced:

```bash
# Detection interpreter, for example ~/miniconda3/envs/detection/bin/python
rosrun --prefix "$DETECTION_PYTHON" \
  mvs yolo_detector.py --save-yolo-video
```

YOLO recording has its own lifetime and output directory. R3 does not pause or stop it. Omit the flag for detection only.

To use the previous YOLO-coupled backend:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false start_legacy_video_manager:=true
```

Legacy mode starts camera/YOLO on demand and saves raw, YOLO, and aimed videos. With `video.adopt_existing_detector: true`, it can also attach to a manually started `yolo_detector.py --save-video` session.

## Paths on a new machine

`~/test_ws`, `~/catkin_ws`, and `~/moma_ws` in the examples are workspace locations. Point them at the arm workspace (`dobot_v4_bringup`), the camera workspace (`mvs`), and the workspace that contains this package.

Legacy recording starts `camera_stream` and `yolo_detector.py` from the camera workspace. The default hybrid recorder does not launch those processes. Each legacy path is resolved in this order:

1. Launch argument, when it is non-empty.
2. Non-empty value in `config/default.yaml` (`~` and `$VARS` are expanded).
3. Environment variable.
4. Automatic discovery.

| Path | Launch arg | YAML | Environment | Discovery |
| --- | --- | --- | --- | --- |
| Camera workspace | `camera_workspace` | `video.camera_workspace` | `TELEOP_CAMERA_WORKSPACE` | Catkin workspace that contains `rospack find mvs`, otherwise `~/catkin_ws` when `devel/setup.bash` is there |
| Detection Python | `python_executable` | `video.python_executable` | `TELEOP_DETECTION_PYTHON` | Active conda env named `detection`, otherwise `detection` under `~/miniconda3`, `~/anaconda3`, `~/miniforge3`, `~/mambaforge`, or `~/micromamba` |

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false \
  start_legacy_video_manager:=true \
  camera_workspace:=~/catkin_ws \
  python_executable:=~/miniconda3/envs/detection/bin/python
```

The same values can be exported before launch. Leave the YAML entries empty so the repository stays free of machine-specific paths.

```bash
export TELEOP_CAMERA_WORKSPACE="$HOME/catkin_ws"
export TELEOP_DETECTION_PYTHON="$HOME/miniconda3/envs/detection/bin/python"
```

## Configuration and Checks

Edit `config/default.yaml` for button mapping, speed limits, camera adjustment, light brightness, and R3 timing. Edit `hybrid_video_recorder/config/default.yaml` for the recording directory and writer settings.

Light brightness is normalized: `0.0~1.0` maps to serial `0~255` (`0.2` gives `51`). HOME sends power and brightness commands through `/dev/light_port`. Initial light state is assumed from configuration, not read from hardware. Camera defaults in the controller YAML do not alone set the camera's startup exposure/gain.

Useful checks:

```bash
rostopic echo /teleop_joy_ctrl/state
rostopic echo /teleop_joy_ctrl/video/status
rostopic echo /light/serial_debug
rostopic hz /camera/image_raw
```

If recording receives no frames, check the camera topic first. For arm failures, check Dobot TCP mode and connectivity. Terminal logs are in Chinese.

## License

Apache License 2.0. See [LICENSE](LICENSE).
