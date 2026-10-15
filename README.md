# teleop_joy_ctrl

ROS1 gamepad control for the chassis, Dobot arm, camera, light, and video recording.

[中文版](README_ZH.md)

## Quick Start

Requires working base/joystick drivers, `dobot_v4_bringup` in `test_ws`, `mvs` in `catkin_ws`, and `teleop_joy_ctrl` plus `hybrid_video_recorder` in `moma_ws`. Recording requires FFmpeg.

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

Videos are saved under:

```text
/home/shansu/catkin_ws/src/mvs/scripts/recordings/<timestamp>/
```

| File | Content | During pause |
| --- | --- | --- |
| `<timestamp>.mp4` | Original camera frames | Keeps recording |
| `aimed_<timestamp>.mp4` | Original camera frames | Stops writing |

Both workers keep subscribing to `/camera/image_raw`; pause affects only aimed-video writing. Ending an R3 session leaves the separately started camera running.

To save a complete YOLO-annotated video, run this in another terminal with the camera workspace sourced:

```bash
rosrun --prefix "/home/shansu/miniconda3/envs/detection/bin/python" \
  mvs yolo_detector.py --save-yolo-video
```

YOLO recording has its own lifetime and output directory. R3 does not pause or stop it. Omit the flag for detection only.

To use the previous YOLO-coupled backend:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false start_legacy_video_manager:=true
```

Legacy mode starts camera/YOLO on demand and saves raw, YOLO, and aimed videos. With `video.adopt_existing_detector: true`, it can also attach to a manually started `yolo_detector.py --save-video` session.

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
