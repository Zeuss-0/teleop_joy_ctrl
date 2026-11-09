# teleop_joy_ctrl

ROS1 gamepad control for the chassis, Dobot arm, camera exposure and gain, and the light.

[中文版](README_ZH.md)

## Quick Start

Requires working base and joystick drivers, and `dobot_v4_bringup` in an arm workspace. Exposure and gain commands are published for a separately running camera node such as `mvs`. This package does not launch or depend on video recording. The `~/test_ws`, `~/catkin_ws`, and `~/moma_ws` paths below are examples; replace them with the directories on the robot. See [Paths on a new machine](#paths-on-a-new-machine).

Start the camera in terminal 1 when exposure and gain adjustments should reach the camera:

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

The same-process node is the default. Set `legacy_single_node:=false` to run the intent publisher and the executor as two nodes.

After installing or changing the packages, build the arm workspace first, then the workspace that contains this package, and source the updated environments.

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
| X | Toggle parameter lock; motion and START remain available |
| Right-stick click (R3) | Not bound |

Up/right increases; down/left decreases. A/B/Y selects the parameter mode; hold the left trigger while adjusting. Mappings are configurable. Right-stick click is not bound.

Manual motion publishes `/teleop_joy_ctrl/cmd_vel` while a shoulder button is held. Releasing it sends one stop command, then stops publishing; navigation can regain control after the mux timeout.

## Paths on a new machine

`~/test_ws`, `~/catkin_ws`, and `~/moma_ws` in the examples are workspace locations. Point them at the arm workspace (`dobot_v4_bringup`), the camera workspace (`mvs`, when the camera node should receive exposure and gain commands), and the workspace that contains this package.

## Configuration and Checks

Edit `config/default.yaml` for button mapping, speed limits, camera adjustment, and light brightness.

Light brightness is normalized: `0.0~1.0` maps to serial `0~255` (`0.2` gives `51`). HOME sends power and brightness commands through `/dev/light_port`. Initial light state is assumed from configuration, not read from hardware. Camera defaults in the controller YAML do not alone set the camera's startup exposure or gain.

Useful checks:

```bash
rostopic echo /teleop_joy_ctrl/state
rostopic echo /camera/exposure_cmd
rostopic echo /camera/gain_cmd
rostopic echo /light/serial_debug
rostopic hz /camera/image_raw
```

If exposure or gain commands have no effect, check that the camera node is running and subscribed to those topics. For arm failures, check Dobot TCP mode and connectivity. Terminal logs are in Chinese.

## License

Apache License 2.0. See [LICENSE](LICENSE).
