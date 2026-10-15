# teleop_joy_ctrl

ROS1 first-version joystick controller package for the robot teleoperation project.

Chinese field manual: [GAMEPAD_CONTROL_MANUAL_ZH.md](GAMEPAD_CONTROL_MANUAL_ZH.md)

## Scope

This first version builds the control framework rather than a final field-ready controller:

- Subscribe to `sensor_msgs/Joy`.
- Publish chassis velocity to `/teleop_joy_ctrl/cmd_vel` only while a shoulder button requests manual control.
- Tune chassis speed and angular-speed limits.
- Publish Hikrobot camera exposure/gain commands.
- Implement Dobot START-button state control: clear error, enable, or disable according to RobotMode.
- Publish light commands to `/light/power_cmd` and `/light/brightness_cmd`.
- Run `light_serial_controller_node.py` to convert those ROS topics into the serial protocol found in `serial-setter/led.cpp`.
- Use R3 to start, pause/resume, safely finalize, and stop three coordinated video outputs.
- Start the camera and detector only on demand through `video_recording_manager_node.py`.
- Keep lock mode for parameter controls while chassis, arm START, and R3 recording safety controls remain available.

## Default Controls

The default mapping follows a common Xbox/Logitech-style layout and must be confirmed on the real gamepad.

- Left shoulder + left stick: low-speed chassis motion.
- Right shoulder + left stick: high-speed chassis motion.
- `A`: select chassis speed-limit adjust mode.
- `B`: select camera exposure/gain adjust mode.
- `Y`: select light brightness adjust mode.
- Left trigger + D-pad in `A` mode: up/down adjusts max linear speed, left/right adjusts max angular speed.
- Left trigger + D-pad in `B` mode: up/down adjusts gain, left/right adjusts exposure.
- Left trigger + D-pad in `Y` mode: up/down adjusts brightness.
- `START`: arm state action. Error/collision clears error, enabled states disable, other states enable.
- `HOME`: toggle light power through `/dev/light_port`.
- `X`: toggle parameter lock mode; chassis, arm START, and R3 recording controls remain available.
- `R3` (`button 10`): double-click while idle to start recording, short-click to pause/resume, and hold for `1.5 s` to save and stop.

## Build and Launch

```bash
source /opt/ros/noetic/setup.bash
source ~/test_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/moma_ws
catkin_make
source devel/setup.bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch
```

`test_ws` supplies `dobot_v4_bringup`, while `catkin_ws` supplies the MVS
camera and detector package. Source `moma_ws` last so this package takes
precedence. The default arm address is `192.168.1.80`; override it when needed:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch robot_ip:=192.168.1.80
```

The current field robot uses `/joy_teleop/joy` for joystick input and `/teleop_joy_ctrl/cmd_vel` for high-priority manual commands. If a test environment uses plain `/joy` and `/cmd_vel`:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch joy_topic:=/joy cmd_vel_topic:=/cmd_vel
```

## Navigation Coexistence

The takeover `twist_mux` gives `/teleop_joy_ctrl/cmd_vel` priority over navigation, but the controller claims that input only while either shoulder button is held. Releasing the shoulder button or losing joystick input sends one zero-speed command, then stops publishing. After the mux input timeout (default `0.3 s`), `/platform_control/cmd_vel` can control the chassis again.

## Light

The field `led.cpp` utility opens `/dev/light_port` at 9600 baud and sends plain text commands with a newline:

```text
set,onall
set,offall
set,stepall,50
```

This package now uses the same protocol. The actual hardware write only happens when HOME toggles the light or Y-mode brightness changes; startup publishes the ROS mirror state but does not send an off command to the serial port.

The serial protocol is wrapped by `light_serial_controller_node.py`:

- subscribes `/light/power_cmd` (`std_msgs/Bool`)
- subscribes `/light/brightness_cmd` (`std_msgs/Float32`)
- publishes `/light/serial_state` (`std_msgs/String`)
- sends `set,onall`, `set,offall`, and `set,stepall,{value}` to `/dev/light_port`

Disable the light node when testing without the device:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch start_light_controller:=false
```

## Hybrid Recording (Default)

The default R3 backend is `hybrid_video_recorder`. It contains two independent
workers that continuously subscribe to `/camera/image_raw`:

- `raw` writes `<timestamp>.mp4` during both recording and pause.
- `aimed` writes `aimed_<timestamp>.mp4` only during recording.

The workers have separate FFmpeg processes and queues. Failure of `aimed`, the
YOLO detector, or the detector GUI does not cancel the raw-data recording.
The hybrid manager never starts or stops `camera_stream`; start it separately
before R3 begins a session:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun mvs camera_stream
```

YOLO is an independent optional process in this mode. It is not allowed to
block, stop, or mark the raw/aimed data session as failed. To use the previous
YOLO-coupled backend temporarily, launch with:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false \
  start_legacy_video_manager:=true
```

## Legacy YOLO-Coupled Recording

The manager exposes these `std_srvs/Trigger` services:

```text
/teleop_joy_ctrl/video/start
/teleop_joy_ctrl/video/pause
/teleop_joy_ctrl/video/resume
/teleop_joy_ctrl/video/stop
```

It publishes latched state, status, and output path on:

```text
/teleop_joy_ctrl/video/state
/teleop_joy_ctrl/video/status
/teleop_joy_ctrl/video/output_dir
```

Each session produces:

```text
/home/shansu/catkin_ws/src/mvs/scripts/recordings/20260723_153012/
  20260723_153012.mp4
  yolo.mp4
  aimed_20260723_153012.mp4
```

The timestamp-named raw video and `yolo.mp4` are written continuously from
recording start to finish. `aimed_<timestamp>.mp4` contains raw frames only during the
active recording intervals, so R3 pause/resume affects only that third video.
R3 recording remains active while X lock is enabled. In automatic mode, a long
press first asks the YOLO node to drain its frame queue and finalize all three
MP4 files, then closes the detector and camera process groups owned by the
manager.

Deploy `teleop_joy_ctrl` to `/home/shansu/moma_ws/src/teleop_joy_ctrl` and the
modified MVS files to `/home/shansu/catkin_ws/src/mvs`. Build `catkin_ws` before
`moma_ws`; do not place a duplicate `mvs` package in `moma_ws`.

## Required Follow-Up

- Confirm the real `axes` and `buttons` mapping from `rostopic echo /joy`.
- Confirm final chassis command topic and whether a `cmd_vel` mux is required.
- Verify the light brightness range. The current command maps `0.0~1.0` to `set,stepall,0~255`.
- Confirm speed, exposure, gain, and brightness step sizes for field use.

## Runtime Diagnostics

The nodes publish status and debug information on:

- `/teleop_joy_ctrl/debug`: joystick integration layer
- `/teleop_joy_ctrl/state`: current mode and parameter snapshot
- `/light/serial_debug`: light serial write layer
- `/light/serial_state`: light serial state snapshot

Useful checks:

```bash
rostopic echo /teleop_joy_ctrl/debug
rostopic echo /teleop_joy_ctrl/state
rostopic echo /light/serial_debug
rostopic echo /light/serial_state
```

Runtime logs are intentionally Chinese so field operators can read state
transitions, command results, and failure causes directly in the terminal.

## Video Manager Modes

The launch file starts `video_recording_manager_node.py` by default, but it does
not start `camera_stream` or `yolo_detector.py` until R3 is double-clicked.
Disable only the manager with:

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch start_video_manager:=false
```

With the default automatic mode, the manager owns the camera and YOLO processes.
It starts them on demand, forwards pause/resume/stop requests, and shuts them
down after a completed recording. `video.detector_headless` defaults to `false`
for NoMachine or local GUI use. Set it to `true` when no graphical display is
available.

If automatic detector startup fails, manually start the two MVS nodes in
separate terminals, then let the manager attach to them:

```bash
# Terminal 1
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun mvs camera_stream

# Terminal 2
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun --prefix "/home/shansu/miniconda3/envs/detection/bin/python" mvs yolo_detector.py --save-video
```

Keep `video.adopt_existing_detector: true` in `config/default.yaml`. On the next
R3 double-click, the manager detects the ready camera and detector services and
attaches to that existing session. R3 pause/resume/stop remains available, but
the manager does not terminate manually started processes. After recording is
finished, close them explicitly:

```bash
rosservice call /yolo_detector_node/recording/shutdown
rosnode kill /camera_stream_node
```
