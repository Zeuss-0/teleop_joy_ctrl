# teleop_joy_ctrl

ROS1 手柄总控功能包，用于底盘运动、Dobot 机械臂、相机参数、光源和录像控制。

[English](README.md)

## 快速启动

需要底盘和手柄驱动已运行，`test_ws` 中有 `dobot_v4_bringup`，`catkin_ws` 中有 `mvs`，`moma_ws` 中有 `teleop_joy_ctrl` 和 `hybrid_video_recorder`。录像需要安装 FFmpeg。

终端 1：启动相机。

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun mvs camera_stream
```

终端 2：启动手柄总控。

```bash
source /opt/ros/noetic/setup.bash
source ~/test_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/moma_ws/devel/setup.bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch
```

如果 Dobot 驱动已经启动，加上 `start_arm_bringup:=false`，复用现有机械臂服务，避免重复拉起节点：

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch start_arm_bringup:=false
```

首次安装或修改功能包后，先编译 `catkin_ws`，再编译 `moma_ws`，并重新加载工作空间环境。

## 手柄操作

| 按键 | 功能 |
| --- | --- |
| 左肩键 + 左摇杆 | 较低速度控制底盘 |
| 右肩键 + 左摇杆 | 按当前速度上限控制底盘 |
| A，再按住左扳机 + 方向键 | 上/下调线速度上限；右/左调角速度上限 |
| B，再按住左扳机 + 方向键 | 上/下调增益；右/左调曝光 |
| Y，再按住左扳机 + 方向上/下 | 调光源亮度；亮度为零时也能上调 |
| START | 按机械臂状态清错、下使能或上使能 |
| HOME | 关闭光源，或恢复配置的亮度 |
| X | 切换参数锁定；底盘运动、START 和 R3 仍可用 |
| 空闲时双击 R3 | 开始录像 |
| 录像或暂停时短按 R3 | 暂停/继续 |
| 长按 R3 至少 1.5 秒 | 保存并结束录像 |

方向上/右增加，下/左减少。A/B/Y 用于选择调节模式，调节时需按住左扳机。R3 是右摇杆按下，对应 `button 10`；映射可在配置中修改。

按住肩键时，总控发布 `/teleop_joy_ctrl/cmd_vel`。松开后发送一次零速度并停止发布，导航在 mux 超时后可重新接管底盘。

## 录像说明

默认使用 `hybrid_video_recorder`，录像前需单独启动 `camera_stream`。YOLO 为独立的可选节点。

保存目录：

```text
/home/shansu/catkin_ws/src/mvs/scripts/recordings/<时间戳>/
```

| 文件 | 内容 | 暂停时 |
| --- | --- | --- |
| `<时间戳>.mp4` | 原始相机画面 | 持续录制 |
| `aimed_<时间戳>.mp4` | 原始相机画面 | 停止写入 |

两个录像节点始终订阅 `/camera/image_raw`；暂停只影响 aimed 视频写入。长按 R3 结束录像后，单独启动的相机继续运行。

如需保存完整的 YOLO 检测视频，在另一个已加载相机工作空间的终端执行：

```bash
rosrun --prefix "/home/shansu/miniconda3/envs/detection/bin/python" \
  mvs yolo_detector.py --save-yolo-video
```

YOLO 视频使用独立的录制时间和输出目录，R3 不控制它的暂停或结束。只需实时检测时，去掉保存参数。

切换到旧版 YOLO 耦合录像：

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false start_legacy_video_manager:=true
```

旧版按需启动相机和 YOLO，保存 raw、YOLO、aimed 三个视频。配置 `video.adopt_existing_detector: true` 时，也可接入手动启动的 `yolo_detector.py --save-video` 录像会话。

## 配置与排查

`config/default.yaml` 配置按键、速度上限、相机调节、光源亮度和 R3 手势时间；`hybrid_video_recorder/config/default.yaml` 配置录像目录及写入参数。

光源亮度 `0.0~1.0` 映射到串口 `0~255`，例如 `0.2` 对应 `51`。HOME 通过 `/dev/light_port` 下发开关与亮度指令。启动光源状态按配置假定，未读取硬件状态。总控 YAML 中的相机默认值不会单独设置相机启动时的曝光和增益。

常用检查命令：

```bash
rostopic echo /teleop_joy_ctrl/state
rostopic echo /teleop_joy_ctrl/video/status
rostopic echo /light/serial_debug
rostopic hz /camera/image_raw
```

录像没有画面时，先检查相机话题是否持续出帧；机械臂控制失败时，检查 TCP 模式及网络连接。终端日志使用中文。
