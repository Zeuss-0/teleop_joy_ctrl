# teleop_joy_ctrl

ROS1 手柄总控功能包，用于底盘运动、Dobot 机械臂、相机参数、光源和录像控制。

[English](README.md)

## 快速启动

需要底盘和手柄驱动已运行，机械臂工作空间中有 `dobot_v4_bringup`，相机工作空间中有 `mvs`，本功能包所在工作空间中有 `teleop_joy_ctrl` 和 `hybrid_video_recorder`。下文的 `~/test_ws`、`~/catkin_ws`、`~/moma_ws` 是示例路径，换机器时改成现场目录。录像需要安装 FFmpeg。路径覆盖方式见[在新机器上配置路径](#在新机器上配置路径)。

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

保存目录为相机功能包下的录像目录：

```text
<camera_ws>/src/mvs/scripts/recordings/<时间戳>/
```

`<camera_ws>` 是包含 `mvs` 的 catkin 工作空间，例如 `~/catkin_ws`。

| 文件 | 内容 | 暂停时 |
| --- | --- | --- |
| `<时间戳>.mp4` | 原始相机画面 | 持续录制 |
| `aimed_<时间戳>.mp4` | 原始相机画面 | 停止写入 |

两个录像节点始终订阅 `/camera/image_raw`；暂停只影响 aimed 视频写入。长按 R3 结束录像后，单独启动的相机继续运行。

如需保存完整的 YOLO 检测视频，在另一个已加载相机工作空间的终端执行：

```bash
# 检测环境解释器，例如 ~/miniconda3/envs/detection/bin/python
rosrun --prefix "$DETECTION_PYTHON" \
  mvs yolo_detector.py --save-yolo-video
```

YOLO 视频使用独立的录制时间和输出目录，R3 不控制它的暂停或结束。只需实时检测时，去掉保存参数。

切换到旧版 YOLO 耦合录像：

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false start_legacy_video_manager:=true
```

旧版按需启动相机和 YOLO，保存 raw、YOLO、aimed 三个视频。配置 `video.adopt_existing_detector: true` 时，也可接入手动启动的 `yolo_detector.py --save-video` 录像会话。

## 在新机器上配置路径

示例中的 `~/test_ws`、`~/catkin_ws`、`~/moma_ws` 分别对应机械臂工作空间（`dobot_v4_bringup`）、相机工作空间（`mvs`）和本功能包所在工作空间。换到新机器时改成实际目录。

旧版录像会从相机工作空间启动 `camera_stream` 和 `yolo_detector.py`。默认的混合录像器不会拉起这两个进程。每条旧版路径按以下顺序解析：

1. 非空的 launch 参数。
2. `config/default.yaml` 中的非空值（展开 `~` 和环境变量）。
3. 环境变量。
4. 自动发现。

| 路径 | Launch 参数 | YAML | 环境变量 | 自动发现 |
| --- | --- | --- | --- | --- |
| 相机工作空间 | `camera_workspace` | `video.camera_workspace` | `TELEOP_CAMERA_WORKSPACE` | `rospack find mvs` 所在 catkin 工作空间；否则在存在 `devel/setup.bash` 时使用 `~/catkin_ws` |
| 检测 Python | `python_executable` | `video.python_executable` | `TELEOP_DETECTION_PYTHON` | 当前名为 `detection` 的 conda 环境；否则查找 `~/miniconda3`、`~/anaconda3`、`~/miniforge3`、`~/mambaforge` 或 `~/micromamba` 下的 `detection` 环境 |

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false \
  start_legacy_video_manager:=true \
  camera_workspace:=~/catkin_ws \
  python_executable:=~/miniconda3/envs/detection/bin/python
```

也可以在启动前导出同样的值。YAML 中这两项留空，仓库里就不会写入某台机器的绝对路径。

```bash
export TELEOP_CAMERA_WORKSPACE="$HOME/catkin_ws"
export TELEOP_DETECTION_PYTHON="$HOME/miniconda3/envs/detection/bin/python"
```

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

## 许可证

Apache License 2.0。详见 [LICENSE](LICENSE)。
