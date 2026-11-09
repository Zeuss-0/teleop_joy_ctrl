# teleop_joy_ctrl

ROS1 手柄总控功能包，用于底盘运动、Dobot 机械臂、相机曝光与增益，以及光源控制。本功能包不启动、也不依赖录像。

[English](README.md)

## 快速启动

需要底盘和手柄驱动已运行，机械臂工作空间中有 `dobot_v4_bringup`。曝光和增益指令发给单独运行的相机节点（例如 `mvs`）。下文的 `~/test_ws`、`~/catkin_ws`、`~/moma_ws` 是示例路径，换机器时改成现场目录。路径说明见[在新机器上配置路径](#在新机器上配置路径)。

终端 1：需要让曝光和增益作用到相机时，启动相机。

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

默认同进程运行总控节点。设置 `legacy_single_node:=false` 时，意图发布节点和执行节点分成两个节点运行。

首次安装或修改功能包后，先编译机械臂工作空间，再编译本功能包所在工作空间，并重新加载环境。

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
| X | 切换参数锁定；底盘运动和 START 仍可用 |
| 右摇杆按下（R3） | 未绑定 |

方向上/右增加，下/左减少。A/B/Y 用于选择调节模式，调节时需按住左扳机。映射可在配置中修改。右摇杆按下不绑定功能。

按住肩键时，总控发布 `/teleop_joy_ctrl/cmd_vel`。松开后发送一次零速度并停止发布，导航在 mux 超时后可重新接管底盘。

## 在新机器上配置路径

示例中的 `~/test_ws`、`~/catkin_ws`、`~/moma_ws` 分别对应机械臂工作空间（`dobot_v4_bringup`）、相机工作空间（`mvs`，仅在需要把曝光和增益送到相机时）和本功能包所在工作空间。换到新机器时改成实际目录。

## 配置与排查

`config/default.yaml` 配置按键、速度上限、相机调节和光源亮度。

光源亮度 `0.0~1.0` 映射到串口 `0~255`，例如 `0.2` 对应 `51`。HOME 通过 `/dev/light_port` 下发开关与亮度指令。启动光源状态按配置假定，未读取硬件状态。总控 YAML 中的相机默认值不会单独设置相机启动时的曝光和增益。

常用检查命令：

```bash
rostopic echo /teleop_joy_ctrl/state
rostopic echo /camera/exposure_cmd
rostopic echo /camera/gain_cmd
rostopic echo /light/serial_debug
rostopic hz /camera/image_raw
```

曝光或增益没有效果时，先确认相机节点已运行并订阅了对应话题。机械臂控制失败时，检查 TCP 模式及网络连接。终端日志使用中文。

## 许可证

Apache License 2.0。详见 [LICENSE](LICENSE)。
