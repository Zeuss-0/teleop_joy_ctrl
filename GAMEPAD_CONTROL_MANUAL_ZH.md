# 机器人手柄控制说明书

版本：第一版现场操作稿  
更新日期：2026-07-27  
适用功能包：`teleop_joy_ctrl`（ROS1）  
适用对象：机器人现场操作、联调和故障排查人员

## 1. 功能范围

本功能包通过同一个手柄完成以下操作：

- 小车低速、高速运动控制；
- 小车线速度和角速度上限调节；
- Dobot 机械臂上使能、下使能和清错；
- 相机曝光、增益调节；
- 光源开关和亮度调节；
- 手柄参数锁定；
- 手柄控制与自主导航之间的自动切换。
- R3 三路视频录制、暂停、继续与安全结束。

手柄输入话题默认为 `/joy_teleop/joy`。手柄小车速度发布到 `/teleop_joy_ctrl/cmd_vel`，经接管用 `twist_mux` 输出到 `/bulldog_velocity_controller/cmd_vel`。

## 2. 使用前安全检查

启动前至少确认以下事项：

1. 小车周围没有人员、线缆或其他障碍物。
2. 急停按钮可用，并明确急停位置。
3. 手柄已正确连接，系统中存在 `/joy_teleop/joy`。
4. 机械臂需要控制时，机械臂已切换到 TCP 模式。
5. 机械臂地址为当前配置的 `192.168.1.80`，端口 `29999`、`30004` 可连接。
6. 光源需要控制时，串口设备 `/dev/light_port` 已连接且当前用户有访问权限。
7. 首次测试时先架空驱动轮或限制运动区域，确认方向无误后再落地运行。

## 3. 启动方法

机械臂服务定义位于 `test_ws`，MVS 相机和 YOLO 包位于 `catkin_ws`，手柄总控功能包位于 `moma_ws`。推荐在启动总控前按顺序加载三个工作空间：

```bash
source ~/test_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/moma_ws/devel/setup.bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch
```

默认 launch 会同时启动：

- 接管用 `twist_mux`；
- `dobot_v4_bringup/bringup_v4.launch`；
- `teleop_joy_ctrl_node.py`；
- `light_serial_controller_node.py`；
- `video_recording_manager_node.py`；
- 退出时恢复系统原手柄节点的守护节点。

导航可以先启动，再启动手柄总控。手柄未主动控制小车时，总控不会持续发送零速度，不会阻断导航。

不连接机械臂或光源进行单项测试时，可使用：

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_arm_bringup:=false \
  start_light_controller:=false
```

## 4. 当前按键与轴映射

| 手柄控件 | ROS 索引 | 当前功能 |
| --- | ---: | --- |
| 左摇杆左右 | axis 0 | 小车转向 |
| 左摇杆前后 | axis 1 | 小车前进、后退 |
| 左扳机 LT | axis 2 | 参数调节确认键，不需要完全按到底 |
| 方向键左右 | axis 6 | 角速度上限或曝光调节 |
| 方向键上下 | axis 7 | 线速度上限、增益或亮度调节 |
| A | button 0 | 选择底盘参数模式 |
| B | button 1 | 选择相机参数模式 |
| X | button 2 | 切换锁定/解锁 |
| Y | button 3 | 选择光源亮度模式 |
| 左肩键 LB | button 4 | 低速小车控制使能 |
| 右肩键 RB | button 5 | 高速小车控制使能 |
| START | button 7 | 机械臂状态控制 |
| HOME | button 8 | 光源开关 |
| 右摇杆按下 R3 | button 10 | 双击开始录像；短按暂停/继续；长按结束 |
| BACK | 未绑定 | 预留，不执行任何操作 |

如更换手柄，必须先用下面的命令重新核对索引：

```bash
rostopic echo /joy_teleop/joy
```

## 5. 小车运动控制

### 5.1 低速控制

1. 按住左肩键 LB。
2. 保持 LB 不松开，推动左摇杆。
3. 左摇杆前后控制前进、后退，左右控制转向。
4. 松开 LB 后，总控发送一次零速度停车，然后停止发布速度。

低速比例为当前速度上限的 `0.60`。节点启动后的默认上限为线速度 `0.4 m/s`、
角速度 `0.8 rad/s`，因此默认低速档为：

- 最大线速度约为 `0.4 × 0.60 = 0.24 m/s`；
- 最大角速度约为 `0.8 × 0.60 = 0.48 rad/s`。

在 A 模式中调高速度上限后，低速档会随当前上限同步提高。例如线速度上限调到
配置允许的 `3.0 m/s` 时，低速档最大线速度为 `1.8 m/s`。

### 5.2 高速控制

1. 按住右肩键 RB。
2. 保持 RB 不松开，推动左摇杆。
3. 松开 RB 后，总控发送一次零速度停车，然后停止发布速度。

高速比例为当前速度上限的 `1.0`。节点启动后的默认最大线速度为 `0.4 m/s`，
默认最大角速度为 `0.8 rad/s`；在 A 模式中调节后，高速档直接使用调节后的当前上限。
配置允许的最高线速度和角速度分别为 `3.0 m/s`、`4.0 rad/s`。

同时按住 LB 和 RB 时，程序按高速档处理。只推动左摇杆而不按肩键，小车不会响应。

### 5.3 与导航的控制权切换

`twist_mux` 当前优先级如下：

| 输入 | 优先级 | 超时 |
| --- | ---: | ---: |
| 急停锁 `/e_stop` | 255 | 0.0 s |
| 手柄总控 `/teleop_joy_ctrl/cmd_vel` | 254 | 0.3 s |
| 原系统手柄 `/joy_teleop/cmd_vel` | 20 | 0.5 s |
| 交互标记 `/gps/cmd_vel` | 10 | 1.0 s |
| 导航 `/platform_control/cmd_vel` | 2 | 0.5 s |
| 外部 `/cmd_vel` | 1 | 0.5 s |

控制过程为：

1. 未按 LB/RB：总控不发布速度，导航可控制小车。
2. 按住 LB 或 RB：总控取得高优先级控制权。
3. 松开肩键：总控发送一次零速度，随后停止发布。
4. 约 `0.3 s` 后，`twist_mux` 允许导航重新接管。
5. 手柄数据超过 `0.5 s` 未更新时，同样发送一次零速度并释放控制权。

## 6. 参数调节通用方法

参数调节采用“先选模式，再按住 LT 加方向键”的方式：

1. 按 A、B 或 Y 选择目标模式。
2. 按住左扳机 LT。当前阈值为 `0.95`，检测到扳机开始按下即可，不需要完全按到底。
3. 在 LT 保持按下时按方向键调节。
4. 持续按住方向键时，每 `0.25 s` 调节一次。

同一时间只保留一个参数模式。按下另一个模式键后会切换模式。

### 6.1 方向键总表

| 已选模式 | 方向上 | 方向下 | 方向左 | 方向右 |
| --- | --- | --- | --- | --- |
| A：底盘参数 | 增大线速度上限 | 减小线速度上限 | 减小角速度上限 | 增大角速度上限 |
| B：相机参数 | 增大增益 | 减小增益 | 减小曝光 | 增大曝光 |
| Y：光源亮度 | 增大亮度 | 减小亮度 | 无功能 | 无功能 |

左右方向已经调整为符合直觉的“左减、右加”。

## 7. 底盘参数调节

1. 按 A，终端应显示“模式=底盘参数”。
2. 按住 LT。
3. 方向上/下调节线速度上限。
4. 方向左/右调节角速度上限。

当前范围和步长：

| 参数 | 默认值 | 最小值 | 最大值 | 单次步长 |
| --- | ---: | ---: | ---: | --- |
| 线速度上限 | 0.4 m/s | 0.05 m/s | 3.0 m/s | 当前值的 10%，但不小于 0.02 m/s |
| 角速度上限 | 0.8 rad/s | 0.05 rad/s | 4.0 rad/s | 当前值的 10%，但不小于 0.05 rad/s |

调节结果只在当前节点运行期间有效，重启节点后恢复 `default.yaml` 中的默认值。

## 8. 相机参数调节

1. 按 B，终端应显示“模式=相机参数”。
2. 按住 LT。
3. 方向上/下调节增益。
4. 方向左/右调节曝光。

当前范围和步长：

| 参数 | 默认值 | 最小值 | 最大值 | 单次步长 |
| --- | ---: | ---: | ---: | --- |
| 曝光 | 5000 | 20 | 50000 | 当前值的 12%，但不小于 100 |
| 增益 | 0 | 0 | 24 | 当前值的 10%，但不小于 0.5 |

控制话题：

- 曝光指令：`/camera/exposure_cmd`
- 增益指令：`/camera/gain_cmd`
- 曝光状态：`/camera/exposure_state`
- 增益状态：`/camera/gain_state`

相机节点发布状态话题后，总控会用实际状态更新内部数值。若相机界面没有同步，先检查四个话题是否存在和是否有数据。

## 9. 机械臂控制

机械臂只使用 START 一个按键，根据当前 `RobotMode` 自动选择操作：

| 当前 RobotMode | START 动作 |
| --- | --- |
| ERROR（9） | 调用 `ClearError` 清错 |
| COLLISION（11） | 调用 `ClearError` 清错 |
| ENABLE_IDLE（5） | 调用 `DisableRobot` 下使能 |
| RUNNING（7） | 调用 `DisableRobot` 下使能 |
| SINGLE_MOVE（8） | 调用 `DisableRobot` 下使能 |
| 其他模式 | 调用 `EnableRobot` 上使能 |

机械臂服务路径默认为：

```text
/dobot_v4_bringup/srv/RobotMode
/dobot_v4_bringup/srv/EnableRobot
/dobot_v4_bringup/srv/DisableRobot
/dobot_v4_bringup/srv/ClearError
```

注意事项：

- 机械臂必须处于 TCP 模式，否则服务存在也可能无法完成控制。
- 当前默认 IP 是 `192.168.1.80`。
- START 是边沿触发按键，每次操作需要松开后重新按下。
- 当前代码中 START 在 X 锁定状态下仍然有效。
- RobotMode 查询或服务调用失败时，以终端警告为准，不要连续快速按键。

## 10. 光源控制

### 10.1 HOME 开关

HOME 只在未锁定状态下有效。

- 当前亮度大于 0：按 HOME 将亮度设为 0，并关闭光源。
- 当前亮度为 0：按 HOME 开启光源，并恢复固定亮度 `0.50`。

HOME 通过两个 ROS 话题驱动串口节点，不是只修改总控节点内部的亮度变量。当前默认配置
`power_publish_repeat: 2`，因此串口命令顺序为：

- 关闭：先发布亮度 `0.00`，串口发送一次 `set,stepall,0`；随后发布关灯状态两次，串口发送两次 `set,offall`。
- 开启：先发布开灯状态两次，串口发送两次 `set,onall`；随后发布恢复亮度 `0.50`，串口发送 `set,stepall,128`。

实际命令是 `set,onall` 与 `set,offall`，不是 `set,on` 或 `set,off`。如在
`default.yaml` 中修改了 `power_publish_repeat`、亮度范围或命令模板，实际串口次数和数值会随之改变。

### 10.2 从 0 开始调节亮度

如果当前亮度为 0，不需要先按 HOME：

1. 按 Y 选择光源亮度模式。
2. 按住 LT。
3. 按方向上，亮度从 `0.00` 增加到 `0.05`。
4. 程序先开启光源，再发送新的亮度值。

该操作与 HOME 的固定 `0.50` 恢复逻辑不同。

### 10.3 亮度范围

- 内部亮度范围：`0.00～1.00`
- 单次步长：`0.05`
- 串口输出范围：`0～255`
- 串口命令：`set,stepall,{value}`
- 开灯命令：`set,onall`
- 关灯命令：`set,offall`

当前版本启动时按配置假定光源已开启、亮度为 `0.50`，没有从光源硬件主动读取开关和亮度状态。如果实际光源状态与配置不一致，第一次 HOME 操作可能与肉眼状态不一致，应以终端状态和串口输出共同判断。

## 11. X 锁定功能

按一次 X 进入锁定，再按一次 X 解锁。切换锁定状态时，当前参数模式会清除为“未选择”。

当前代码的锁定行为如下：

| 功能 | 锁定后是否可用 |
| --- | --- |
| LB/RB + 左摇杆控制小车 | 可用 |
| A/B/Y 选择参数模式 | 不可用 |
| LT + 方向键调参 | 不可用 |
| HOME 光源开关 | 不可用 |
| START 机械臂控制 | 仍可用 |
| R3 录像控制 | 仍可用 |

## 12. 终端状态和调试输出

总控使用中文输出执行结果。常用话题：

```bash
rostopic echo /teleop_joy_ctrl/state
rostopic echo /teleop_joy_ctrl/debug
rostopic echo /light/serial_state
rostopic echo /light/serial_debug
```

`/teleop_joy_ctrl/state` 包含：

- 最近事件；
- 是否锁定；
- 当前参数模式；
- 当前线速度、角速度上限；
- 当前曝光、增益；
- 当前光源开关和亮度。

成功指令会以 `[成功]` 开头，例如：

```text
[成功] 底盘速度指令 ...
[成功] 底盘手柄控制已释放，已发送一次零速度并停止发布，等待导航接管
[成功] 相机曝光指令 ...
[成功] 机械臂上使能指令 ...
[成功] 光源亮度指令 ...
```

## 13. 现场快速检查命令

### 13.1 检查手柄

```bash
rostopic hz /joy_teleop/joy
rostopic echo -n 1 /joy_teleop/joy
```

### 13.2 检查导航与手柄速度

```bash
rostopic hz /platform_control/cmd_vel
rostopic hz /teleop_joy_ctrl/cmd_vel
rostopic echo /bulldog_velocity_controller/cmd_vel
```

未按 LB/RB 时，`/teleop_joy_ctrl/cmd_vel` 应保持静默；按住肩键后才应持续出现消息。

### 13.3 检查机械臂

```bash
rosservice list | grep -E 'RobotMode|EnableRobot|DisableRobot|ClearError'
ping -c 3 192.168.1.80
nc -vz -w 3 192.168.1.80 29999
nc -vz -w 3 192.168.1.80 30004
```

### 13.4 检查光源

```bash
ls -l /dev/light_port
rostopic echo /light/serial_debug
rostopic echo /light/serial_state
```

### 13.5 检查相机

```bash
rostopic echo /camera/exposure_state
rostopic echo /camera/gain_state
rostopic echo /camera/exposure_cmd
rostopic echo /camera/gain_cmd
```

## 14. 常见问题

### 小车完全不动

1. 确认按住了 LB 或 RB，左摇杆单独操作无效。
2. 检查 `/joy_teleop/joy` 是否持续发布。
3. 检查手柄按键索引是否与本说明书一致。
4. 检查 `/teleop_joy_ctrl/cmd_vel` 和底盘最终话题。
5. 检查 `/e_stop` 是否处于锁定状态。

### 启动总控后导航不动

1. 不按任何肩键，检查 `/teleop_joy_ctrl/cmd_vel` 是否仍有持续消息。
2. 检查 `/platform_control/cmd_vel` 是否有导航速度。
3. 检查接管用 `twist_mux` 是否加载了 `twist_mux_takeover.yaml`。
4. 松开肩键后等待至少 `0.3 s` 再观察导航输出。

### START 没有控制机械臂

1. 确认机械臂处于 TCP 模式。
2. 确认已加载 `test_ws` 和 `moma_ws`。
3. 确认四个 Dobot 服务存在。
4. 检查 `192.168.1.80:29999` 和 `192.168.1.80:30004`。
5. 查看终端中的 RobotMode 和服务调用警告。

### HOME 无法控制光源

1. 确认当前没有处于 X 锁定状态。
2. 检查 `/dev/light_port` 是否存在及其权限。
3. 检查 `/light/power_cmd`、`/light/brightness_cmd`。
4. 查看 `/light/serial_debug` 是否出现串口写入成功信息。

### 模式按键第一次没有反应

节点收到第一帧 Joy 消息时会先记录按键状态。若启动节点时某个按键已经按住，需要先松开，再重新按一次。LB/RB 的持续运动控制不依赖该边沿逻辑。

## 15. 默认混合解耦录像

当前默认 R3 后端为 `hybrid_video_recorder`，由三个独立节点组成：

- `hybrid_video_recorder/raw`：持续订阅 `/camera/image_raw`，录制与暂停期间均写入原始视频。
- `hybrid_video_recorder/aimed`：持续订阅同一话题，仅在“录制中”写入 aimed 视频。
- `video_recording_manager_node`：保持原有 `/teleop_joy_ctrl/video/*` 服务，协调两个写入节点。

相机 `camera_stream` 不再由 R3 启动或结束，必须作为独立进程先启动：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun mvs camera_stream
```

空闲状态双击 R3 后，管理器在同一时间戳目录中创建：

```text
<时间戳>.mp4
aimed_<时间戳>.mp4
```

短按 R3 暂停时，两个工作节点保持图像订阅不断开：原始视频继续写入，aimed 视频停止写入；再次短按后 aimed 继续写入同一文件。长按 R3 至少 `1.5 s` 时，两个独立 FFmpeg 写入器分别封装。aimed 写入或 YOLO 检测失败不会取消原始视频保存。

YOLO 是此模式下的独立可选进程，不再能够阻断 R3 的数据保底录像。需要临时回退旧版 YOLO 耦合录像时，使用：

```bash
roslaunch teleop_joy_ctrl teleop_joy_ctrl.launch \
  start_hybrid_video_manager:=false \
  start_legacy_video_manager:=true
```

## 16. 旧版 YOLO 耦合录像回退

R3 已实测对应 `/joy_teleop/joy` 的 `buttons[10]`。录像控制在 X
锁定状态下仍然有效，确保现场人员始终能够暂停或安全结束录像。

### 15.1 开始录像

空闲状态下双击 R3，两次短按的间隔不得超过 `0.4 s`。录像管理节点会：

1. 从 `/home/shansu/catkin_ws` 启动 `mvs camera_stream`。
2. 等待 `/camera/image_raw` 就绪。
3. 使用 `/home/shansu/miniconda3/envs/detection/bin/python` 启动
   `mvs yolo_detector.py --save-video`。默认保留 NoMachine 图形预览；无图形终端时将
   `default.yaml` 中的 `video.detector_headless` 改为 `true`。
4. 等待 YOLO 录像服务就绪，然后进入 `RECORDING` 状态。

启动 `teleop_joy_ctrl.launch` 时只会启动录像管理节点，不会立即占用相机
或启动 YOLO。空闲状态单击或长按 R3 不执行录像操作。

### 15.2 暂停与继续

- `RECORDING` 状态短按 R3：暂停 `aimed_<时间戳>.mp4` 的原始帧写入。
- `PAUSED` 状态短按 R3：继续写入同一个 `aimed_<时间戳>.mp4` 文件。
- 暂停期间相机采集、YOLO 推理、原始连续视频和带识别框视频继续写入；仅 aimed 视频跳过暂停区间。
- 录制或暂停状态不识别双击，因此快速短按不会被判定为结束录像。

### 15.3 结束并关闭节点

在 `RECORDING` 或 `PAUSED` 状态长按 R3 至少 `1.5 s`：

1. 禁止新录像帧入队并排空待写队列。
2. 关闭两个 FFmpeg 写入器并完成 MP4 封装。
3. 关闭管理节点自己启动的 `yolo_detector_node`。
4. 关闭管理节点自己启动的 `camera_stream_node`。
5. 录像状态返回 `IDLE`。

自动模式下，管理节点会关闭自己启动的相机和 YOLO 进程。若相机和 YOLO 在启动总控前
已由人工启动，且 `video.adopt_existing_detector: true`，管理节点会检查相机图像话题、
YOLO 录像服务和录像状态；检查通过后，下一次空闲状态双击 R3 会接管现有录像会话，
不会重新启动第二个 YOLO 进程。

手动接管模式下，R3 的暂停、继续和结束仍然有效；但长按结束只完成录像封装，不会关闭
人工启动的进程。结束后按下面命令完全退出：

```bash
rosservice call /yolo_detector_node/recording/shutdown
rosnode kill /camera_stream_node
```

手动启动命令如下，自动拉起 YOLO 失败时可使用此方式：

```bash
# 终端 1
source ~/catkin_ws/devel/setup.bash
rosrun mvs camera_stream

# 终端 2
source ~/catkin_ws/devel/setup.bash
rosrun --prefix "/home/shansu/miniconda3/envs/detection/bin/python" mvs yolo_detector.py --save-video
```

### 15.4 文件命名与检查

每次录像只生成一个时间戳。目录和原始视频同名：

```text
/home/shansu/catkin_ws/src/mvs/scripts/recordings/20260723_153012/
├── 20260723_153012.mp4
├── yolo.mp4
└── aimed_20260723_153012.mp4
```

时间戳命名的原始视频与 `yolo.mp4` 从启动到结束连续写入；
`aimed_时间戳.mp4` 使用原始画面，但仅在“录制中”写入，暂停期间不写入。

常用检查命令：

```bash
rosnode list | grep -E 'video_recording_manager|camera_stream_node|yolo_detector_node'
rosservice list | grep '/teleop_joy_ctrl/video'
rostopic echo /teleop_joy_ctrl/video/state
rostopic echo /teleop_joy_ctrl/video/status
rostopic echo /teleop_joy_ctrl/video/output_dir
```

部署时将 `teleop_joy_ctrl` 放入 `/home/shansu/moma_ws/src`，将修改后的
MVS 文件放入 `/home/shansu/catkin_ws/src/mvs`。不要在 `moma_ws` 中放置
第二份 `mvs` 包。

## 17. 当前版本限制

- 所有调节值默认只保存在当前运行内存中，重启后恢复配置值。
- 光源启动状态是配置假定值，不是硬件回读值。
- START 在锁定状态下仍可控制机械臂。
- BACK 尚未绑定功能。
- R3 录像功能依赖 detection Python、FFmpeg 和足够的磁盘空间；自动拉起 YOLO 失败时可使用第 15 节的手动接管流程。
- 视频全黑通常表示输入画面亮度不足，而非 FFmpeg 封装失败；先检查光源、曝光和增益。
- 更换手柄、驱动或系统后，axes/buttons 索引可能改变，必须重新测试。
- 本说明书依据 2026-07-27 当前代码和 `default.yaml` 编写；后续修改控制逻辑时应同步更新。
