# catch_moveit_demo MoveIt2 使用与构建说明

本文档说明如何在 Piper/PiperX 机械臂上使用 MoveIt2，把 `catch_ros` 视觉节点输出的箱体顶部预抓取位姿转换为机械臂可执行轨迹。

当前推荐链路是：

```text
D435 + YOLO + 点云拟合
        |
        v
catch_ros 发布 /catch/top_pregrasp_pose
        |
        v
catch_moveit_demo 订阅目标 PoseStamped
        |
        v
MoveIt2 move_group 完成 IK、碰撞检查、轨迹规划
        |
        v
Piper ros2_control / trajectory controller 执行关节轨迹
```

## 1. 包的作用分工

### `piper_with_gripper_moveit`

这是 Piper 带夹爪版本的 MoveIt2 配置包。它本身不负责你的抓取算法，而是告诉 MoveIt：

- 机械臂 URDF/Xacro 模型在哪里；
- 机械臂有哪些规划组，例如 `arm` 和 `gripper`；
- 机械臂每个关节的限位、速度和加速度约束；
- 使用哪个 IK 求解器；
- 轨迹要发送到哪个控制器；
- RViz 和 `move_group` 应该加载哪些配置。

关键配置目录：

```bash
/home/artorias/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config
```

常用文件：

- `piper.urdf.xacro`：机器人模型和 link/joint 定义。
- `piper.srdf`：MoveIt 语义配置，定义规划组、末端 link、自碰撞矩阵。
- `kinematics.yaml`：IK 求解器参数。
- `joint_limits.yaml`：规划时使用的关节限位、速度、加速度。
- `moveit_controllers.yaml`：MoveIt 输出轨迹对应的控制器。
- `ros2_controllers.yaml`：ros2_control 控制器配置。

当前 `piper.srdf` 中常用规划组：

- `arm`：机械臂 1-6 轴，链路通常是 `base_link -> link6`。
- `gripper`：夹爪组，通常单独控制。

### `move_group`

`move_group` 是 MoveIt2 的核心规划节点。它负责：

- 接收目标位姿或目标关节角；
- 查询当前关节状态；
- 调用 IK；
- 检查碰撞；
- 调用 OMPL、Pilz 等规划器；
- 把规划轨迹发送给控制器。

注意：`catch_moveit_demo` 不会自动启动 `move_group`。你必须先启动 `piper_with_gripper_moveit` 的 launch 文件，让 `move_group` 在线。

可用下面命令检查：

```bash
ros2 node list | grep move_group
```

能看到 `/move_group` 才表示 MoveIt 规划服务已经启动。

### `catch_moveit_demo`

这是本项目新增的 C++ MoveIt 客户端示例包。它做的事很少，但很关键：

- 订阅 `catch_ros` 发布的 `/catch/top_pregrasp_pose`；
- 使用 `MoveGroupInterface` 设置末端目标位姿；
- 请求 `move_group` 规划；
- 根据 `execute` 参数决定是否真正执行轨迹。

默认参数：

- `planning_group:=arm`
- `end_effector_link:=link6`
- `target_pose_topic:=/catch/top_pregrasp_pose`
- `execute:=false`

其中 `execute:=false` 是安全默认值，只规划不运动。真实测试前必须先用 dry-run 看规划是否合理。

### `catch_ros`

`catch_ros` 是视觉目标发布节点。它使用 D435、YOLO 和点云拟合得到箱体顶部预抓取点，并发布：

```text
/catch/top_pregrasp_pose  geometry_msgs/msg/PoseStamped
```

该话题的坐标系应为 `base_link`，单位为米，姿态为四元数。

## 2. 推荐环境

以下说明按 Ubuntu 22.04 + ROS2 Humble 编写。

建议把 ROS2、Piper SDK、MoveIt2 和本项目都放在同一个 ROS2 工作空间中：

```bash
/home/artorias/piper_ros
```

每个终端启动前都建议 source：

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source /home/artorias/ws_moveit/install/setup.bash
source install/setup.bash
```

如果你的 MoveIt2 是通过 apt 安装，而不是 `/home/artorias/ws_moveit` 源码工作空间，则第二行可以省略。

## 3. 系统依赖安装

### 3.1 ROS2 与 MoveIt2

如果还没有安装 MoveIt2，可以使用 apt：

```bash
sudo apt update
sudo apt install -y ros-humble-moveit
```

如果你使用的是源码编译的 MoveIt2，则需要保证：

```bash
source /home/artorias/ws_moveit/install/setup.bash
```

之后能找到 MoveIt 包：

```bash
ros2 pkg list | grep moveit_ros_planning_interface
```

### 3.2 FCL 依赖

MoveIt 的碰撞检测依赖 FCL。如果编译 `catch_moveit_demo` 时出现：

```text
Could not find a package configuration file provided by "fcl"
```

安装：

```bash
sudo apt install -y libfcl-dev
```

### 3.3 Piper 与控制器依赖

确保 Piper ROS 包已经能单独启动，并且 CAN 口正常：

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  gripper_exist:=true \
  gripper_val_mutiple:=2
```

检查关节状态：

```bash
ros2 topic echo /joint_states
```

如果 `/joint_states` 没有数据，MoveIt 无法知道当前机械臂状态，也就无法可靠规划。

## 4. 完整构建流程

进入 ROS2 工作空间：

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source /home/artorias/ws_moveit/install/setup.bash
```

先确认包能被发现：

```bash
colcon list | grep -E "catch_ros|catch_moveit_demo|piper_with_gripper_moveit"
```

构建视觉节点：

```bash
colcon build --packages-select catch_ros
```

构建 MoveIt demo：

```bash
colcon build --packages-select catch_moveit_demo
```

或者一次构建两个包：

```bash
colcon build --packages-select catch_ros catch_moveit_demo
```

构建完成后 source：

```bash
source install/setup.bash
```

检查可执行文件是否注册成功：

```bash
ros2 pkg executables catch_moveit_demo
```

应该看到类似：

```text
catch_moveit_demo catch_moveit_planner
```

## 5. 启动顺序

真实机械臂测试建议开 4 个终端，按顺序启动。

### 终端 1：启动 Piper 机械臂底层

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  gripper_exist:=true \
  gripper_val_mutiple:=2
```

确认：

```bash
ros2 topic echo /joint_states
```

### 终端 2：启动 MoveIt2

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source /home/artorias/ws_moveit/install/setup.bash
source install/setup.bash

ros2 launch piper_with_gripper_moveit demo.launch.py
```

`demo.launch.py` 通常会启动：

- `robot_state_publisher`
- `move_group`
- RViz
- controller 相关节点或配置

确认 `move_group` 在线：

```bash
ros2 node list | grep move_group
```

### 终端 3：启动 MoveIt 规划节点，先 dry-run

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source /home/artorias/ws_moveit/install/setup.bash
source install/setup.bash

ros2 launch catch_moveit_demo catch_moveit_planner.launch.py execute:=false
```

此时节点会等待 `/catch/top_pregrasp_pose`。收到目标后，它会规划，但不会执行。

### 终端 4：启动视觉目标发布节点

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch catch_ros catch_demo.launch.py \
  model_path:=/home/artorias/work/vision_model/best.pt \
  target_class:=red \
  plane_fit:=true \
  show_window:=true \
  execute:=false \
  publish_moveit_target:=true \
  moveit_target_pose_topic:=/catch/top_pregrasp_pose
```

这里的 `execute:=false` 是 `catch_ros` 自己的直控开关。使用 MoveIt 后，不建议再让 `catch_ros` 直接发 `/pos_cmd` 控制机械臂。

确认视觉节点是否发布目标：

```bash
ros2 topic echo /catch/top_pregrasp_pose
```

## 6. 单点规划测试

不接 D435、不跑 YOLO，也可以先测试 MoveIt 链路。

只规划不执行：

```bash
ros2 launch catch_moveit_demo catch_moveit_planner.launch.py \
  plan_on_start:=true \
  execute:=false \
  target_x:=0.20 \
  target_y:=0.00 \
  target_z:=0.32 \
  target_roll:=0.0 \
  target_pitch:=1.5708 \
  target_yaw:=0.0
```

如果 RViz 中轨迹合理，再尝试执行：

```bash
ros2 launch catch_moveit_demo catch_moveit_planner.launch.py \
  plan_on_start:=true \
  execute:=true \
  target_x:=0.20 \
  target_y:=0.00 \
  target_z:=0.32 \
  target_roll:=0.0 \
  target_pitch:=1.5708 \
  target_yaw:=0.0
```

参数单位：

- `target_x/y/z`：米，基于 `base_link`。
- `target_roll/pitch/yaw`：弧度，基于 `base_link` 下的末端姿态。

## 7. `catch_moveit_planner` 参数说明

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `planning_group` | `arm` | MoveIt 规划组名称，对应 `piper.srdf`。 |
| `end_effector_link` | `link6` | 设置 PoseTarget 的末端 link。若以后增加真实 TCP link，应改为 TCP link。 |
| `target_pose_topic` | `/catch/top_pregrasp_pose` | 订阅的目标位姿话题。 |
| `execute` | `false` | `false` 只规划；`true` 规划成功后执行。 |
| `plan_on_start` | `false` | 启动后是否直接用手动参数规划一次。 |
| `planning_time` | `5.0` | 单次规划允许时间，秒。 |
| `planning_attempts` | `5` | 规划尝试次数。 |
| `max_velocity_scaling` | `0.20` | 最大速度比例，真实测试建议先低速。 |
| `max_acceleration_scaling` | `0.20` | 最大加速度比例。 |
| `position_tolerance` | `0.01` | 末端位置容差，米。 |
| `orientation_tolerance` | `0.10` | 末端姿态容差，弧度。 |
| `min_plan_interval_sec` | `1.0` | 两次规划之间的最小间隔，避免视觉节点每帧触发规划。 |
| `target_position_delta` | `0.02` | 新目标与上一次已接收目标的位置差小于该值时忽略，单位米。 |
| `target_orientation_delta` | `0.15` | 新目标与上一次已接收目标的姿态差小于该值时忽略，单位弧度。 |
| `target_frame` | `base_link` | 手动测试目标坐标系。 |
| `target_x/y/z` | `0.20/0.00/0.32` | 手动测试目标位置，米。 |
| `target_roll/pitch/yaw` | `0.0/1.5708/0.0` | 手动测试目标姿态，弧度。 |

当前 launch 文件已经加载 `piper_with_gripper_moveit` 的 MoveIt 配置，包括 `robot_description`、`robot_description_semantic`、`robot_description_kinematics` 和规划流水线参数。这样 `catch_moveit_planner` 与 `move_group` 使用同一套机器人模型和 IK 配置。

## 8. 与视觉节点融合

`catch_ros` 当前会根据 D435 深度点云估计箱体可见面，再转换到机械臂 `base_link` 坐标系，最后发布顶部预抓取位姿：

```text
/catch/top_pregrasp_pose
```

MoveIt 节点只关心这个目标位姿，不直接读取图像或点云。

融合时重点检查三件事：

1. `catch_ros` 发布的 `header.frame_id` 应该是 `base_link`。
2. `catch_ros` 使用的 D435 到末端标定文件要正确，例如 `/home/artorias/piperx_d435_1.calib`。
3. MoveIt 的 `base_link`、Piper 驱动的 `base_link`、你标定时使用的机械臂基坐标必须一致。

检查目标话题：

```bash
ros2 topic echo /catch/top_pregrasp_pose
```

输出中的 `position` 应该是机械臂基座坐标系下的位置，单位是米。比如：

```text
header:
  frame_id: base_link
pose:
  position:
    x: 0.25
    y: 0.02
    z: 0.31
```

含义是：目标点位于机械臂基座前方 0.25 m、左/右方向 0.02 m、高度 0.31 m。具体 y 正方向以 Piper URDF 坐标定义为准。

## 9. 从 dry-run 到真实执行的安全流程

建议按这个顺序测试：

1. `plan_on_start:=true execute:=false`，验证 MoveIt 能规划。
2. 在 RViz 里确认机械臂模型轨迹不穿桌面、不撞自身、不撞夹爪。
3. `plan_on_start:=true execute:=true`，低速执行一个空中点。
4. 启动 `catch_ros`，只发布视觉目标，`catch_moveit_demo execute:=false`。
5. 对比 `/catch/top_pregrasp_pose` 和真实箱子位置，确认坐标方向、尺度、高度正确。
6. `catch_moveit_demo execute:=true`，只移动到箱体上方预抓取点，不下降夹取。
7. 增加二次校准、下降、闭夹、抬升等动作。

比赛抓取程序不建议让视觉节点直接连续更新并立刻执行。更稳的做法是：

- 底盘停稳；
- 视觉发布目标；
- MoveIt 规划到顶部预抓取点；
- 腕部相机或 D435 二次校准；
- 直线下探；
- 夹爪闭合；
- 力/位移确认；
- 抬升和撤离。

## 10. 常见问题

### 10.1 `move_group` 不存在

现象：

```bash
ros2 node list | grep move_group
```

没有输出。

解决：先启动 MoveIt：

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

### 10.2 编译时报 `fcl` 找不到

安装：

```bash
sudo apt install -y libfcl-dev
```

然后重新构建：

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source /home/artorias/ws_moveit/install/setup.bash
colcon build --packages-select catch_moveit_demo
```

### 10.3 规划失败

常见原因：

- 目标点超出机械臂工作空间；
- 目标姿态过于严格，IK 无解；
- 当前 `/joint_states` 不正确；
- `end_effector_link` 不对；
- `base_link` 坐标系和视觉输出坐标系不一致；
- 目标点在碰撞模型里，例如桌面或箱体模型附近。

处理建议：

- 先增大 `orientation_tolerance`，例如 `0.2`；
- 降低目标高度风险，先测空中点；
- 确认 `/joint_states` 正常；
- 在 RViz 中看目标点是否在机械臂可达范围；
- 暂时只测位置，再逐步收紧姿态。

### 10.4 日志出现 `No kinematics plugins defined`

这个警告表示当前节点没有加载 `kinematics.yaml`，常见于只启动了一个 MoveIt 客户端节点，但没有把 MoveIt 配置传给它。

本包的 `catch_moveit_planner.launch.py` 已经通过下面方式加载 Piper MoveIt 配置：

```python
MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
```

如果仍然看到该警告，优先检查：

```bash
source /home/artorias/ws_moveit/install/setup.bash
source /home/artorias/piper_ros/install/setup.bash
ros2 pkg prefix piper_with_gripper_moveit
```

确认 `piper_with_gripper_moveit/config/kinematics.yaml` 存在，并且当前终端 source 到的是最新构建后的包。

### 10.5 机械臂不动

先确认 `catch_moveit_demo` 是不是 `execute:=false`。这个模式只规划不执行。

如果已经 `execute:=true`，检查：

```bash
ros2 control list_controllers
```

确认机械臂轨迹控制器处于 `active` 状态。

再检查 MoveIt 控制器配置：

```bash
/home/artorias/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/moveit_controllers.yaml
```

其中 `arm_controller` 应该和实际启动的控制器名称一致。

### 10.6 收不到视觉目标

检查话题名是否一致：

```bash
ros2 topic list | grep top_pregrasp
```

检查 `catch_ros`：

```bash
ros2 launch catch_ros catch_demo.launch.py \
  model_path:=/home/artorias/work/vision_model/best.pt \
  target_class:=red \
  publish_moveit_target:=true \
  moveit_target_pose_topic:=/catch/top_pregrasp_pose
```

检查 `catch_moveit_demo`：

```bash
ros2 launch catch_moveit_demo catch_moveit_planner.launch.py \
  target_pose_topic:=/catch/top_pregrasp_pose \
  execute:=false
```

### 10.7 视觉节点发布太频繁，planner 反复规划

`catch_ros` 可能每帧发布一次 `/catch/top_pregrasp_pose`。如果目标点轻微抖动，MoveIt 会连续收到规划请求。

本节点已经加入目标去重：

- `min_plan_interval_sec`：限制两次规划的最小时间间隔；
- `target_position_delta`：目标位置变化太小则忽略；
- `target_orientation_delta`：目标姿态变化太小则忽略。

如果你希望更稳定，可以增大阈值：

```bash
ros2 launch catch_moveit_demo catch_moveit_planner.launch.py \
  execute:=false \
  min_plan_interval_sec:=2.0 \
  target_position_delta:=0.04 \
  target_orientation_delta:=0.25
```

如果你希望调试时更灵敏，可以减小阈值：

```bash
ros2 launch catch_moveit_demo catch_moveit_planner.launch.py \
  execute:=false \
  min_plan_interval_sec:=0.3 \
  target_position_delta:=0.01
```

### 10.8 坐标方向明显不对

优先检查：

- D435 外参标定文件是否加载；
- 是否误用了 `invert_calib`；
- 当前末端位姿订阅是否正常；
- `catch_ros` 输出是否为 `base_link` 坐标系；
- MoveIt 中机器人基座是否也是同一个 `base_link`。

用固定箱子做验证：把箱子放到机械臂正前方，输出的 `x` 应明显为正；抬高箱子时，输出的 `z` 应增加。

## 11. 后续抓取动作如何接入

当前 `catch_moveit_demo` 只规划到顶部预抓取点。完整抓取还需要增加动作序列：

1. MoveIt 到顶部预抓取点；
2. 二次视觉校准箱体中心；
3. 笛卡尔直线下探到插入/抓取高度；
4. 闭合夹爪；
5. 判断夹持是否成功；
6. 抬升；
7. 移动到九宫格放置点；
8. 下降放置；
9. 张开夹爪；
10. 撤离。

建议实现方式：

- 大范围移动：用 MoveIt 规划；
- 最后 50-150 mm 下探：用 MoveIt Cartesian path 或低速伺服；
- 夹爪闭合：单独发夹爪控制命令；
- 所有动作外层用状态机保护。

不要把视觉模型直接变成机械臂执行命令。视觉只给候选目标，最终执行前仍要经过：

- 坐标变换检查；
- 工作空间检查；
- 碰撞检查；
- 目标类别和置信度检查；
- 抓取区域是否安全检查。

## 12. 最小可运行命令汇总

构建：

```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source /home/artorias/ws_moveit/install/setup.bash
colcon build --packages-select catch_ros catch_moveit_demo
source install/setup.bash
```

启动 Piper：

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  gripper_exist:=true \
  gripper_val_mutiple:=2
```

启动 MoveIt：

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

启动 MoveIt demo，只规划：

```bash
ros2 launch catch_moveit_demo catch_moveit_planner.launch.py execute:=false
```

启动视觉节点：

```bash
ros2 launch catch_ros catch_demo.launch.py \
  model_path:=/home/artorias/work/vision_model/best.pt \
  target_class:=red \
  plane_fit:=true \
  show_window:=true \
  execute:=false \
  publish_moveit_target:=true
```

确认安全后执行：

```bash
ros2 launch catch_moveit_demo catch_moveit_planner.launch.py execute:=true
```
