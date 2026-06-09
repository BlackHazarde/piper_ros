#### 环境安装
安装moveit2
```bash
sudo apt-get update
sudo apt install ros-$ROS_DISTRO-moveit
```

安装can依赖,can工具,科学计算库,Piper 机械臂SDK,ros2-control,gui,imu-tools,imu-filter-madgwick,卸载brltty(冲突)
```bash
pip3 install python-can
pip3 install scipy
pip3 install piper_sdk
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
sudo apt install can-utils
sudo apt install ethtool
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install ros-humble-imu-filter-madgwick
sudo apt remove brltty
sudo apt install ros-humble-imu-tools
```

在ros2中安装驱动串口serial_driver
```bash
sudo apt update
sudo apt install ros-humble-asio-cmake-module ros-humble-serial-driver
```

安装tf工具

```bash
sudo apt install ros-$ROS_DISTRO-tf-transformations
```
安装roboticstoolbox-python包

```bash
pip3 install roboticstoolbox-python
```

升级transforms3d包

```bash
pip install --user --upgrade transforms3d
```

#### 构建和source 
构建
```bash
colcon build
```

source
```bash
source install/setup.bash
```

#### 启动
配置串口权限
```bash
sudo chmod 666 /dev/ttyACM0
```

launch 启动
```bash
ros2 launch bringup rc_control.launch.py 
```

#### 分布启动
配置串口权限
```bash
sudo chmod 666 /dev/ttyACM0
```

启动串口驱动
```bash
ros2 launch serial_driver serial_driver_bridge_node.launch.py
```

can查找
```bash
bash ./pkg/piper/find_all_can_port.sh
```

配置波特率
```bash
bash can_activate.sh can0 1000000
```

两种启动方式（前者不带rviz）
```bash
ros2 launch piper start_single_piper.launch.py 
ros2 launch piper start_single_piper_rviz.launch.py 
```

启动解包串口数据
```bash
ros2 run process_serial_data unload_serial_data
```

启动发布目标位姿
```bash
ros2 run process_serial_data pub_target_pose
```

启动广播tf
```bash
ros2 run process_serial_data tf_broadcast
```

启动发布来自下位机的夹爪控制消息
```bash
ros2 run process_serial_data pub_gripper_control
```

启动变换后的tf
```bash
ros2 run robotic_arm_control offset_tf_broadcast
```

启动遥控器控制
```bash
ros2 run robotic_arm_control rc_control
```


#### 调试
获取/end_pose
```bash
ros2 topic echo /end_pose
```

命令行控制一次机械臂，注意不要改过大，可以从/end_pose获取末端位姿
```bash
ros2 topic pub /pos_cmd piper_msgs/msg/PosCmd "{x: 0.04, y: 0.01, z: 0.3, roll: -0.15, pitch: 0.57, yaw: 0.02, gripper: 0.0}" --once
```

获取/end_pose
```bash
ros2 topic echo /end_pose
```

查看serial_driver的topic
```bash
ros2 topic list
```

打印serial_read
```bash
ros2 topic echo serial_read --once
```

#### 协议
接收协议 下位机至上位机

| 偏移 | 字节大小 | 数据类型    | 含义      | 详细解释                     |
|:---|:-----|---------|:--------|:-------------------------|
| 0  | 2    |         | 帧头      | 0x55和0xAA                |
| 2  | 4    | float32 | 四元数 w   | -1到1                     |
| 6  | 4    | float32 | 四元数 x   | -1到1                     |
| 10 | 4    | float32 | 四元数 y   | -1到1                     |
| 14 | 4    | float32 | 四元数 z   | -1到1                     |
| 18 | 2    | int16   | 右摇杆上下方向 | -660到+660                |
| 20 | 2    | int16   | 右摇杆左右方向 | -660到+660                |
| 22 | 2    | int16   | 左摇杆上下方向 | -660到+660                |
| 24 | 2    | int16   | 左摇杆上下方向 | -660到+660                |
| 26 | 2    | int16   | 滚轮      | -660到+660(顺时钟是负数，逆时针是正数) |
| 28 | 1    | uint8   | 右拨动开关   | 下2 中间2 上1                |
| 30 | 1    | uint8   | 左拨动开关   | 下2 中间2 上1                |
| 62 | 2    |         | 帧尾      | 0x0D和0x0A                |

#### 使用的开源
piper的humble分支
```bash
git clone https://github.com/agilexrobotics/piper_ros.git
git checkout humble
```

亚博智能的imu 密码ibhe
```
https://www.yahboom.com/study/IMU_Sensor#xuanzhon_5
```

#### 疑难杂症
numpy 版本过高报错
```bash
pip install numpy==1.23.5
```
```bash
cd /home/artorias/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

bash can_activate.sh can0 1000000

ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2

  ros2 run catch_ros catch_node --ros-args \
  -p model_path:=/home/artorias/work/vision_model/best.pt \
  -p target_class:=blue \
  -p plane_fit:=true \
  -p enable_arm:=true \
  -p execute:=true
  ```

  "{x: 0.2, y: 0.0, z: 0.2,roll: 0.0,pitch :1.2,yaw: 0.0, gripper: 0.0,mode1: 0,mode2: 0}"

  ros2 topic pub --once /pos_cmd piper_msgs/msg/PosCmd "{x: -0.006617,y: -0.051305,z: 0.20,roll: 0.0,pitch: 0.0,yaw: 0.0, gripper: 0.0,mode1: 0,mode2: 0}"


  ros2 launch piper start_single_piper.launch.py can_port:=can0 gripper_exist:=true gripper_val_mutiple:=2
  ros2 launch piper_with_gripper_moveit demo.launch.py
  ros2 launch catch_moveit_demo catch_moveit_planner.launch.py execute:=false
  os2 launch catch_ros catch_demo.launch.py model_path:=/home/artorias/work/vision_model/best.pt target_class:=red execute:=false publish_moveit_target:=true
