# ROS1 上位机启动指南

## 1. 创建树莓派工作空间

在树莓派 Ubuntu 20.04 上执行：

```bash
mkdir -p ~/tao_robot_ws/src
cd ~/tao_robot_ws
catkin_make
source devel/setup.bash
```

建议加入 `~/.bashrc`：

```bash
echo 'source ~/tao_robot_ws/devel/setup.bash' >> ~/.bashrc
```

## 2. 验证 ROS1

终端 1：

```bash
roscore
```

终端 2：

```bash
rostopic list
```

应至少看到：

```text
/rosout
/rosout_agg
```

## 3. 确认 STM32 串口

连接 STM32 后执行：

```bash
ls /dev/ttyUSB*
ls /dev/ttyAMA*
ls /dev/ttyS*
```

常见设备：

```text
/dev/ttyUSB0
```

加入串口权限组：

```bash
sudo usermod -aG dialout $USER
```

执行后重新登录或重启。

## 4. 安全测试顺序

第一条控制命令必须是 stop，不直接前进。

推荐顺序：

1. ROS 节点能打开串口。
2. 发送 stop。
3. 低速 `/cmd_vel` 测试底盘。
4. 发送机械臂 Home。
5. 再做键盘控制、摄像头、视觉和巡检状态机。

不要一开始运行导航、建图、MoveIt、巡线或颜色抓取真车程序。

## 5. 当前底盘链路验证

当前已经验证的 ROS 到 STM32 底盘链路：

```text
/cmd_vel -> tao_serial_node -> BASE_VEL v2 frame -> STM32F407 -> 四轮电机
```

启动串口节点：

```bash
cd ~/tao_robot_ws
source devel/setup.bash
roslaunch tao_bringup serial_test.launch open_serial:=true port:=/dev/ttyS0
```

另开终端低速测试：

```bash
cd ~/tao_robot_ws
source devel/setup.bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

预期：四个轮子低速转动，`tao_serial_node` 日志出现 `cmd_vel -> base_vel` 和 `TX v2`。

停止测试：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

如果停止发布 `/cmd_vel`，节点会在 `cmd_timeout` 后继续发送零速度；如果上位机断联，STM32 应依靠心跳超时停车。

## 6. 后续验收顺序

底盘跑通后，不要立刻进入完整任务。建议继续按以下顺序验收：

1. `/buzzer/play -> BUZZER`。
2. `ARM_JOINTS` 单关节低速测试。
3. `GRIPPER` 开合测试。
4. `STATUS` 到 ROS 状态 / `/joint_states`。
5. `ACK/ERROR` 错误可视化。
6. MoveIt `home` 和 `wave`。
7. 巡线空跑，不抓取。
8. 巡线 + 抓取 + 放置。

## 7. 蜂鸣器、夹爪、机械臂直连测试

这些测试只验证通信底座，不代表最终 MoveIt 控制方式。机械臂测试必须先断开危险负载、抬空车轮、确认舵机限位无误，再从极小角度开始。

蜂鸣器 ROS topic 测试：

```bash
rostopic pub -1 /buzzer/play std_msgs/UInt8 "data: 1"
```

夹爪 ROS topic 测试：

```bash
rostopic pub -1 /gripper/command std_msgs/UInt8 "data: 50"
```

机械臂协议单位 ROS topic 测试，单位是 `rad * 1000`，第一版固定 6 个关节：

```bash
rostopic pub -1 /tao_arm/joints_protocol_units std_msgs/Int16MultiArray "data: [0, 0, 0, 0, 0, 0]"
```

也可以绕过 ROS，直接用串口脚本测试：

```bash
python3 src/tao_serial/scripts/test_f407_v2_serial.py --port /dev/ttyS0 buzzer --melody 1 --repeat 1
python3 src/tao_serial/scripts/test_f407_v2_serial.py --port /dev/ttyS0 gripper 50
python3 src/tao_serial/scripts/test_f407_v2_serial.py --port /dev/ttyS0 arm-joints 0 0 0 0 0 0 --duration 500
```

预期：

- `tao_serial_node` 日志能看到 `TX v2`。
- STM32 对低频命令返回 `ACK`，上位机 `/tao_serial/rx` 能看到 `ack ...`。
- 如果 STM32 返回状态帧，`/tao_serial/rx` 能看到 `status ... joints=(...)`。

## 8. 虚拟机 MoveIt 真机桥接测试

最终联调时建议分两台 ROS 主机运行：

```text
虚拟机 192.168.137.2：roscore、MoveIt、RViz、tao_moveit_bridge
树莓派 192.168.137.100：tao_serial_node、STM32 串口
```

虚拟机终端 1：

```bash
export ROS_MASTER_URI=http://192.168.137.2:11311
export ROS_IP=192.168.137.2
cd ~/tao_robot_ws
source devel/setup.bash
roscore
```

树莓派终端：

```bash
export ROS_MASTER_URI=http://192.168.137.2:11311
export ROS_IP=192.168.137.100
cd ~/tao_robot_ws
source devel/setup.bash
roslaunch tao_bringup serial_test.launch open_serial:=true port:=/dev/ttyS0
```

虚拟机终端 2 启动 MoveIt 真机桥接：

```bash
export ROS_MASTER_URI=http://192.168.137.2:11311
export ROS_IP=192.168.137.2
cd ~/tao_robot_ws
source devel/setup.bash
roslaunch tao_bringup moveit_real.launch
```

如果使用本仓库的本地 VS Code 工作流，可以直接运行任务：

```text
Terminal -> Run Task -> Pi: Sync and catkin make
Terminal -> Run Task -> Pi: Launch serial test
Terminal -> Run Task -> Pi: Send arm home via tx
```

这样 Cline 和代码分析运行在电脑本地，树莓派只负责 ROS 节点和 STM32 串口。

`moveit_real.launch` 会启动 `tao_moveit_bridge`，提供 MoveIt simple controller 需要的 action：

```text
/arm_controller/follow_joint_trajectory
```

桥接节点会把 MoveIt 关节角转换成当前 STM32 协议单位，并通过 `/tao_serial/tx` 发送带执行时长的 `ARM_JOINTS` 命令：

```text
protocol = (moveit_angle - offset) * 1000
/tao_serial/tx: ARM_JOINTS seq j0 j1 j2 j3 j4 j5 duration_ms
```

`moveit_real.launch` 默认关闭 `/tao_arm/joints_protocol_units` 影子 topic，避免同一轨迹被 topic 和 tx 两条链路重复下发。如需观察协议单位，可把 `publish_shadow_topic` 改为 `true`，但不要同时让 `tao_serial_node` 订阅并执行这条影子 topic。

当前 home offset：

```text
[0.0, 0.0, 1.602, 1.523, 0.0, 0.0]
```

首次测试必须只做小幅动作，先在 RViz 中 `Plan`，确认轨迹安全后再 `Execute`。如果方向反了，先不要继续扩大动作，修改 `moveit_real.launch` 里的 `protocol_signs` 对应关节符号。
