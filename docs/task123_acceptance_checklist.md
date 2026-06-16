# 任务 1/2/3 通信与 MoveIt 底座验收清单

目标：把通信协议、ROS 桥接、MoveIt 真机机械臂链路收口到足够稳定，再进入巡线、物块、人脸、标签码垛等视觉任务。

原则：安全、简单、可靠。所有测试默认低速、短时；机械臂测试前必须确认周围无人、无障碍物，手边保留急停/断电手段。

## 1. 当前交付内容

### 1.1 通信协议底座

- `docs/serial_protocol_v2.md`：v2 二进制帧格式与命令定义。
- `src/tao_serial/scripts/serial_protocol.py`：ROS 端 v2 编解码。
- `src/tao_serial/scripts/tao_serial_node.py`：统一串口节点，负责收发、心跳、基础桥接和反馈发布。
- `src/tao_serial/scripts/test_f407_v2_serial.py`：串口直连安全测试工具。

### 1.2 ROS 桥接

上位机统一通过以下接口控制硬件：

```text
/cmd_vel                          geometry_msgs/Twist       -> BASE_VEL
/buzzer/play                      std_msgs/UInt8            -> BUZZER
/gripper/command                  std_msgs/UInt8            -> GRIPPER
/tao_arm/joints_protocol_units    std_msgs/Int16MultiArray  -> ARM_JOINTS
/tao_serial/tx                    std_msgs/String           -> 调试文本命令转 v2 帧
```

STM32 反馈统一发布为：

```text
/tao_serial/rx                    std_msgs/String           人工调试摘要
/tao_serial/status_json           std_msgs/String           STATUS JSON
/tao_serial/ack_json              std_msgs/String           ACK JSON
/tao_serial/error_json            std_msgs/String           ERROR JSON
/tao_serial/pong_json             std_msgs/String           PONG JSON
/joint_states                     sensor_msgs/JointState    STATUS 关节镜像，若 STM32 有回传
```

### 1.3 MoveIt 真机链路

- `src/tao_serial/scripts/tao_moveit_bridge.py`：`FollowJointTrajectory` 到 `ARM_JOINTS` 桥接。
- `src/tao_bringup/config/joint_map.yaml`：关节方向、零点、限幅、单位换算。
- `src/tao_bringup/config/task_poses.yaml`：后续任务使用的 home/pick/place/wave/stack 初始位姿。
- `src/tao_bringup/launch/moveit_real.launch`：真机 MoveIt 启动入口。

## 2. 树莓派 ROS 串口节点验收

启动：

```bash
cd ~/tao_robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tao_bringup serial_test.launch open_serial:=true port:=/dev/ttyS0 log_tx:=true log_rx:=true
```

另开终端检查：

```bash
source ~/tao_robot_ws/devel/setup.bash
rostopic list
```

必须能看到：

```text
/cmd_vel
/buzzer/play
/gripper/command
/tao_serial/tx
/tao_serial/rx
/tao_serial/status_json
/tao_serial/ack_json
/tao_serial/error_json
/tao_serial/pong_json
```

## 3. 通信底座安全测试

如果直接连串口测试，可用：

```bash
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 ping
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 set-mode ros_auto
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 status-watch --duration 5
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 buzzer-test
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 gripper-test
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 joint-test --joint 0 --delta 80
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 stop
```

一键低风险烟雾测试：

```bash
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 all-safe
```

如果暂时不想动机械臂：

```bash
rosrun tao_serial test_f407_v2_serial.py --port /dev/ttyS0 all-safe --skip-joint
```

验收标准：

```text
[ ] PING 能收到 PONG
[ ] SET_MODE ROS_AUTO 能收到 ACK 或状态变化
[ ] STATUS 能解析出 mode/error/battery/joints
[ ] STOP 任何时候都能让底盘停、机械臂停止继续动作
[ ] BASE_VEL 零速度持续发送时底盘不乱动
[ ] BUZZER 播放不阻塞主循环
[ ] GRIPPER 百分比开合方向正确
[ ] ARM_JOINTS 小角度动作方向正确，越界不打坏舵机
[ ] 关闭上位机心跳后，STM32 能在约 300-500 ms 内进入安全停
```

## 4. ROS topic 桥接验收

启动 `serial_test.launch` 后测试：

```bash
rostopic pub -1 /buzzer/play std_msgs/UInt8 "data: 1"
rostopic pub -1 /gripper/command std_msgs/UInt8 "data: 20"
rostopic pub -1 /gripper/command std_msgs/UInt8 "data: 70"
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

调试反馈：

```bash
rostopic echo /tao_serial/status_json
rostopic echo /tao_serial/ack_json
rostopic echo /tao_serial/error_json
```

验收标准：

```text
[ ] ROS topic 可以控制蜂鸣器
[ ] ROS topic 可以控制夹爪
[ ] ROS topic 可以控制底盘低速运动
[ ] /tao_serial/error_json 无持续错误
[ ] /tao_serial/status_json 能用于任务节点判断底盘/机械臂状态
```

## 5. MoveIt 真机链路验收

树莓派保持 `serial_test.launch` 正常运行。虚拟机/上位机启动：

```bash
export ROS_MASTER_URI=http://192.168.137.100:11311
export ROS_IP=192.168.137.2
cd ~/tao_robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tao_bringup moveit_real.launch
```

验收标准：

```text
[ ] RViz/MoveIt Plan 正常
[ ] Execute 后 /tao_serial/tx 出现 ARM_JOINTS
[ ] 树莓派串口日志出现 0x20 ARM_JOINTS 帧
[ ] 机械臂小角度动作方向正确
[ ] joint_map.yaml 修改 sign/offset 后，无需改代码即可重新校准
[ ] task_poses.yaml 中 home/pick/place/wave/stack 可作为后续任务参数源
```

## 6. 给视觉同学的接口边界

前三个任务验收通过后，可以把以下接口直接交给视觉同学调试。

视觉同学只需要输出检测结果，不要直接拼串口协议，不要直接操作 STM32：

```text
/vision/line/error              std_msgs/Float32   黑线中心误差，建议归一化到 -1..1
/vision/line/visible            std_msgs/Bool      是否看到线
/vision/intersection/detected   std_msgs/Bool      是否检测到路口
/vision/object/detected         std_msgs/Bool      是否看到物块
/vision/object/color            std_msgs/String    red/green/blue/unknown
/vision/object/offset_x         std_msgs/Float32   物块横向偏差，建议 -1..1
/vision/object/offset_y         std_msgs/Float32   物块纵向偏差，建议 -1..1
/vision/face/detected           std_msgs/Bool      是否检测到人脸
/vision/tag/id                  std_msgs/Int32     标签 ID，未识别为 -1
```

任务同学/主控节点负责把视觉结果转换为硬件动作：

```text
巡线/对准       -> /cmd_vel
蜂鸣器          -> /buzzer/play
夹爪            -> /gripper/command
机械臂          -> MoveIt action 或 task_poses.yaml named pose
异常/急停       -> /tao_serial/tx: STOP 或任务层 estop
```

因此，做好任务 1/2/3 后，视觉同学可以并行调 `line_node/object_node/face_node/tag_node`，他们不需要等任务状态机全部写完。

## 7. 仍需实机确认的安全项

代码已经提供接口和测试入口，但以下必须在真机上确认后才能称为 90%+：

```text
[ ] STM32 收不到 HEARTBEAT 后确实安全停
[ ] STOP 在底盘运动、机械臂运动、蜂鸣器播放时都最高优先级
[ ] STM32 对 ARM_JOINTS/GRIPPER/BASE_VEL 做限幅
[ ] BUZZER 非阻塞
[ ] STATUS/ACK/ERROR 与文档长度和字段完全一致
```
