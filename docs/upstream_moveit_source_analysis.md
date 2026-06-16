# 厂家 ROS1 上位机源码分析与迁移建议

## 分析目标

本文记录厂家上位机源码中与 MoveIt、机械臂、视觉抓取相关的可复用内容，明确哪些模块可以直接迁移，哪些模块只能作为参考，避免把厂家工程整包复制到当前项目后造成串口协议、关节方向、相机标定和控制边界混乱。

## 源码范围

已分析的主要路径来自厂家上位机源码：

```text
src/roscar_control
src/cam_control
src/yeahbot_c1
src/yeahbot_c1_moveit_config
src/rplidar_ros
```

其中当前项目最相关的是：

- `yeahbot_c1`：机器人 URDF、mesh、坐标系和关节模型。
- `yeahbot_c1_moveit_config`：MoveIt 配置包。
- `roscar_control`：ROS 话题、MoveIt 轨迹 action 与串口协议之间的桥接。
- `cam_control`：颜色识别、AprilTag、手写逆运动学和抓取状态机。

## MoveIt 相关模块

### `yeahbot_c1`

该包是机器人模型包，核心内容是：

```text
yeahbot_c1/urdf/yeahbot_c1.urdf
yeahbot_c1/meshes/*.STL
```

模型中包含底盘、轮子、机械臂、夹爪、摄像头、IMU、雷达等坐标系。MoveIt 主要依赖机械臂和夹爪部分。

机械臂主体关节：

```text
arm_0_joint
arm_1_joint
arm_2_joint
arm_3_joint
arm_4_joint
```

夹爪关节：

```text
arm_5_1_joint
arm_5_2_joint
arm_5_3_joint
arm_5_4_joint
arm_5_5_joint
arm_5_6_joint
```

该包可作为当前项目 MoveIt 建模的主要参考，但需要确认实际机械臂连杆尺寸、舵机安装方向和零位是否与厂家模型一致。

### `yeahbot_c1_moveit_config`

该包是 MoveIt Setup Assistant 生成的配置包，核心内容包括：

```text
config/yeahbot_c1.srdf
config/joint_limits.yaml
config/kinematics.yaml
config/simple_moveit_controllers.yaml
launch/demo.launch
launch/move_group.launch
```

SRDF 中定义了两个关键规划组：

```text
arm  = arm_0_joint ~ arm_4_joint
hand = arm_5_1_joint ~ arm_5_6_joint
```

命名姿态可参考复用：

```text
home
start
put
open
close
init
```

`kinematics.yaml` 使用 TRAC-IK：

```text
trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
```

如果目标树莓派或 Ubuntu 20.04 ROS1 环境未安装 TRAC-IK，MoveIt 可能无法正常求解 IK。迁移时需要优先确认依赖，或者切换为 KDL IK 插件进行初步验证。

## `roscar_control` 的作用

`roscar_control/src/roscar_control.cpp` 是厂家工程中最关键的真实硬件桥接节点。它主要完成：

- 订阅 `/cmd_vel`，转换成底盘串口速度帧。
- 订阅 `/arm_states`，转换成机械臂关节串口帧。
- 订阅 `/ik_states`，转换成机械臂关节加执行时间串口帧。
- 提供 `arm_controller/follow_joint_trajectory` action server。
- 提供 `hand_controller/follow_joint_trajectory` action server。
- 发布 `/joint_states`，供 MoveIt 获取当前关节状态。
- 发布 `/odom`、`/imu`、`/PowerVoltage`。

MoveIt 到真实机械臂的链路是：

```text
MoveIt 规划轨迹
  -> FollowJointTrajectory action
  -> roscar_control::execute_Callback()
  -> 串口帧
  -> STM32 执行舵机动作
```

厂家协议中，机械臂 MoveIt 轨迹帧大致为：

```text
AA 55 0F 70 joint0_h joint0_l ... joint4_h joint4_l checksum
```

夹爪 MoveIt 轨迹帧大致为：

```text
AA 55 11 60 joint0_h joint0_l ... joint5_h joint5_l checksum
```

角度转换规则是：

```text
int16_angle = joint_position_rad * 1000
checksum = 前面所有字节累加低 8 位
```

这部分不建议整文件直接搬迁，因为当前项目的 F407 固件、串口设备名、协议类型、舵机方向和安全策略可能不同。建议只复用设计模式：

```text
FollowJointTrajectory action server
  -> 当前项目 tao_arm_bridge
  -> 当前项目 serial_link_protocol
  -> F407 USART2 协议
```

## `cam_control` 视觉抓取模块

### `color_follow_arm.py`

该脚本订阅 `/usb_cam/image_raw`，用 OpenCV 识别红色目标，根据图像中心误差调整机械臂目标位置，然后调用手写 `kinematics_analysis()` 计算关节角，并发布 `/ik_states`。

可复用点：

- ROS 图像转 OpenCV 图像流程。
- HSV 颜色识别流程。
- 图像中心对准逻辑。
- 手写逆运动学结构。
- `/ik_states` 的 `JointState` 发布形式。

限制：

- 默认只识别红色。
- 未输出真实三维坐标。
- 强依赖固定相机安装位置和厂家机械臂尺寸。
- 不使用 MoveIt 规划和避障。

### `color_sorting.py`

该脚本实现颜色分拣状态机，识别红、绿、蓝色块，先对准目标，再执行抓取、抬起、移动、放置流程。

可复用点：

- HSV 阈值分色。
- 抓取状态机设计。
- 对准目标后的计数防抖。
- 夹爪开合动作序列。
- 固定放置区策略。

限制：

- 动作参数是厂家实机调参结果。
- 依赖 `/ik_states` 和厂家串口协议。
- 没有深度信息和通用位姿估计。

### `label_palletizing.py`

该脚本使用 AprilTag 识别目标，读取 tag ID、中心点和角度，然后执行抓取与码垛流程。

可复用点：

- AprilTag 识别流程。
- 标签角度计算。
- 根据目标角度旋转夹爪。
- 多目标、多层放置状态机。

相比单纯颜色识别，AprilTag 更适合比赛、实验和结构化场景。

## 迁移边界

### 可以优先复用

- `yeahbot_c1` 的 URDF 建模思路和 mesh 组织方式。
- `yeahbot_c1_moveit_config` 的 SRDF 分组、命名姿态、关节限制和控制器配置思路。
- `roscar_control` 的 FollowJointTrajectory action server 到串口桥接模式。
- `roscar_control` 的 `/joint_states` 发布模式。
- `cam_control` 的抓取状态机、颜色识别、AprilTag 识别和手写 IK 参考。

### 需要重写或适配

- 串口设备名，例如厂家默认 `/dev/ttl`，当前项目可能是 `/dev/ttyS0`、`/dev/ttyAMA0` 或 USB 串口。
- 串口协议类型码、帧长度、校验规则和 F407 固件解析逻辑。
- 关节角正方向、零点、限幅和角度到 PWM 映射。
- MoveIt 输出关节名与当前 F407 舵机编号的映射。
- 摄像头坐标系、相机内参、手眼标定和目标坐标转换。
- 安全策略，例如超时停车、低速首测、急停、舵机软限位。

## 推荐落地路线

### 第一阶段：先打通非 MoveIt 机械臂控制

目标是快速让上位机能可靠控制 F407 机械臂。

```text
视觉或键盘节点
  -> 生成 joints[0..5] + time
  -> tao_arm_bridge
  -> tao_serial
  -> F407
```

建议先实现：

- 机械臂 Home。
- 单关节低速正反方向测试。
- 夹爪打开/闭合。
- `/ik_states` 或等价话题到串口帧。
- 当前关节状态发布 `/joint_states`。

### 第二阶段：迁移 MoveIt 模型与配置

目标是让 MoveIt 在仿真/RViz 中能规划机械臂。

建议步骤：

1. 复制或重建 `yeahbot_c1` 模型包。
2. 复制或重建 `yeahbot_c1_moveit_config`。
3. 启动 `demo.launch` 验证 RViz 规划。
4. 确认 `arm` 和 `hand` group 可用。
5. 校验命名姿态 `home`、`start`、`open`、`close`。

### 第三阶段：实现 MoveIt 到 F407 桥接

目标是让 MoveIt 真实控制机械臂。

需要新增或扩展：

```text
tao_arm_trajectory_server
```

职责：

- 提供 `arm_controller/follow_joint_trajectory`。
- 提供 `hand_controller/follow_joint_trajectory`。
- 校验轨迹点数量、关节名和限幅。
- 将 `JointTrajectoryPoint.positions` 转为当前串口协议。
- 分点发送给 `tao_serial`。
- 超时或 preempt 时发送安全停止/保持帧。

### 第四阶段：视觉 + MoveIt 抓取

最终链路：

```text
/usb_cam/image_raw
  -> 颜色 / AprilTag / YOLO 识别
  -> 目标在 camera_link 下的位置和姿态
  -> TF 转到 base_link
  -> MoveIt PoseStamped 目标
  -> FollowJointTrajectory
  -> tao_arm_trajectory_server
  -> F407
```

该阶段必须做相机标定和手眼标定，否则视觉坐标无法稳定转换成机械臂可抓取的目标位姿。

## 当前项目建议

当前项目不建议整包搬迁厂家 ROS 工程。更安全的路线是：

1. 保持 F407 固件作为执行层。
2. 按 `docs/ros1_migration_plan.md` 的包结构逐步建立 ROS1 上位机。
3. 先用厂家 `cam_control` 思路实现简单视觉抓取。
4. 再迁移 MoveIt 模型和配置。
5. 最后实现 `FollowJointTrajectory` 到当前串口协议的桥接。

这样可以避免 MoveIt、视觉算法、串口协议、F407 控制逻辑同时变化，降低调试风险。