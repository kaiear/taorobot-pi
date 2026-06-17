# MoveIt / controller / serial 配置低风险清理记录

本次清理目标是减少启动警告和关节名不匹配风险，同时不改变当前巡线、抓取、放置状态机，不改变串口协议，不修改 URDF/SRDF 核心结构。

## 清理范围

检查目录：

- `src/yeahbot_c1_moveit_config/config/`
- `src/tao_bringup/config/`
- `src/tao_serial/scripts/`
- `src/tao_tasks/config/`

重点核对：

- MoveIt planning group 是否只使用实际控制的 `arm` 组；
- controller joint list 是否只包含 `arm_0_joint` 到 `arm_4_joint`；
- `/joint_states` 是否避免发布 MoveIt 不控制的虚拟/夹爪残留关节；
- `tao_moveit_bridge` 与 `control.yaml` 的 action 名称和 5 轴主臂配置是否一致。

## 已修改项

### 1. 删除 OMPL 中残留的 `hand` planning group 配置

文件：`src/yeahbot_c1_moveit_config/config/ompl_planning.yaml`

原因：当前 `yeahbot_c1.srdf` 只定义了 `arm` group，未定义 `hand` group；`kinematics.yaml` 也只包含 `arm`。保留 `hand` 的 planner 配置没有实际作用，容易造成 MoveIt 参数加载时的无效 group 干扰。

### 2. 删除 STOMP 中残留的 `stomp/hand` 配置

文件：`src/yeahbot_c1_moveit_config/config/stomp_planning.yaml`

原因同上。当前抓取流程的夹爪开合走 `/gripper/command` 和串口 `GRIPPER` 指令，不依赖 MoveIt `hand` group 规划。

### 3. `/joint_states` 默认发布关节收敛为 5 个主臂关节

文件：

- `src/tao_bringup/config/serial.yaml`
- `src/tao_serial/scripts/tao_serial_node.py`

原因：MoveIt 主臂只控制：

```text
arm_0_joint
arm_1_joint
arm_2_joint
arm_3_joint
arm_4_joint
```

此前默认还发布 `arm_5_joint`，但当前 SRDF/MoveIt controller 不控制该 joint，容易产生状态名与规划组不一致的隐性干扰。

注意：这不改变下位机 v2 串口协议。`ARM_JOINTS` 仍保持 6 个协议值，与 `joint_map.yaml` 中的 `protocol_joint_count: 6` 一致。只是 ROS `/joint_states` 只发布 MoveIt 实际关心的 5 个主臂 joint。

## 已确认未修改项

### `simple_moveit_controllers.yaml`

已经是 5 轴主臂控制器：

```text
arm_0_joint ... arm_4_joint
```

无需修改。

### `ros_controllers.yaml`

已经是 5 轴主臂控制器，无 `hand_controller`，无需修改。

### `kinematics.yaml`

只包含 `arm` group，无残留 `hand`，无需修改。

### `joint_map.yaml`

保留 `protocol_joint_count: 6`。

原因：这是串口 `ARM_JOINTS <seq> <j1> ... <j6> <duration_ms>` 协议需要的 6 个值，不等同于 MoveIt 控制的 5 个 ROS joint。

### `yeahbot_c1.srdf`

不修改。

原因：其中 `arm_5_*_link` 和 wheel link 主要出现在 collision matrix 内，属于 URDF/SRDF 几何碰撞语义，不是规划 group 或 controller 残留。按低风险原则，不动核心结构。

## 与机械臂串口接收的关系

当前任务链路仍保持：

```text
line_follow_controller.py
  -> /tao_serial/tx: ARM_JOINTS seq j1 j2 j3 j4 j5 j6 duration_ms
  -> tao_serial_node.py
  -> serial_protocol.encode_arm_joints(...)
  -> STM32/F407
```

或 MoveIt 路径：

```text
MoveIt arm group
  -> /arm_controller/follow_joint_trajectory
  -> tao_moveit_bridge.py
  -> /tao_serial/tx 或 /tao_arm/joints_protocol_units
  -> tao_serial_node.py
```

本次只清理 MoveIt/ROS 状态层面的残留项，不改变 `ARM_JOINTS` 和 `GRIPPER` 的编码格式，因此不会破坏下位机接收。
