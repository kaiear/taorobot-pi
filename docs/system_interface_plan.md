# Tao 机器人系统接口与任务落地计划

本文档记录当前项目从通信底座到完整任务系统的落地顺序。所有后续代码优先遵守 `docs/serial_protocol_v2.md`，不要让视觉、任务、MoveIt 或 STM32 各自定义新的硬件控制格式。

## 1. 系统边界

```text
STM32 下位机：硬件执行、安全保护、限幅、超时、状态反馈
ROS 上位机：底盘运动控制算法、机械臂轨迹、视觉、任务状态机、MoveIt 调度、串口桥接
MoveIt：机械臂轨迹规划，不直接控制底盘或视觉
视觉节点：输出检测结果，不直接控制硬件
任务节点：根据视觉和状态决定底盘、机械臂、蜂鸣器动作
```

核心原则：采用弱下位机架构。下位机不理解巡线、人脸、标签、码垛等高级任务，也不负责底盘路径规划、巡线控制、机械臂轨迹规划或 MoveIt 规划；下位机只接收上位机给出的目标速度、目标关节、夹爪、蜂鸣器和安全命令，并负责安全、实时地执行。

### 1.1 弱下位机分工

树莓派 / ROS 上位机负责“算”：

- 底盘运动控制算法：巡线误差到 `/cmd_vel`，导航速度规划，任务过程中的速度切换。
- 机械臂控制算法：MoveIt 轨迹规划、关节目标生成、动作序列和抓取流程。
- 任务决策：地图任务、人脸、物块、标签、码垛、蜂鸣器触发时机。
- 协议桥接：把 ROS topic / action 转成 v2 串口命令。

STM32F407 下位机负责“保底执行”：

- 串口帧解析、CRC 校验、`ACK/ERROR/STATUS` 反馈。
- `BASE_VEL` 到四轮电机输出的实时执行和限幅。
- `ARM_JOINTS/GRIPPER/BUZZER` 到舵机 / 蜂鸣器输出的实时执行和限幅。
- 急停、心跳超时、速度 / 加速度 / 关节范围限制、低电压等安全保护。

判断规则：如果逻辑需要理解任务、地图、视觉、轨迹或策略，放上位机；如果逻辑需要毫秒级保护、直接操作 PWM / 舵机 / 电机或断联后仍必须安全，放下位机。

## 2. 当前进度

已经完成：

- v2 串口协议文档初版。
- ROS 端 `serial_protocol.py` 编解码。
- STM32 端 v2 协议基础收发。
- `PING/PONG`、`STOP`、`SET_MODE ROS_AUTO`、`HEARTBEAT` 测试。
- `/cmd_vel -> BASE_VEL -> STM32 -> 四轮电机` 实车验证。

当前阶段结论：项目已经从“协议设计”进入“通信底座补齐和 ROS 桥接拆分”阶段。

## 3. 下一阶段优先级

### 3.1 通信底座补齐

目标：让 ROS 能稳定控制 STM32 的基础执行能力。

交付：

- `BUZZER` 测试：`/buzzer/play -> BUZZER`。
- `ARM_JOINTS` 测试：单关节低速运动，验证方向、限幅和单位。
- `GRIPPER` 测试：开合方向和百分比范围。
- `STATUS` 解析：发布底盘、机械臂、错误码、电压、关节镜像。
- `ACK/ERROR` 处理：调试时能明确看到命令是否被接受。

成功标准：不启动视觉、不启动 MoveIt 时，仅用 ROS topic 或测试脚本就能验证所有基础硬件能力。

### 3.2 ROS 基础桥接层

目标：把标准 ROS 接口转换为 v2 协议命令，并逐步把运动控制算法保留在 ROS 侧。

建议节点边界：

```text
tao_serial_node       串口收发、协议编解码、状态发布
tao_base_bridge       /cmd_vel -> BASE_VEL，后续承接巡线 / 导航输出的速度目标
tao_buzzer_bridge     /buzzer/play -> BUZZER
tao_joint_state       STATUS 关节镜像 -> /joint_states
tao_arm_trajectory_server  FollowJointTrajectory -> ARM_JOINTS/GRIPPER
```

第一版可以先保留在 `tao_serial_node.py` 内部集成，等 `BUZZER/ARM/STATUS` 都跑通后再拆分，避免过早拆节点增加调试复杂度。

### 3.3 MoveIt 真机机械臂链路

目标：机械臂统一通过 MoveIt 控制。

交付：

- `tao_description` / URDF。
- `tao_moveit_config`。
- `joint_map.yaml`。
- `joint_limits.yaml`。
- `tao_arm_trajectory_server`。
- `/joint_states` 发布。

成功标准：MoveIt 能执行 `home`、单关节、`wave`、`pick/place` 等低速动作。

### 3.4 任务层顺序

任务按依赖从低到高推进：

1. 地图巡线和三物块归位。
2. 人脸识别、蜂鸣器、挥手。
3. 机械臂视觉跟随物块。
4. 标签识别和码垛。
5. 现场调参、dry-run、rosbag 和 debug image。

## 4. ROS 接口约定

底盘：

```text
订阅：/cmd_vel geometry_msgs/Twist
发布：/tao_base/state
```

蜂鸣器：

```text
订阅：/buzzer/play std_msgs/UInt8
```

机械臂：

```text
Action：/arm_controller/follow_joint_trajectory
Action：/gripper_controller/follow_joint_trajectory
发布：/joint_states sensor_msgs/JointState
```

视觉：

```text
/vision/line/error std_msgs/Float32
/vision/line/visible std_msgs/Bool
/vision/intersection/detected std_msgs/Bool
/vision/object/detected std_msgs/Bool
/vision/object/color std_msgs/String
/vision/object/offset_x std_msgs/Float32
/vision/object/offset_y std_msgs/Float32
/vision/face/detected std_msgs/Bool
/vision/tag/id std_msgs/Int32
```

任务：

```text
/task/mode std_msgs/String
/task/state std_msgs/String
/task/error std_msgs/String
/task/estop std_msgs/Bool
```

## 5. 配置文件规划

后续应逐步增加：

```text
config/serial.yaml              串口、波特率、心跳、超时
config/joint_map.yaml           关节 index/sign/offset/limit
config/control.yaml             巡线、转弯、跟随控制参数
config/vision_thresholds.yaml   视觉阈值
config/route_plan.yaml          地图路口动作表
config/task_poses.yaml          home/wave/pick/place/stack joint pose
```

现场原则：光照、路线、位姿、速度和限幅优先改 YAML，不现场改代码。

## 6. 推荐下一步执行

下一步不要直接做巡线或 MoveIt 大功能，先做通信底座剩余项：

1. 给 `tao_serial_node.py` 增加 `/buzzer/play` 到 `BUZZER` 的桥接。
2. 给测试脚本增加 `buzzer-test`、`gripper-test`、`joint-test`。
3. 确认 STM32 对 `STATUS/ACK/ERROR` 的反馈格式和 ROS 解析一致。
4. 在树莓派上逐项验收，再进入 MoveIt 真机链路。

后续写代码时按如下边界推进：

1. 先让 STM32 稳定执行 `BASE_VEL/ARM_JOINTS/GRIPPER/BUZZER`，并反馈 `STATUS/ACK/ERROR`。
2. 再在 ROS 侧实现底盘控制算法，例如巡线 PID 输出 `/cmd_vel`。
3. 再在 ROS 侧实现机械臂轨迹服务器，把 MoveIt 的 `FollowJointTrajectory` 转成 `ARM_JOINTS`。
4. 最后在任务节点中组合视觉、底盘、机械臂和蜂鸣器，不把任务逻辑下沉到 STM32。