# 巡线抓取分拣第二阶段实车测试计划

本测试计划用于验证当前“巡线逻辑保持不变，机械臂抓取/放置作为 cross 流程夹层动作插入”的实现。本文档不引入速度斜坡、不引入抓取结果二次判断、不引入抓取失败重试。

## 1. 启动前检查

### 目的

确认 ROS 参数、串口、MoveIt action、相机输入处于可测试状态。

### 步骤

1. 确认实车电源、急停、机械臂活动空间安全。
2. 确认 `src/tao_tasks/config/control.yaml` 中：
   - `enable_pick_place: true`
   - `pick_crosses: [2, 5, 9]`
   - `post_pick_uturn_crosses: [2, 5, 9]`
   - `uturn_crosses: []`
   - `use_tx_command: true`
3. 确认 `src/tao_bringup/config/joint_map.yaml` 中 `protocol_joint_count: 6`。
4. 启动前先让底盘悬空或关闭 `publish_cmd_vel` 做空跑。

### 预期结果

- 无明显 MoveIt `hand` group / `arm_5_joint` mismatch 警告。
- line_follow_controller 能启动并持续输出调试日志。
- 串口节点能看到心跳或 `TX v2` 日志。

## 2. 串口链路检查

### 目的

确认机械臂相关指令能够从 ROS 到达串口节点。

### 步骤

1. 启动 `tao_serial_node.py`，建议临时打开：

   ```yaml
   log_tx: true
   log_rx: true
   ```

2. 手动发送夹爪命令：

   ```bash
   rostopic pub -1 /gripper/command std_msgs/UInt8 "data: 20"
   rostopic pub -1 /gripper/command std_msgs/UInt8 "data: 70"
   ```

3. 手动发送一帧 ARM_JOINTS 文本命令：

   ```bash
   rostopic pub -1 /tao_serial/tx std_msgs/String "data: 'ARM_JOINTS 1 0 0 0 0 0 0 500'"
   ```

### 预期结果

- 日志出现 `gripper topic -> GRIPPER percent=...`。
- 日志出现 `TX v2: ...`。
- 下位机执行对应夹爪/机械臂动作，或至少 ACK/状态帧有响应。

### 异常排查

- 没有日志：检查 topic 名称是否与 `serial.yaml` 一致。
- 有日志无动作：检查串口端口、波特率、`open_serial`、下位机模式。
- `ARM_JOINTS requires 6 values`：说明发到 `/tao_arm/joints_protocol_units` 的数组长度错误；文本 `/tao_serial/tx` 应使用 6 个 joint 协议值。

## 3. MoveIt action 链路检查

### 目的

确认 MoveIt 只规划 5 轴主臂，并通过 `tao_moveit_bridge` 转换为 6 值串口协议。

### 步骤

1. 启动 `moveit_real.launch`。
2. 观察 `tao_moveit_bridge` 是否启动：

   ```text
   /arm_controller/follow_joint_trajectory
   ```

3. 确认 `control.yaml` 中：

   ```yaml
   moveit_arm_joint_names: [arm_0_joint, arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint]
   ```

### 预期结果

- MoveIt group 为 `arm`。
- action 名称与 `moveit_action_name` 对齐。
- 不出现 `hand` group kinematics/config missing 类警告。

## 4. cross=2/5/9 抓取夹层验证

### 目的

验证抓取动作只作为 cross 命中后的夹层，不破坏原巡线/cross 逻辑。

### 步骤

1. 放置色块，运行巡线。
2. 让车辆依次经过 cross=1、cross=2。
3. 在 cross=2 观察：
   - 底盘停车；
   - 进入色块对准/抓取流程；
   - 发送 `ARM_JOINTS` 和 `GRIPPER`；
   - 抓取结束后执行 post-pick U-turn；
   - 回到正常巡线。
4. cross=5、cross=9 重复同样检查。

### 预期结果

- cross=2/5/9 不再跳过抓取。
- 抓取结束后才掉头。
- 掉头后继续原巡线流程。

### 异常排查

- 直接掉头未抓取：检查 `pick_crosses`、`enable_pick_place`、`uturn_crosses` 是否配置正确。
- 停车后不动：检查色块识别日志、`pick_align_timeout_ticks`、串口 TX 日志。
- 有抓取动作但下位机没动：按“串口链路检查”排查。

## 5. cross=3/6/10 放置/分拣验证

### 目的

验证放置逻辑按现有 C++ 策略执行，例如 cross=3 后准备分拣，机械臂左转 90 度，放下物块后归正并继续巡线。

### 步骤

1. 先完成一次 cross=2 抓取。
2. 继续运行到 cross=3。
3. 观察车辆是否按现有分拣流程停车/转向/放置。
4. 验证放下后机械臂归正，车辆继续巡线。
5. cross=6、cross=10 按同样方式验证。

### 预期结果

- 放置流程与 C++ 逻辑策略一致。
- 放置后不会执行额外 U-turn。
- 机械臂归正后继续巡线。

## 6. 视觉超时兜底验证

### 目的

确认识别不到色块时不会永久停车。

### 步骤

1. 在 cross=2 前移除或遮挡色块。
2. 观察抓取对准阶段日志。
3. 等待超过 `pick_align_timeout_ticks`。

### 预期结果

- 日志提示识别/对准超时。
- 根据当前配置执行 skip 或后退/恢复流程。
- 不出现永久停车。

## 7. 全流程回归测试

### 目的

验证一次完整路线中的抓取、掉头、放置、继续巡线是否连贯。

### 步骤

1. 低速运行，建议先保持 `cpp_base_speed: 0.12` 或更低。
2. 完整通过 cross=2、3、5、6、9、10、13。
3. 记录每个 cross 的日志和动作。

### 预期结果

- cross=2/5/9：抓取后掉头。
- cross=3/6/10：放置后归正继续巡线。
- cross=13：停止。

## 8. 快速定位表

| 现象 | 优先检查 |
| --- | --- |
| 启动出现 hand group 警告 | `ompl_planning.yaml`、`stomp_planning.yaml` 是否仍有 hand 配置 |
| MoveIt joint mismatch | `simple_moveit_controllers.yaml`、`control.yaml`、`joint_map.yaml` 的 5 轴/6 协议值边界 |
| 串口无动作 | `open_serial`、端口、波特率、`log_tx`、下位机模式 |
| cross=2/5/9 未抓取 | `enable_pick_place`、`pick_crosses`、`uturn_crosses` |
| 抓取后未掉头 | `post_pick_uturn_crosses`、`post_pick_uturn_delay` |
| 放置后不继续巡线 | place 超时参数、放置状态结束日志、cross ignore ticks |
