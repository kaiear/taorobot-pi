# 视觉同学交接说明

本文档用于说明当前 STM32 下位机、树莓派 ROS1 上位机、视觉控制机械臂之间的交接状态。

## 结论

当前可以把板子交给视觉同学继续开发，但需要说明清楚：

- STM32 板子到树莓派的串口通信已经打通。
- 树莓派 ROS 端已经可以接收 STM32 遥测帧。
- 树莓派 ROS 端发送 `PING`，STM32 可以回复 `PONG`，说明双向链路正常。
- 目前还没有完整的“视觉控制机械臂”上层接口。
- 视觉控制机械臂前，需要继续定义并实现 ROS 到 STM32 的机械臂控制命令协议。

## 硬件连接

- STM32 侧：USART2，`PD5` / `PD6`。
- 树莓派侧：40-pin 串口引脚，当前 Linux 串口设备为 `/dev/ttyS0`。
- 串口参数：`115200 8N1`。
- 电平：3.3 V TTL。
- 接线要求：TX/RX 交叉连接，并且必须共地。

更完整的串口协议说明见 `docs/serial_link_protocol.md`。

## 树莓派工程位置

树莓派上的 ROS 工作空间建议位置：

```bash
/home/ubuntu/tao_robot_ws
```

源码目录：

```bash
/home/ubuntu/tao_robot_ws/src
```

串口节点位置：

```bash
/home/ubuntu/tao_robot_ws/src/tao_serial/scripts/tao_serial_node.py
```

启动文件位置：

```bash
/home/ubuntu/tao_robot_ws/src/tao_bringup/launch/serial_test.launch
```

## 启动串口测试

在树莓派上执行：

```bash
cd /home/ubuntu/tao_robot_ws
source devel/setup.bash
roslaunch tao_bringup serial_test.launch port:=/dev/ttyS0 open_serial:=true
```

如果串口、权限、接线都正确，ROS 端会开始接收 STM32 遥测数据。

## 查看 STM32 遥测

查看 ROS 接收到的数据：

```bash
rostopic echo /tao_serial/rx
```

当前遥测内容会以字符串摘要形式输出，例如：

```text
stm_data acc=(...) gyro=(...) vel=(...) arm=(...) bat_x100=...
```

其中：

- `acc`：IMU 加速度。
- `gyro`：IMU 陀螺仪。
- `vel`：速度字段，当前按 `x1000` 缩放。
- `arm`：机械臂相关反馈字段，目前需要继续确认每个值对应的关节/舵机含义。
- `bat_x100`：电池电压乘 100，目前如果显示 0，说明该字段可能还没有接入或没有有效数据。

## 测试 ROS 到 STM32

发送一次 `PING`：

```bash
rostopic pub -1 /tao_serial/tx std_msgs/String "data: 'PING'"
```

如果 STM32 回复 `PONG`，说明树莓派到 STM32 的发送链路正常。

## 当前已经具备的能力

- 可以证明 STM32 到树莓派的串口遥测链路正常。
- 可以证明树莓派到 STM32 的串口发送链路正常。
- ROS 端已经有最小串口桥接节点。
- 后续视觉节点可以基于 ROS topic 接入，不需要直接操作串口底层。

## 当前还缺的内容

视觉同学如果要实现“视觉识别后控制机械臂”，还需要补齐以下内容：

1. 定义 ROS 侧机械臂控制 topic，例如 `/tao_arm/cmd`。
2. 定义 ROS 到 STM32 的机械臂控制命令帧。
3. STM32 端实现对应命令解析。
4. STM32 端将命令转换为舵机/PWM/机械臂动作。
5. 明确每个机械臂关节或舵机的编号、角度范围、零位、安全限位。
6. 增加急停、超时停止、限幅等安全保护。

## 推荐的软件结构

建议视觉同学不要直接在视觉代码里写串口协议，而是按下面结构开发：

```text
camera / vision_node
        ↓
识别目标位置
        ↓
发布机械臂目标 /tao_arm/cmd
        ↓
arm_control_node 转换为关节/舵机目标
        ↓
tao_serial_node 打包串口命令
        ↓
STM32 解析命令并驱动机械臂
```

这样可以把视觉、运动控制、串口通信、下位机执行分开，后续更容易调试。

## 初期建议控制协议

调试早期可以先使用文本协议，便于串口助手和 `rostopic pub` 测试：

```text
SET_SERVO <id> <angle>
```

示例：

```text
SET_SERVO 1 90
```

机械臂多关节可以先约定为：

```text
ARM <joint1> <joint2> <joint3> <joint4> <joint5> <gripper>
```

示例：

```text
ARM 1500 1500 1500 1500 1500 1000
```

其中数值单位需要由上下位机共同确认，可以是舵机 PWM 微秒值、角度值，或者编码器目标值。

正式运行时建议升级为二进制帧，例如：

```text
AA 55 CMD LEN PAYLOAD CHECKSUM 7D
```

这样比纯文本更稳定，也更适合持续控制。

## 视觉同学需要确认的问题

交接时建议重点确认：

- 视觉输出的是目标像素坐标、相机坐标，还是机械臂末端目标位姿。
- 是否已有相机标定和手眼标定。
- 机械臂控制是关节空间控制，还是末端位置控制。
- 每个舵机/关节的安全范围是多少。
- 夹爪开合命令如何表示。
- 控制频率需要多高。
- 视觉丢目标时机械臂应该保持、回 home，还是停止。
- 串口命令是否需要 ACK、错误码和超时重发。

## 可以直接转告视觉同学的话

```text
板子和树莓派串口已经打通，ROS 端能收到 STM32 遥测帧，也能向 STM32 发送 PING 并收到 PONG。
现在可以基于这条链路继续做视觉控制机械臂，但当前还没有完整的机械臂控制协议。
建议你在 ROS 侧新增视觉节点，输出机械臂目标 topic，再由控制节点和串口节点把命令发给 STM32。
需要你和下位机一起确认机械臂命令格式、关节编号、角度/脉宽范围、安全限位和 ACK 机制。
```

## 交接状态

- 串口物理链路：已验证。
- STM32 到 ROS 遥测：已验证。
- ROS 到 STM32 `PING/PONG`：已验证。
- 机械臂反馈字段：已有，但字段语义还需要确认。
- 视觉控制机械臂：可以开始开发，但控制协议和执行逻辑还需要补齐。