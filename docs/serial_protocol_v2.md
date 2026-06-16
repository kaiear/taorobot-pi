# Tao 机器人串口协议 v2

本文档定义 ROS / MoveIt 上位机与 STM32F407 下位机之间的二进制 UART 通信协议。

## 0. 当前冻结状态

本协议是后续底盘、机械臂、蜂鸣器、视觉任务和 MoveIt 真机控制的第一版通信底座。当前已经完成并实车验证：

- `PING/PONG` 链路测试。
- `STOP` 安全停止命令。
- `SET_MODE ROS_AUTO` 自动模式切换。
- `HEARTBEAT` 心跳发送。
- `/cmd_vel` 到 `BASE_VEL` 的 ROS 桥接。
- 四轮底盘低速转动验证。

第一版协议冻结原则：

- STM32 下位机只理解速度、关节、夹爪、蜂鸣器、停止、模式、心跳和状态反馈。
- 项目采用弱下位机架构：底盘运动控制算法、机械臂轨迹规划、任务策略和视觉决策放在 ROS 上位机。
- 巡线、人脸、物块、标签、码垛等高级任务全部保留在 ROS 上位机。
- MoveIt 只通过 `FollowJointTrajectory` 控制机械臂，不直接操作串口。
- 视觉节点只发布检测结果，不直接控制底盘、机械臂或蜂鸣器。
- 后续功能优先扩展 ROS 桥接节点，不改变已经冻结的帧格式和基础命令语义。

## 1. 适用范围

- 上位机：树莓派 / Linux ROS 节点，负责导航、视觉、MoveIt 规划、任务决策和串口命令生成。
- 下位机：STM32F407，负责电机 / 舵机执行、实时安全限制、状态反馈和链路看门狗处理。
- 设计目标：让 STM32 逻辑保持简单、确定，同时让帧格式足够可靠，能适应电机噪声较大的 UART 环境。

本协议只传递执行目标和反馈状态，不承载高层算法。上位机可以持续发送速度目标、关节目标和任务状态；下位机必须对这些目标做实时限幅和安全保护，但不反推巡线、导航、抓取或码垛策略。

## 2. 帧格式

所有 v2 帧统一使用如下格式：

```text
AA LEN TYPE PAYLOAD CRC8 BB
```

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| `AA` | 1 字节 | 帧头，固定为 `0xAA` |
| `LEN` | 1 字节 | 仅表示负载长度，不包含 `TYPE` |
| `TYPE` | 1 字节 | 消息类型 |
| `PAYLOAD` | `LEN` 字节 | 消息数据 |
| `CRC8` | 1 字节 | 对 `LEN + TYPE + PAYLOAD` 计算 CRC-8/MAXIM |
| `BB` | 1 字节 | 帧尾，固定为 `0xBB` |

规则：

- `LEN = PAYLOAD 字节数`。
- `TOTAL_FRAME_LEN = LEN + 5`.
- CRC 输入字节严格为 `LEN`、`TYPE` 和所有 `PAYLOAD` 字节。
- CRC 不包含 `AA`、`CRC8`、`BB`。
- 所有多字节整数均使用小端序。
- v2 第一阶段最大负载长度为 `64` 字节，除非上下位机明确同步调整。

## 3. CRC-8/MAXIM

STM32F407 和上位机软件都使用软件方式实现 CRC-8。本协议不使用 STM32F407 硬件 CRC 外设。

| 参数 | 取值 |
| --- | --- |
| 名称 | CRC-8/MAXIM, CRC-8/DALLAS |
| 多项式 | `0x31` |
| 初始值 | `0x00` |
| RefIn | `True` |
| RefOut | `True` |
| XorOut | `0x00` |
| 校验输入 | `"123456789"` |
| 校验结果 | `0xA1` |

如果使用右移位实现，需要使用反射多项式 `0x8C`。

参考 C 实现：

```c
uint8_t tao_crc8_maxim_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x01) {
            crc = (crc >> 1) ^ 0x8C;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}
```

单帧 CRC 计算流程：

```text
crc = 0x00
crc = update(crc, LEN)
crc = update(crc, TYPE)
for byte in PAYLOAD:
    crc = update(crc, byte)
```

## 4. 类型范围

| 范围 | 方向 | 含义 |
| --- | --- | --- |
| `0x00` - `0x7F` | 上位机 -> STM32 | 命令 |
| `0x80` - `0xFF` | STM32 -> 上位机 | 反馈 / 响应 |

## 5. 单位与编码

| 数据 | 类型 | 单位 / 缩放 |
| --- | --- | --- |
| `vx`, `vy` | `i16` | `m/s * 1000` |
| `wz` | `i16` | `rad/s * 1000` |
| 关节角 | `i16` | `rad * 1000` |
| 持续时间 | `u16` | 毫秒 |
| 电池电压 | `u16` | 毫伏 |
| 夹爪 | `u8` | 百分比，`0` 表示张开，`100` 表示闭合 |

v2 第一阶段 `JOINT_COUNT` 固定为 `6`。如果真实机械臂关节数更少，未使用关节必须发送 `0`，STM32 侧忽略这些关节。

## 6. 上位机 -> STM32 命令

| 类型 | 名称 | LEN | 负载 |
| --- | --- | ---: | --- |
| `0x00` | `STOP` | 0 | 无 |
| `0x01` | `SET_MODE` | 1 | `mode u8` |
| `0x02` | `PING` | 4 | `time_ms u32` |
| `0x10` | `BASE_VEL` | 6 | `vx_i16, vy_i16, wz_i16` |
| `0x20` | `ARM_JOINTS` | 15 | `seq u8, joint0_i16 ... joint5_i16, duration_ms_u16` |
| `0x21` | `GRIPPER` | 1 | `gripper_percent u8` |
| `0x22` | `ARM_PRESET` | 1 | `preset_id u8` |
| `0x30` | `BUZZER` | 2 | `melody_id u8, repeat u8` |
| `0x40` | `HEARTBEAT` | 1 | `state u8` |

### 模式

| 取值 | 名称 | 行为 |
| --- | --- | --- |
| `0x00` | `MANUAL` | 手动 / 遥控模式 |
| `0x01` | `ROS_AUTO` | 接收自动运动命令 |
| `0x02` | `ESTOP` | 急停锁定 |
| `0x03` | `SAFE_IDLE` | 安全空闲，不执行自动运动 |

STM32 上电默认必须是 `SAFE_IDLE` 或 `ESTOP`，不能默认为 `ROS_AUTO`。

### 预设动作

| 取值 | 名称 |
| --- | --- |
| `0` | `HOME` |
| `1` | `SAFE_UP` |
| `2` | `PICK_READY` |
| `3` | `PLACE_READY` |
| `4` | `WAVE_SIMPLE` |
| `5` | `OPEN_GRIPPER` |
| `6` | `CLOSE_GRIPPER` |

### 心跳状态

| 取值 | 名称 |
| --- | --- |
| `0` | `IDLE` |
| `1` | `MAP_RUN` |
| `2` | `FACE_GREETING` |
| `3` | `OBJECT_TRACK_ARM` |
| `4` | `TAG_PALLETIZING` |
| `5` | `ESTOP` |

## 7. STM32 -> 上位机反馈

| 类型 | 名称 | LEN | 负载 |
| --- | --- | ---: | --- |
| `0x80` | `STATUS` | 22 | `mode u8, base_state u8, arm_state u8, buzzer_state u8, error_code u16, battery_mv u16, joint_count u8, joint0_i16 ... joint5_i16, last_arm_seq u8` |
| `0x81` | `ACK` | 2 | `ack_type u8, result u8` |
| `0x82` | `ERROR` | 3 | `error_code u16, detail u8` |
| `0x83` | `PONG` | 4 | `time_ms u32` |
| `0x84` | `DEBUG` | 可变 | 数值调试负载，第一阶段不发送字符串 |

如果舵机没有真实位置反馈，`STATUS` 中的关节值应回显最近一次已接受的关节目标值。

### ACK 结果

| 取值 | 名称 |
| --- | --- |
| `0` | `OK` |
| `1` | `BUSY` |
| `2` | `REJECTED` |
| `3` | `BAD_MODE` |
| `4` | `LIMIT_CLAMPED` |
| `5` | `BAD_LENGTH` |

低频命令应返回 `ACK`：`STOP`、`SET_MODE`、`PING`、`GRIPPER`、`ARM_PRESET`、`BUZZER`。高频 `BASE_VEL` 和 `ARM_JOINTS` 不建议每帧都返回 `ACK`；输入非法时发送 `ERROR` 即可。

## 8. 错误码

`STATUS.error_code` 和 `ERROR.error_code` 使用同一张错误码表。

| 编码 | 名称 |
| --- | --- |
| `0x0000` | `OK` |
| `0x0001` | `BAD_HEADER` |
| `0x0002` | `BAD_LENGTH` |
| `0x0003` | `BAD_CRC` |
| `0x0004` | `BAD_TAIL` |
| `0x0005` | `UNKNOWN_TYPE` |
| `0x0006` | `NOT_ROS_AUTO_MODE` |
| `0x0007` | `BASE_TIMEOUT` |
| `0x0008` | `ARM_LIMIT` |
| `0x0009` | `ESTOP_ACTIVE` |
| `0x000A` | `LOW_BATTERY` |
| `0x000B` | `SERIAL_OVERFLOW` |

## 9. 安全规则

- `STOP` 优先级最高，必须在所有模式下都能被接受。
- `STOP` 会进入 `ESTOP`；恢复必须通过显式 `SET_MODE` 命令完成。
- 除 `STOP` 外，自动运动命令只在 `ROS_AUTO` 模式下接受。
- 心跳超时是唯一的自动断联保护触发源；`PING` 只用于启动检查和人工延迟测试。
- 上位机应以 `10` 到 `20 Hz` 发送 `HEARTBEAT`。
- 如果 `300` 到 `500 ms` 内没有收到有效心跳，STM32 必须停止底盘并进入安全保护。
- 如果断联发生在机械臂运动期间，STM32 应安全减速 / 保持；如果硬件允许，可以在更长超时后移动到 `SAFE_UP`。
- STM32 必须限制底盘速度、底盘加速度、关节角度、关节速度和夹爪范围。
- 蜂鸣器播放必须是非阻塞的。

## 10. 接收状态机

推荐解析状态：

```text
WAIT_AA
READ_LEN
READ_TYPE
READ_PAYLOAD
READ_CRC
READ_BB
VERIFY_AND_DISPATCH
```

解析规则：

- 忽略所有字节，直到找到 `0xAA`。
- 如果 `LEN > MAX_PAYLOAD_LEN`，拒绝该帧。
- 读取 `LEN` 后，继续读取 1 个 `TYPE`、`LEN` 个负载字节、1 个 `CRC8` 和 1 个帧尾字节。
- 如果帧尾不是 `0xBB`，拒绝该帧。
- 如果 CRC-8/MAXIM 不匹配，拒绝该帧。
- CRC 或帧尾失败时，滑动 1 个字节并重新搜索 `0xAA`，用于重同步。
- RX 缓冲区溢出时，丢弃数据直到下一个可能的 `0xAA`，上报 `SERIAL_OVERFLOW`，并触发安全停止保护。

## 11. 示例帧

下面的 CRC 值均按 `LEN + TYPE + PAYLOAD` 计算 CRC-8/MAXIM 得到。

| 含义 | 字节 |
| --- | --- |
| `STOP` | `AA 00 00 00 BB` |
| `SET_MODE ROS_AUTO` | `AA 01 01 01 31 BB` |
| `BASE_VEL vx=100 vy=0 wz=300` | `AA 06 10 64 00 00 00 2C 01 A5 BB` |
| `PING time_ms=0x12345678` | `AA 04 02 78 56 34 12 76 BB` |

## 12. 迁移说明

`docs/serial_link_protocol.md` 中记录的旧版 STM32 遥测帧在迁移期间保留兼容。新的 ROS 和 STM32 代码应使用 v2 帧处理新命令和新反馈。

## 13. 第一阶段验收清单

通信底座必须按如下顺序验收，避免任务层问题和串口问题混在一起：

1. `PING/PONG`：确认上位机和 STM32 能稳定收发二进制帧。
2. `STOP`：确认任意模式下都能立即停车。
3. `SET_MODE`：确认只有 `ROS_AUTO` 下执行自动运动命令。
4. `HEARTBEAT`：确认上位机定频发送，断联后 STM32 自动停车。
5. `BASE_VEL`：确认 `/cmd_vel` 能控制四轮低速运动，并且方向映射正确。
6. `BUZZER`：确认蜂鸣器非阻塞播放。
7. `ARM_JOINTS`：确认单关节低速、安全限幅运动。
8. `GRIPPER`：确认夹爪开合范围和方向正确。
9. `STATUS`：确认 STM32 能反馈模式、底盘状态、机械臂状态、错误码、电压和关节镜像值。
10. `ACK/ERROR`：确认非法命令、错误模式、越界和校验错误都能被上位机看到。

当前项目位置：已经完成第 1 到第 5 步的底盘链路验证，下一步应补齐 `BUZZER`、`ARM_JOINTS`、`GRIPPER`、`STATUS`、`ACK/ERROR` 的可重复测试。
