# ROS1 上位机迁移计划

## 当前状态

- STM32F407 下位机已经能通过遥控器实现小车移动和机械臂抓取。
- 下位机后续应冻结为执行层，不继续加入自动巡检算法。
- ROS1 已安装，上位机系统为 Ubuntu 20.04。

## 职责边界

### STM32 下位机保留

- 电机 PWM 输出
- 舵机 PWM 输出
- 基础麦轮执行或 `vx/vy/wz` 到轮速转换
- 舵机限幅
- stop 急停
- 串口超时停车
- 上电安全
- USART2 协议解析
- 遥控器现场调试入口

### ROS1 上位机负责

- `/cmd_vel` 控制
- 串口桥接
- 键盘/手柄控制
- 摄像头采集
- 巡线、颜色识别、视觉处理
- 自动巡检状态机
- 机械臂动作编排

## 推荐包结构

```text
src/
├── tao_serial/
├── tao_base_bridge/
├── tao_arm_bridge/
├── tao_teleop/
├── tao_camera/
├── tao_vision/
└── tao_inspection/
```

## 迁移顺序

```text
tao_serial
→ tao_base_bridge
→ tao_arm_bridge
→ tao_teleop
→ tao_camera
→ tao_vision
→ tao_inspection
```

## 速度帧参考

```text
AA 55 0B 50 vx_h vx_l vy_h vy_l wz_h wz_l checksum
```

其中：

```text
vx, vy, wz = 浮点速度 * 1000 后转 int16
checksum = 前面所有字节累加低 8 位
```

## 第一阶段成功标准

1. ROS 节点能打开 STM32 串口。
2. ROS 能发送 stop。
3. ROS `/cmd_vel` 能低速控制底盘。
4. ROS 能发送机械臂 Home。
5. 下位机遥控器功能仍可作为现场调试/救援入口。
