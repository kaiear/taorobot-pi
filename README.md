# tao-ros1-upper

自动巡检机械臂小车 ROS1 上位机仓库。

本仓库用于保存 Ubuntu 20.04 / ROS1 Noetic 上位机代码。STM32F407 下位机仓库仍保持独立，只作为实时执行层，负责电机、舵机、PWM、安全保护和串口协议解析。

## 工作边界

```text
Windows 本地：写代码、使用 Cline、Git 管理、SFTP 同步
树莓派/Ubuntu：运行 ROS1、catkin_make、连接 STM32 串口、硬件测试
STM32F407：执行串口命令、输出电机/舵机 PWM、保留遥控调试入口
```

## 推荐路径

```text
Windows 本地仓库：D:\cpprobot\pi
树莓派工作空间：/home/ubuntu/tao_robot_ws
树莓派源码目录：/home/ubuntu/tao_robot_ws/src
```

## 目录结构

```text
tao-ros1-upper/
├── README.md
├── docs/
├── scripts/
├── src/
└── sftp_config.example.json
```

## 第一阶段目标

1. 打通 Windows 本地 Cline + SFTP + 树莓派 Remote SSH 工作流。
2. 在树莓派创建 `/home/ubuntu/tao_robot_ws`。
3. 创建最小 ROS1 串口桥接包。
4. 先发送 stop，再低速测试 `/cmd_vel`。
5. 再测试机械臂 Home，不直接运行视觉、导航或 MoveIt。
