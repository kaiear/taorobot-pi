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
