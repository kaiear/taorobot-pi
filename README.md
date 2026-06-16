<<<<<<< HEAD
# vision_sorter

这个目录把原来的巡线、物块识别/夹取、人脸识别和总任务流程拆成了独立入口，便于逐个调试。当前 PC 没有 ROS1 时，可以先用普通 C++ 可执行程序测试摄像头、OpenCV 识别和原任务状态机；上车到 ROS1 环境后，再用 catkin 构建 ROS 节点。

## 节点划分

普通本地可执行程序：

- `line_node`：只做黑线/路口检测，输出未来 ROS topic 对应字段。
- `object_node`：只做红/绿/蓝物块检测，输出颜色和中心偏差。
- `face_node`：只做人脸检测。
- `mission_node`：总控流程，保留原逻辑：先人脸识别，30 秒超时或识别到人脸并挥臂后进入巡线，再按交叉口计数执行抓取/放置策略。
- `vision_sorter`：兼容原入口，行为基本等同原程序。
- `serial_link_test`：Linux 实车串口冒烟测试工具。

ROS1/catkin 节点：

- `line_node` 发布 `/vision/line/*`。
- `object_node` 发布 `/vision/object/*`。
- `face_node` 发布 `/vision/face/detected`。
- `mission_node` 发布控制 topic，调用 `taorobot-pi-main` 里的 `tao_serial_node.py`。

## 与 taorobot-pi-main 对齐的接口

控制硬件的 topic 由 `taorobot-pi-main/src/tao_serial/scripts/tao_serial_node.py` 接收：

```text
/cmd_vel                          geometry_msgs/Twist       底盘速度
/buzzer/play                      std_msgs/UInt8            蜂鸣器
/gripper/command                  std_msgs/UInt8            夹爪百分比
/tao_arm/joints_protocol_units    std_msgs/Int16MultiArray  机械臂协议单位
/tao_serial/tx                    std_msgs/String           STOP/SET_MODE/ARM_JOINTS 等文本命令
```

视觉节点预留/发布的 topic：

```text
/vision/line/error              std_msgs/Float32   黑线中心偏差，约 -1..1
/vision/line/visible            std_msgs/Bool      是否看到黑线
/vision/intersection/detected   std_msgs/Bool      是否检测到路口
/vision/object/detected         std_msgs/Bool      是否看到物块
/vision/object/color            std_msgs/String    red/green/blue/unknown
/vision/object/offset_x         std_msgs/Float32   物块横向偏差，约 -1..1
/vision/object/offset_y         std_msgs/Float32   物块纵向偏差，约 -1..1
/vision/face/detected           std_msgs/Bool      是否检测到人脸
/vision/tag/id                  std_msgs/Int32     预留 AprilTag ID，未识别为 -1
```

## 普通 Linux 构建

依赖：C++17、OpenCV 4、`pkg-config`。AprilTag 可选。

```bash
cd cpp
chmod +x install_deps.sh build.sh
./install_deps.sh
./build.sh
```

构建产物：

```text
./line_node
./object_node
./face_node
./mission_node
./vision_sorter
./serial_link_test
```

如果系统安装了 `apriltag` 开发库，还会生成：

```text
./vision_sorter_apriltag
./mission_node_apriltag
```

## 本地可测流程

先只测摄像头和识别，不接串口、不动硬件：

```bash
cd cpp
./line_node --camera 0 --show
./object_node --camera 0 --show
./face_node --camera 0 --show
```

测试原完整流程但不发送真实串口帧：

```bash
./mission_node --camera 0 --dry-run --show
```

兼容原入口：

```bash
./vision_sorter --camera 0 --dry-run --show
./vision_sorter --face-test --camera 0 --show
```

窗口里按 `q` 或 `Esc` 退出。

## ROS1 构建与运行

把 `cpp` 目录作为 ROS 包放入 catkin 工作空间，例如：

```bash
mkdir -p ~/tao_robot_ws/src
cp -r cpp ~/tao_robot_ws/src/vision_sorter
cd ~/tao_robot_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

先启动串口桥：

```bash
roslaunch tao_bringup serial_test.launch open_serial:=true port:=/dev/ttyS0 log_tx:=true log_rx:=true
```

分别调试视觉节点：

```bash
rosrun vision_sorter line_node --camera 0 --show
rosrun vision_sorter object_node --camera 0 --show
rosrun vision_sorter face_node --camera 0 --show
```

也可以用 launch 文件：

```bash
roslaunch vision_sorter line_node.launch camera:=0 show:=true
roslaunch vision_sorter object_node.launch camera:=0 show:=true
roslaunch vision_sorter face_node.launch camera:=0 show:=true
```

检查 topic：

```bash
rostopic echo /vision/line/error
rostopic echo /vision/intersection/detected
rostopic echo /vision/object/color
rostopic echo /vision/face/detected
```

运行总控节点：

```bash
rosrun vision_sorter mission_node --camera 0 --show
roslaunch vision_sorter mission_node.launch camera:=0 show:=true
```

`mission_node` 会向 `/cmd_vel` 和 `/tao_serial/tx` 发控制命令；串口由 `tao_serial_node.py` 统一处理。

## 实车安全测试顺序

1. 不上车，先跑 `line_node/object_node/face_node --show`，确认识别稳定。
2. 启动 `serial_test.launch`，确认 `rostopic list` 能看到 `/cmd_vel`、`/buzzer/play`、`/gripper/command`、`/tao_serial/tx`。
3. 单独测试蜂鸣器、夹爪、低速底盘：

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

4. 确认急停/断电手段可用后，再运行 `mission_node`。
5. 实车观察 `/tao_serial/error_json` 和 `/tao_serial/status_json`，确认没有持续错误。

## 当前 PC 的限制

本次本地环境是 Windows/PowerShell，没有 ROS1、bash、pkg-config、cmake，也没有 Linux `termios.h`。因此本机只能做非 ROS 的代码结构检查和部分 C++ 编译检查；完整 OpenCV/Linux 串口/ROS 通信需要在 Ubuntu/树莓派 ROS1 环境验证。
=======
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
>>>>>>> fededa552a8845b87603200500ef0cdd9ec21204
