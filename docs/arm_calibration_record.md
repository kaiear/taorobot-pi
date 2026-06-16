# 机械臂标定记录表

本表用于记录机械臂接入 MoveIt 前的实机标定结果。当前协议单位为 `rad * 1000`，例如 `100` 约等于 `0.1 rad`，约 `5.7°`。

## 0. 厂家模型来源

已复制到当前工作区 `src/`：

- `src/yeahbot_c1`
- `src/yeahbot_c1_moveit_config`

对应上游源码位置：

- `../download/tao资料/10.相关工具/1.源码/上位机源码/1.src文件/src/yeahbot_c1`
- `../download/tao资料/10.相关工具/1.源码/上位机源码/1.src文件/src/yeahbot_c1_moveit_config`

厂家 URDF：`src/yeahbot_c1/urdf/yeahbot_c1.urdf`

### 0.1 主臂关节轴对照

| 真机关节 | 厂家 URDF 关节名 | `axis` | 备注 |
| --- | --- | --- | --- |
| `joint0` | `arm_0_joint` | `0 0 -1` | 底座回转 |
| `joint1` | `arm_1_joint` | `-0.040462 0.99918 0` | 肩关节 |
| `joint2` | `arm_2_joint` | `-0.0404617 0.999181 0` | 肘关节 |
| `joint3` | `arm_3_joint` | `-0.0404617 0.999181 0` | 腕部 |
| `joint4` | `arm_4_joint` | `-7.78422704402624E-05 0 -0.999999996965322` | 末端旋转 |
| `joint5` / 夹爪 | `arm_5_1_joint` ~ `arm_5_6_joint` | 联动结构 | 建议先保持独立协议，不直接映射为多个自由度 |

## 1. 测试前确认

| 项目 | 结果 | 备注 |
| --- | --- | --- |
| F407 是否已重新烧录最新固件 | 是 / 否 | 需要包含 `0` 角度回到 `servo_home_pwm(index)` 的修正 |
| 树莓派工作空间 | `~/tao_robot_ws` |  |
| 串口设备 | `/dev/ttyS0` |  |
| 串口节点是否正常启动 | 是 / 否 | `roslaunch tao_bringup serial_test.launch open_serial:=true port:=/dev/ttyS0` |
| 是否已切到 `ROS_AUTO` | 是 / 否 | `SET_MODE ROS_AUTO` |
| 是否已准备急停命令 | 是 / 否 | `rostopic pub -1 /tao_serial/tx std_msgs/String "data: 'STOP'"` |

## 2. Home 验证

| 项目 | 结果 | 备注 |
| --- | --- | --- |
| `ARM_PRESET 0` 是否到达原 home | 是 / 否 |  |
| `[0, 0, 0, 0, 0, 0]` 是否和 `ARM_PRESET 0` 一致 | 是 / 否 |  |
| 如果不一致，哪个关节偏移 |  | 例如：`joint2`、`joint3` |
| Home 姿态是否安全、不顶结构 | 是 / 否 |  |
| Home 姿态是否适合作为 MoveIt 零位 | 是 |  |

已确认的 Home 偏移记录：

| 关节 | MoveIt home 角度（rad） | 备注 |
| --- | ---: | --- |
| `joint0` | `0` | 与真机协议 0 一致 |
| `joint1` | `0` | 与真机协议 0 一致 |
| `joint2` | `1.602` | 与真机协议 0 对应的实际 home 角度 |
| `joint3` | `1.523` | 与真机协议 0 对应的实际 home 角度 |
| `joint4` | `0` | 与真机协议 0 一致 |
| `joint5` / 夹爪 | `0` | 当前按协议 0 记为 home |

## 3. 单关节方向与角度测量

每次只动一个关节，先测 `+100`，回 `0`，再测 `-100`，再回 `0`。

| 关节 | Home 姿态描述 | `+100` 实际运动方向 | `+100` 实测角度 | `-100` 实际运动方向 | `-100` 实测角度 | 是否符合预期正方向 | 备注 |
| --- | --- | --- | ---: | --- | ---: | --- | --- |
| `joint0` | 朝向正前方 | 顺时针 | 10 |  |  | 是 / 否 / 不确定 |  |
| `joint1` | 竖直 |俯  |10  |  |  | 是 / 否 / 不确定 |  |
| `joint2` |水平  |  俯|10  |  |  | 是 / 否 / 不确定 |  |
| `joint3` | 垂直 | 俯 |10  |  |  | 是 / 否 / 不确定 |  |
| `joint4` | 正常 | 逆时针 | 10 |  |  | 是 / 否 / 不确定 |  |
| `joint5` / 夹爪 | 半张 | 完全打开 |  | 闭合 |  | 是 / 否 / 不确定 | `joint5 -100` 为闭合方向；夹爪百分比映射已反向 |

## 4. 安全范围测量

方向确认后再测安全范围。建议按 `100 -> 200 -> 300 -> 500 -> 800 -> 1000` 逐步增加，每次都先回 `0`。

| 关节 | 正方向安全最大协议值 | 负方向安全最小协议值 | 推荐软件限位 | 是否碰机械限位 | 舵机是否抖动/异响 | 备注 |
| --- | ---: | ---: | --- | --- | --- | --- |
| `joint0` |  |  | 例如：`[-800, 800]` | 是 / 否 | 是 / 否 |  |
| `joint1` |  |  |  | 是 / 否 | 是 / 否 |  |
| `joint2` |  |  |  | 是 / 否 | 是 / 否 |  |
| `joint3` |  |  |  | 是 / 否 | 是 / 否 |  |
| `joint4` |  |  |  | 是 / 否 | 是 / 否 |  |
| `joint5` / 夹爪 |  |  |  | 是 / 否 | 是 / 否 |  |

## 5. 夹爪测量

| 命令 | 实际动作 | 是否正确 | 备注 |
| --- | --- | --- | --- |
| `gripper 0` | 张开 / 闭合 / 无动作 | 是 / 否 | 预期：张开 |
| `gripper 50` | 半开 / 半闭 / 无动作 | 是 / 否 | 预期：半闭 |
| `gripper 100` | 张开 / 闭合 / 无动作 | 是 / 否 | 预期：闭合；F407 映射为 joint5 负方向 |

## 6. 测试命令记录

启动串口节点：

```bash
cd ~/tao_robot_ws
source devel/setup.bash
roslaunch tao_bringup serial_test.launch open_serial:=true port:=/dev/ttyS0
```

切自动模式和回 home：

```bash
cd ~/tao_robot_ws
source devel/setup.bash
rostopic pub -1 /tao_serial/tx std_msgs/String "data: 'SET_MODE ROS_AUTO'"
rostopic pub -1 /tao_serial/tx std_msgs/String "data: 'ARM_PRESET 0'"
rostopic pub -1 /tao_arm/joints_protocol_units std_msgs/Int16MultiArray "data: [0, 0, 0, 0, 0, 0]"
```

`joint0` 示例：

```bash
rostopic pub -1 /tao_arm/joints_protocol_units std_msgs/Int16MultiArray "data: [100, 0, 0, 0, 0, 0]"
rostopic pub -1 /tao_arm/joints_protocol_units std_msgs/Int16MultiArray "data: [0, 0, 0, 0, 0, 0]"
rostopic pub -1 /tao_arm/joints_protocol_units std_msgs/Int16MultiArray "data: [-100, 0, 0, 0, 0, 0]"
rostopic pub -1 /tao_arm/joints_protocol_units std_msgs/Int16MultiArray "data: [0, 0, 0, 0, 0, 0]"
```

急停：

```bash
rostopic pub -1 /tao_serial/tx std_msgs/String "data: 'STOP'"
```

恢复自动模式：

```bash
rostopic pub -1 /tao_serial/tx std_msgs/String "data: 'SET_MODE ROS_AUTO'"
```

## 7. MoveIt 启动验证建议

先在当前工作区确认包可被 catkin 识别，然后优先验证厂家 demo：

```bash
cd ~/tao_robot_ws
catkin_make
source devel/setup.bash
roslaunch yeahbot_c1_moveit_config demo.launch
```

如果只想检查 URDF / RViz 显示，可以先用 fake controller 模式；如果要接真机，再把控制器配置切到 `ros_control` 或你的串口桥接节点。

## 8. 当前映射结论

当前可直接采用的零位映射：

```text
joint0 = protocol0 / 1000 + 0
joint1 = protocol1 / 1000 + 0
joint2 = protocol2 / 1000 + 1.602
joint3 = protocol3 / 1000 + 1.523
joint4 = protocol4 / 1000 + 0
joint5 = protocol5 / 1000 + 0
```

因为你已经确认方向一致，所以当前不需要加负号；后续若某个关节发现方向相反，再单独对该关节加符号修正。
