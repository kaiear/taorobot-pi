# Tao Robot 第一批任务节点实车测试计划

## 目标

第一批只验证底层任务能力，不直接跑完整巡线抓取：

1. 串口链路和 `/cmd_vel` 安全可控。
2. 巡线视觉能稳定发布偏差。
3. 小车能低速跟黑线并在丢线时停车。
4. 人脸识别能触发一次欢迎动作，20 秒内不重复触发。

## 0. 编译

```bash
cd ~/tao_robot_ws
catkin_make
source devel/setup.bash
```

## 1. 串口节点

```bash
roslaunch tao_bringup serial_test.launch open_serial:=true port:=/dev/ttyS0
```

建议先让车轮离地，确认 STM32 进入 ROS 自动模式后不会自行动作。

## 2. 只测巡线视觉，不动车

```bash
roslaunch vision_sorter line_node.launch camera:=0 show:=true
```

另开终端：

```bash
rostopic echo /vision/line/visible
rostopic echo /vision/line/error
rostopic echo /vision/intersection/detected
```

预期：

- 看到黑线时 `/vision/line/visible` 为 `true`。
- 黑线左右移动时 `/vision/line/error` 正负变化。
- 到十字/粗线区域时 `/vision/intersection/detected` 会变化。

如果这一步不稳定，先调 `vision_sorter/config/camera.yaml` 的黑线 HSV 和 ROI。

## 3. 巡线控制 debug，不发 `/cmd_vel`

```bash
roslaunch tao_tasks line_follow_debug.launch
```

预期：节点每秒打印一次当前 `error`、`intersection` 和计算出来的速度，但不会让车动。

## 4. 架空车轮低速测试

```bash
roslaunch tao_tasks line_follow_test.launch publish_cmd_vel:=true forward_speed:=0.05
```

预期：

- 车轮低速转动。
- 线在画面右边时，控制量会向右修正；线在左边时向左修正。
- 遮住黑线或移开地图后，`lost_timeout` 后停车。

如果方向反了，修改：

```yaml
line_follow:
  angular_sign: 1.0   # 或 -1.0
```

## 5. 地面短距离巡线

仍从低速开始：

```bash
roslaunch tao_tasks line_follow_test.launch publish_cmd_vel:=true forward_speed:=0.05
```

调参建议：

- 左右震荡：降低 `kp_angular` 或 `forward_speed`。
- 转弯跟不上：提高 `kp_angular`，但不要超过安全速度。
- 丢线不停车：检查 `/vision/line/visible` 和 `lost_timeout`。

## 6. 人脸欢迎测试

启动人脸视觉：

```bash
roslaunch vision_sorter face_node.launch camera:=0 show:=true
```

启动欢迎动作：

```bash
roslaunch tao_tasks face_greeting_test.launch
```

默认效果：

- 第一次看到人脸：发布 `/buzzer/play=1`，底盘左右轻微摇动后停止。
- 20 秒内不重复触发。
- 机械臂挥手默认关闭。

确认机械臂姿态安全后再打开：

```bash
roslaunch tao_tasks face_greeting_test.launch enable_arm_wave:=true
```

## 下一阶段

第一批验证通过后，再进入：

1. 路口计数与路线 YAML。
2. 到指定路口停车。
3. 静态物块识别与机械臂抓取。
4. 巡线到点抓取和放置闭环。
