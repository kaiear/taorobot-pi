# vision_sorter 视觉包集成说明

`src/vision_sorter` 是从视觉同学交接的 `cpp/` 目录中整理出的 ROS1/catkin 包。

## 迁入原则

已迁入：

- `CMakeLists.txt`
- `package.xml`
- `config/camera.yaml`
- `include/` 中当前节点需要的头文件
- `launch/line_node.launch`
- `launch/object_node.launch`
- `launch/face_node.launch`
- `launch/mission_node.launch`
- `src/ros1_line_node.cpp`
- `src/ros1_object_node.cpp`
- `src/ros1_face_node.cpp`
- `src/ros1_mission_node.cpp`
- 视觉识别、机械臂动作参考、ROS1 控制适配相关 `.cpp`

未迁入或已清理：

- 原 `cpp/.git`
- 原 `cpp/.vscode`
- 原 `cpp/docs`
- 原 `cpp/scripts`
- 原 `cpp/src/tao_bringup`
- 原 `cpp/src/tao_serial`
- 原 `cpp/src/yeahbot_c1`
- 原 `cpp/src/yeahbot_c1_moveit_config`
- 原 `cpp/README.md`，因为里面有 Git 冲突标记
- 未接入当前 CMake 的旧串口/AprilTag 残留文件

## 正式推荐节点

### line_node

巡线识别节点。

发布：

```text
/vision/line/visible             std_msgs/Bool
/vision/line/error               std_msgs/Float32
/vision/intersection/detected    std_msgs/Bool
```

运行：

```bash
roslaunch vision_sorter line_node.launch camera:=0 show:=false
```

### object_node

红/绿/蓝物块识别节点。

发布：

```text
/vision/object/detected          std_msgs/Bool
/vision/object/color             std_msgs/String
/vision/object/offset_x          std_msgs/Float32
/vision/object/offset_y          std_msgs/Float32
```

运行：

```bash
roslaunch vision_sorter object_node.launch camera:=0 show:=false
```

### face_node

人脸检测节点。

发布：

```text
/vision/face/detected            std_msgs/Bool
```

运行：

```bash
roslaunch vision_sorter face_node.launch camera:=0 show:=false
```

## 备用节点：mission_node

`mission_node` 保留为备用/参考节点。它会直接发布机器人控制 topic：

```text
/cmd_vel
/buzzer/play
/gripper/command
/tao_arm/joints_protocol_units
/tao_serial/tx
```

注意：它包含原视觉同学的整体任务流程，会直接控制底盘和机械臂。正式比赛主控不建议直接使用它，后续应由统一任务管理节点订阅 `/vision/*` 并调用 `tao_serial`、MoveIt 或任务动作接口。

## 调试顺序建议

1. 先单独启动摄像头识别节点，不接机器人动作。
2. 用 `rostopic echo` 确认 `/vision/*` topic 数据稳定。
3. 再接入任务主控节点。
4. 只有在确认急停和低速控制可靠后，才尝试 `mission_node` 备用流程。
