# 巡线取物分拣 Demo 说明

本 Demo 面向当前地图：10 个路口、3 个不同取物点、分拣区红/绿/蓝三个色圈。

## 视觉输出

`vision_sorter/object_node` 保留原有物块 topic，并补充发布原视觉结果中已经计算出的面积和中心点：

```text
/vision/object/detected
/vision/object/color
/vision/object/offset_x
/vision/object/offset_y
/vision/object/area
/vision/object/center_x
/vision/object/center_y
```

`offset_x` 约定：目标在画面右侧为正，左侧为负。`offset_y` 下方为正，上方为负。

## 节点职责

- `visual_pick_place_controller.py`
  - `/visual_pick_place/pick`：用 `/vision/object/*` 对准物块，夹取，并记忆颜色。
  - `/visual_pick_place/place`：继续使用综合 `/vision/object/*`，只有当前识别颜色等于记忆颜色时才对准并放置。
- `line_pick_sort_mission.py`
  - 按 `route_plan.yaml` 进行 10 个路口计数、转向、取物、放置、返回起点。

## 启动

```bash
roslaunch tao_tasks line_pick_sort_demo.launch auto_start:=false
```

手动开始任务：

```bash
rosservice call /line_pick_sort_mission/start
```

停止任务：

```bash
rosservice call /line_pick_sort_mission/stop
```

## 现场优先调参

配置文件：

```text
src/tao_tasks/config/visual_servo.yaml
src/tao_tasks/config/route_plan.yaml
```

如果视觉对准方向反了，优先改：

```yaml
visual_pick_place:
  servo:
    angular_sign: -1.0
```

如果路口转向角度不够或过头，优先改：

```yaml
line_pick_sort_mission:
  turns:
    left_sec: 1.15
    right_sec: 1.15
    turn_back_sec: 2.35
```

如果机械臂夹爪开合不合适，优先改：

```yaml
visual_pick_place:
  gripper:
    open: 80
    close: 25
```
