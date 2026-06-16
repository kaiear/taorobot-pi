# vision_sorter

从视觉同学交接的 `cpp/` 目录整理出来的 ROS1/catkin 视觉包。

## 正式推荐使用的节点

- `line_node`：巡线识别，发布 `/vision/line/*` 和 `/vision/intersection/detected`。
- `object_node`：红/绿/蓝物块识别，发布 `/vision/object/*`。
- `face_node`：人脸检测，发布 `/vision/face/detected`。

## 备用节点

- `mission_node`：保留原视觉同学的整体任务流程作为备用/参考。它会直接发布 `/cmd_vel`、`/tao_serial/tx`、夹爪和机械臂控制 topic，不建议作为正式主控入口。正式任务主控后续应由 `tao_tasks` 或统一任务管理节点负责。

## 未迁入的污染项

原 `cpp/` 中的 `.git`、`.vscode`、`docs`、`scripts`、重复的 `src/tao_*`、`src/yeahbot_*` 旧包副本，以及带 Git 冲突标记的旧 README 没有迁入。

更多说明见：`docs/vision_sorter_integration.md`。
