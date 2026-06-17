#ifndef VISION_SORTER_ROS_TOPICS_H
#define VISION_SORTER_ROS_TOPICS_H

namespace vision_sorter::ros_topics {

constexpr const char* kCmdVel = "/cmd_vel";
constexpr const char* kBuzzerPlay = "/buzzer/play";
constexpr const char* kGripperCommand = "/gripper/command";
constexpr const char* kArmJointsProtocolUnits = "/tao_arm/joints_protocol_units";
constexpr const char* kSerialTx = "/tao_serial/tx";

constexpr const char* kLineError = "/vision/line/error";
constexpr const char* kLineVisible = "/vision/line/visible";
constexpr const char* kIntersectionDetected = "/vision/intersection/detected";
constexpr const char* kObjectDetected = "/vision/object/detected";
constexpr const char* kObjectColor = "/vision/object/color";
constexpr const char* kObjectOffsetX = "/vision/object/offset_x";
constexpr const char* kObjectOffsetY = "/vision/object/offset_y";
constexpr const char* kObjectArea = "/vision/object/area";
constexpr const char* kObjectCenterX = "/vision/object/center_x";
constexpr const char* kObjectCenterY = "/vision/object/center_y";
constexpr const char* kFaceDetected = "/vision/face/detected";
constexpr const char* kTagId = "/vision/tag/id";

}  // namespace vision_sorter::ros_topics

#endif  // VISION_SORTER_ROS_TOPICS_H
