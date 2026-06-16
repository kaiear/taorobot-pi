#include "ros1_robot_control.h"

#include "common.h"
#include "ros_topics.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace vision_sorter {

Ros1RobotControl::Ros1RobotControl(ros::NodeHandle& nh) {
    ros::NodeHandle pnh("~");
    pnh.param("protocol_offsets", protocol_offsets_, protocol_offsets_);
    pnh.param("protocol_signs", protocol_signs_, protocol_signs_);
    pnh.param("protocol_scale", protocol_scale_, protocol_scale_);
    pnh.param("protocol_min", protocol_min_, protocol_min_);
    pnh.param("protocol_max", protocol_max_, protocol_max_);
    pnh.param("publish_shadow_topic", publish_shadow_topic_, publish_shadow_topic_);
    pnh.param("use_tx_command", use_tx_command_, use_tx_command_);

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(ros_topics::kCmdVel, 10);
    buzzer_pub_ = nh.advertise<std_msgs::UInt8>(ros_topics::kBuzzerPlay, 10);
    gripper_pub_ = nh.advertise<std_msgs::UInt8>(ros_topics::kGripperCommand, 10);
    arm_units_pub_ = nh.advertise<std_msgs::Int16MultiArray>(ros_topics::kArmJointsProtocolUnits, 10);
    serial_tx_pub_ = nh.advertise<std_msgs::String>(ros_topics::kSerialTx, 10);
}

bool Ros1RobotControl::openPort() {
    return true;
}

void Ros1RobotControl::closePort() {
    stop();
}

void Ros1RobotControl::stop() {
    setVelocity(0.0, 0.0, 0.0);
    std_msgs::String msg;
    msg.data = "STOP";
    serial_tx_pub_.publish(msg);
}

void Ros1RobotControl::setVelocity(double x, double y, double w) {
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = w;
    cmd_vel_pub_.publish(msg);
}

void Ros1RobotControl::sendArm(const std::vector<double>& joints) {
    sendIk(joints);
}

void Ros1RobotControl::sendHand(double hand) {
    int percent = clampValue<int>(static_cast<int>(std::lround(hand * 100.0)), 0, 100);
    std_msgs::UInt8 msg;
    msg.data = static_cast<uint8_t>(percent);
    gripper_pub_.publish(msg);
}

void Ros1RobotControl::sendIk(const std::vector<double>& joints) {
    if (joints.size() < 6) {
        ROS_WARN("IK command requires at least 6 joint values, got %zu", joints.size());
        return;
    }
    int duration_ms = joints.size() >= 7 ? clampValue<int>(static_cast<int>(joints[6]), 50, 5000) : 500;
    publishArmCommand(jointsToProtocol(joints), duration_ms);
}

void Ros1RobotControl::beep(int times, int, int) {
    std_msgs::UInt8 msg;
    msg.data = static_cast<uint8_t>(std::max(1, times));
    buzzer_pub_.publish(msg);
}

void Ros1RobotControl::resetRobot() {
    std_msgs::String msg;
    msg.data = "SET_MODE ROS_AUTO";
    serial_tx_pub_.publish(msg);
}

std::vector<int> Ros1RobotControl::jointsToProtocol(const std::vector<double>& joints) const {
    std::vector<int> values(6, 0);
    for (size_t i = 0; i < values.size() && i < joints.size(); ++i) {
        double offset = i < protocol_offsets_.size() ? protocol_offsets_[i] : 0.0;
        int sign = i < protocol_signs_.size() ? protocol_signs_[i] : 1;
        int value = static_cast<int>(std::lround((joints[i] - offset) * protocol_scale_ * sign));
        values[i] = clampValue<int>(value, protocol_min_, protocol_max_);
    }
    return values;
}

void Ros1RobotControl::publishArmCommand(const std::vector<int>& protocol_values, int duration_ms) {
    if (publish_shadow_topic_) {
        std_msgs::Int16MultiArray msg;
        for (int value : protocol_values) {
            msg.data.push_back(static_cast<int16_t>(clampValue<int>(value, -32768, 32767)));
        }
        arm_units_pub_.publish(msg);
    }

    if (!use_tx_command_) {
        return;
    }

    arm_seq_ = static_cast<uint8_t>((arm_seq_ + 1) & 0xFF);
    std::ostringstream command;
    command << "ARM_JOINTS " << static_cast<int>(arm_seq_);
    for (int value : protocol_values) {
        command << " " << value;
    }
    command << " " << duration_ms;

    std_msgs::String msg;
    msg.data = command.str();
    serial_tx_pub_.publish(msg);
}

}  // namespace vision_sorter
