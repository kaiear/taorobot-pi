#ifndef VISION_SORTER_ROS1_ROBOT_CONTROL_H
#define VISION_SORTER_ROS1_ROBOT_CONTROL_H

#include "robot_control.h"

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

namespace vision_sorter {

class Ros1RobotControl : public RobotControl {
public:
    explicit Ros1RobotControl(ros::NodeHandle& nh);

    bool openPort() override;
    void closePort() override;
    void stop() override;
    void setVelocity(double x, double y, double w) override;
    void sendArm(const std::vector<double>& joints) override;
    void sendHand(double hand) override;
    void sendIk(const std::vector<double>& joints) override;
    void beep(int times, int on_time_ms, int off_time_ms) override;
    void resetRobot() override;

private:
    std::vector<int> jointsToProtocol(const std::vector<double>& joints) const;
    void publishArmCommand(const std::vector<int>& protocol_values, int duration_ms);

    ros::Publisher cmd_vel_pub_;
    ros::Publisher buzzer_pub_;
    ros::Publisher gripper_pub_;
    ros::Publisher arm_units_pub_;
    ros::Publisher serial_tx_pub_;

    std::vector<double> protocol_offsets_{0.0, 0.0, 1.602, 1.523, 0.0, 0.0};
    std::vector<int> protocol_signs_{1, 1, 1, 1, 1, 1};
    double protocol_scale_ = 1000.0;
    int protocol_min_ = -30000;
    int protocol_max_ = 30000;
    bool publish_shadow_topic_ = true;
    bool use_tx_command_ = true;
    uint8_t arm_seq_ = 0;
};

}  // namespace vision_sorter

#endif  // VISION_SORTER_ROS1_ROBOT_CONTROL_H
