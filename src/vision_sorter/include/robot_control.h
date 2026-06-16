#ifndef VISION_SORTER_ROBOT_CONTROL_H
#define VISION_SORTER_ROBOT_CONTROL_H

#include <vector>

namespace vision_sorter {

// RobotControl is the task-layer command boundary.
// The current implementation is RobotSerial. A ROS1 implementation should
// publish the same commands to tao_serial topics instead of opening UART here.
class RobotControl {
public:
    virtual ~RobotControl() = default;

    virtual bool openPort() = 0;
    virtual void closePort() = 0;
    virtual void stop() = 0;
    virtual void setVelocity(double x, double y, double w) = 0;
    virtual void sendArm(const std::vector<double>& joints) = 0;
    virtual void sendHand(double hand) = 0;
    virtual void sendIk(const std::vector<double>& joints) = 0;
    virtual void beep(int times, int on_time_ms, int off_time_ms) = 0;
    virtual void resetRobot() = 0;
};

}  // namespace vision_sorter

#endif  // VISION_SORTER_ROBOT_CONTROL_H
