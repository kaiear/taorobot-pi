#ifndef VISION_SORTER_ROBOT_SERIAL_H
#define VISION_SORTER_ROBOT_SERIAL_H

#include "robot_control.h"

#include <cstdint>
#include <string>
#include <vector>

namespace vision_sorter {

// RobotSerial 这个类只负责“和下位机串口通信”。
// main 和控制器不需要知道 termios 怎么配置，也不需要知道每一帧字节怎么拼。
// 它们只调用 setVelocity 或 sendIk，就能发送底盘速度和机械臂命令。
class RobotSerial : public RobotControl {
public:
    RobotSerial(std::string port, int baud, bool dry_run);

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
    static std::vector<uint8_t> makeFrame(uint8_t id, size_t frame_length);
    static void appendAngleRad(std::vector<uint8_t>& frame, double value);
    static void appendInt16(std::vector<uint8_t>& frame, int value);
    static void appendUInt16(std::vector<uint8_t>& frame, uint16_t value);
    static void finishFrame(std::vector<uint8_t>& frame);
    void writeFrame(const std::vector<uint8_t>& frame, const char* label);

    std::string port_;
    int baud_;
    bool dry_run_;
    int fd_ = -1;
};

}  // namespace vision_sorter

#endif  // VISION_SORTER_ROBOT_SERIAL_H
