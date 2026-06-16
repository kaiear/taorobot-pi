#include "robot_serial.h"

#include "common.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

namespace vision_sorter {

namespace {

constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;

constexpr uint8_t kIdVelocity = 0x50;
constexpr uint8_t kIdHand = 0x60;
constexpr uint8_t kIdArm = 0x70;
constexpr uint8_t kIdIk = 0x90;
constexpr uint8_t kIdBeep = 0x66;
constexpr uint8_t kIdReset = 0x6F;

constexpr size_t kVelocityFrameLength = 11;
constexpr size_t kHandFrameLength = 7;
constexpr size_t kArmFrameLength = 15;
constexpr size_t kIkFrameLength = 19;
constexpr size_t kResetFrameLength = 6;

constexpr int kVelocityScale = 1000;
constexpr int kAngularScale = 1000;
constexpr int kMaxVelocityX = 1500;
constexpr int kMaxVelocityY = 1200;
constexpr int kMaxVelocityW = 6280;
constexpr uint8_t kResetCommand = 100;

// 将波特率整数转换为 termios 的 speed_t 值。
speed_t baudToTermios(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            std::cerr << "unsupported baud " << baud << ", fallback to 115200\n";
            return B115200;
    }
}

}  // namespace

// 构造函数：设置串口路径、波特率与是否为 dry-run 模式。
RobotSerial::RobotSerial(std::string port, int baud, bool dry_run)
    : port_(std::move(port)), baud_(baud), dry_run_(dry_run) {}

// 打开并配置串口设备，返回是否成功。
bool RobotSerial::openPort() {
    if (dry_run_) {
        std::cerr << "[serial] dry-run enabled; frames will not be sent\n";
        return true;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "[serial] cannot open " << port_ << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "[serial] tcgetattr failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    cfsetospeed(&tty, baudToTermios(baud_));
    cfsetispeed(&tty, baudToTermios(baud_));
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 2;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[serial] tcsetattr failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    std::cerr << "[serial] opened " << port_ << " @" << baud_ << std::endl;
    return true;
}

// 关闭串口并停止小车。
void RobotSerial::closePort() {
    stop();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// 停止小车（发送零速度）。
void RobotSerial::stop() {
    setVelocity(0.0, 0.0, 0.0);
}

// 发送速度命令到下位机（m/s, m/s, rad/s）。
void RobotSerial::setVelocity(double x, double y, double w) {
    std::vector<uint8_t> frame = makeFrame(kIdVelocity, kVelocityFrameLength);
    appendInt16(frame, clampValue<int>(static_cast<int>(std::lround(x * kVelocityScale)),
                                       -kMaxVelocityX, kMaxVelocityX));
    appendInt16(frame, clampValue<int>(static_cast<int>(std::lround(y * kVelocityScale)),
                                       -kMaxVelocityY, kMaxVelocityY));
    appendInt16(frame, clampValue<int>(static_cast<int>(std::lround(w * kVelocityScale)),
                                       -kMaxVelocityW, kMaxVelocityW));
    finishFrame(frame);
    writeFrame(frame, "vel");
}

// 发送机械臂 1~5 轴角度命令（rad）。
void RobotSerial::sendArm(const std::vector<double>& joints) {
    if (joints.size() < 5) {
        std::cerr << "[serial] arm command requires at least 5 joint values\n";
        return;
    }

    std::vector<uint8_t> frame = makeFrame(kIdArm, kArmFrameLength);
    for (int i = 0; i < 5; ++i) {
        appendAngleRad(frame, joints[i]);
    }
    finishFrame(frame);
    writeFrame(frame, "arm");
}

// 只发送夹爪角度命令（rad），对应 taorobot 下位机 ID_ROS2STM_HAND。
void RobotSerial::sendHand(double hand) {
    std::vector<uint8_t> frame = makeFrame(kIdHand, kHandFrameLength);
    appendAngleRad(frame, hand);
    finishFrame(frame);
    writeFrame(frame, "hand");
}

// 发送机械臂 IK/全 6 轴角度命令（6 个 rad + 运动时间 ms）。
void RobotSerial::sendIk(const std::vector<double>& joints) {
    if (joints.size() != 7) {
        std::cerr << "[serial] IK command requires 7 values\n";
        return;
    }

    std::vector<uint8_t> frame = makeFrame(kIdIk, kIkFrameLength);
    for (int i = 0; i < 6; ++i) {
        appendAngleRad(frame, joints[i]);
    }
    appendUInt16(frame, static_cast<uint16_t>(clampValue<int>(static_cast<int>(joints[6]), 0, 65535)));
    finishFrame(frame);
    writeFrame(frame, "ik");
}

void RobotSerial::beep(int times, int on_time_ms, int off_time_ms) {
    std::vector<uint8_t> frame = makeFrame(kIdBeep, 11);
    appendInt16(frame, times);
    appendInt16(frame, on_time_ms);
    appendInt16(frame, off_time_ms);
    finishFrame(frame);
    writeFrame(frame, "beep");
}

void RobotSerial::resetRobot() {
    std::vector<uint8_t> frame = makeFrame(kIdReset, kResetFrameLength);
    frame.push_back(kResetCommand);
    finishFrame(frame);
    writeFrame(frame, "reset");
}

// 创建 taorobot USART2 接收帧：AA 55 LEN ID PAYLOAD... SUM。
std::vector<uint8_t> RobotSerial::makeFrame(uint8_t id, size_t frame_length) {
    std::vector<uint8_t> frame;
    frame.reserve(frame_length);
    frame.push_back(kHeader0);
    frame.push_back(kHeader1);
    frame.push_back(static_cast<uint8_t>(frame_length));
    frame.push_back(id);
    return frame;
}

// 下位机把机械臂角度按 rad * 1000 解析，再换算成 PWM。
void RobotSerial::appendAngleRad(std::vector<uint8_t>& frame, double value) {
    appendInt16(frame, static_cast<int>(std::lround(value * kAngularScale)));
}

// 以大端序把有符号 16 位整数写入 frame。
void RobotSerial::appendInt16(std::vector<uint8_t>& frame, int value) {
    int scaled = clampValue<int>(value, -32768, 32767);
    uint16_t u = static_cast<uint16_t>(static_cast<int16_t>(scaled));
    frame.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(u & 0xFF));
}

// 以大端序把无符号 16 位整数写入 frame。
void RobotSerial::appendUInt16(std::vector<uint8_t>& frame, uint16_t value) {
    frame.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(value & 0xFF));
}

// 追加简单和校验：最后 1 字节等于前面所有字节求和的低 8 位。
void RobotSerial::finishFrame(std::vector<uint8_t>& frame) {
    uint32_t sum = 0;
    for (uint8_t byte : frame) {
        sum += byte;
    }
    frame.push_back(static_cast<uint8_t>(sum & 0xFF));
}

// 把封装好的帧写入串口，dry-run 模式下打印而不发送。
void RobotSerial::writeFrame(const std::vector<uint8_t>& frame, const char* label) {
    if (dry_run_) {
        static int skip = 0;
        if (++skip % 20 == 0 || std::strcmp(label, "ik") == 0) {
            std::cerr << "[dry-run] " << label << " frame";
            for (uint8_t b : frame) {
                std::cerr << " " << std::hex << static_cast<int>(b) << std::dec;
            }
            std::cerr << std::endl;
        }
        return;
    }

    if (fd_ < 0) {
        return;
    }

    ssize_t n = ::write(fd_, frame.data(), frame.size());
    if (n != static_cast<ssize_t>(frame.size())) {
        std::cerr << "[serial] short write for " << label << std::endl;
    }
}

}  // namespace vision_sorter
