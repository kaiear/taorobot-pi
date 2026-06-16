#include "arm_kinematics.h"

#include "common.h"

#include <cmath>

namespace vision_sorter {

std::optional<std::vector<double>> ArmKinematics::move(double x, double y, double z, int move_time_ms) {
// 计算机械臂到目标 (x,y,z) 的关节值并返回（包含移动时间在 joints_[6]）。
// 若目标不可达则返回 std::nullopt。
    if (y < 0) {
        return std::nullopt;
    }

    std::optional<int> best_alpha;
    for (int alpha = 0; alpha >= -135; --alpha) {
        if (analysis(x, y, z, alpha) == 0) {
            best_alpha = alpha;
        }
    }

    if (!best_alpha.has_value()) {
        return std::nullopt;
    }

    analysis(x, y, z, *best_alpha);
    joints_[6] = move_time_ms;
    return joints_;
}

std::vector<double> ArmKinematics::claw(double spin_claw, double hand, int move_time_ms) {
// 设置爪子相关关节值并返回当前 joints 列表。
    joints_[4] = spin_claw;
    joints_[5] = hand;
    joints_[6] = move_time_ms;
    return joints_;
}

int ArmKinematics::analysis(double x, double y, double z, double alpha) {
// 内部分析函数：根据参数计算各轴角度并存入 joints_。
// 返回 0 表示成功，非 0 表示不同类型的错误/不可达情况。
    x *= 10.0;
    y *= 10.0;
    z *= 10.0;
    constexpr double l0 = 2100.0;
    constexpr double l1 = 1250.0;
    constexpr double l2 = 1200.0;
    constexpr double l3 = 1550.0;

    double theta6 = (x == 0.0) ? 0.0 : std::atan(x / y) * 270.0 / kPi;
    y = std::sqrt(x * x + y * y);
    y = y - l3 * std::cos(alpha * kPi / 180.0);
    z = z - l0 - l3 * std::sin(alpha * kPi / 180.0);
    if (z < -l0) return 1;
    if (std::sqrt(y * y + z * z) > (l1 + l2)) return 2;

    double ccc = std::acos(y / std::sqrt(y * y + z * z));
    double bbb = (y * y + z * z + l1 * l1 - l2 * l2) / (2.0 * l1 * std::sqrt(y * y + z * z));
    if (bbb > 1.0 || bbb < -1.0) return 5;
    double zf_flag = z < 0.0 ? -1.0 : 1.0;
    double theta5 = (ccc * zf_flag + std::acos(bbb)) * 180.0 / kPi;
    if (theta5 > 180.0 || theta5 < 0.0) return 6;

    double aaa = -(y * y + z * z - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (aaa > 1.0 || aaa < -1.0) return 3;
    double theta4 = 180.0 - std::acos(aaa) * 180.0 / kPi;
    if (theta4 > 135.0 || theta4 < -135.0) return 4;
    double theta3 = alpha - theta5 + theta4;
    if (theta3 > 90.0 || theta3 < -90.0) return 7;

    joints_[0] = -theta6 * kPi / 180.0;
    joints_[1] = -(theta5 - 90.0) * kPi / 180.0;
    joints_[2] = theta4 * kPi / 180.0;
    joints_[3] = -theta3 * kPi / 180.0;
    return 0;
}

}  // namespace vision_sorter
