#ifndef VISION_SORTER_ARM_KINEMATICS_H
#define VISION_SORTER_ARM_KINEMATICS_H

#include <optional>
#include <vector>

namespace vision_sorter {

// ArmKinematics 这个类负责机械臂逆运动学。
// “逆运动学”的意思是：我们给出夹爪想去的位置 x/y/z，
// 类内部计算每个关节应该转多少角度。
class ArmKinematics {
public:
    std::optional<std::vector<double>> move(double x, double y, double z, int move_time_ms);
    std::vector<double> claw(double spin_claw, double hand, int move_time_ms);

private:
    int analysis(double x, double y, double z, double alpha);

    std::vector<double> joints_{0.0, -0.93, 2.07, 1.3, 0.0, 0.8, 1500.0};
};

}  // namespace vision_sorter

#endif  // VISION_SORTER_ARM_KINEMATICS_H
