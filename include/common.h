#ifndef VISION_SORTER_COMMON_H
#define VISION_SORTER_COMMON_H

#include <algorithm>

namespace vision_sorter {

// clampValue 是一个小工具函数：把 value 限制在 [lo, hi] 范围内。
// 例如串口发送 int16 数据时，数值不能超过 -32768 到 32767，
// 用这个函数可以避免数据溢出。
template <typename T>
T clampValue(T value, T lo, T hi) {
    return std::max(lo, std::min(value, hi));
}

constexpr double kPi = 3.1415926;

}  // namespace vision_sorter

#endif  // VISION_SORTER_COMMON_H
