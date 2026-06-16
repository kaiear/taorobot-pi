#include "sorter_controller.h"

#include "common.h"

#include <cmath>
#include <string>

#include <opencv2/imgproc.hpp>

namespace vision_sorter {

// 构造函数：绑定串口通信对象与视觉配置引用。
SorterController::SorterController(RobotControl& robot, const VisionConfig& vision)
    : robot_(robot), vision_(vision) {}

// 机器人初始化：复位机械臂到预设位置并打开夹爪。
void SorterController::initRobot() {
    moveX_ = armErrX_;
    moveY_ = 150.0;
    if (auto joints = arm_.move(moveX_, moveY_, armUp_, 1500)) {
        robot_.sendIk(*joints);
    }
    robot_.sendIk(arm_.claw(0.0, openGripper_, 1000));
    robot_.stop();
}

// 启动阶段的人脸识别姿态：前四关节尽量让机械臂竖直，末端保持向前。
void SorterController::prepareFaceRecognitionPose() {
    moveX_ = facePoseX_;
    moveY_ = facePoseY_;
    moveArm(moveX_, moveY_, facePoseZ_, 1500);
    claw(0.0, openGripper_, 1000);
    robot_.stop();
}

// 识别到人脸后，让机械臂沿 z 方向上下摆动一次。
void SorterController::waveArmForFace() {
    robot_.stop();
    moveArm(facePoseX_, facePoseY_, faceWaveHighZ_, 500);
    moveArm(facePoseX_, facePoseY_, faceWaveLowZ_, 500);
    moveArm(facePoseX_, facePoseY_, faceWaveHighZ_, 500);
    moveArm(facePoseX_, facePoseY_, facePoseZ_, 500);
}

// 每帧处理入口：缩放图像、预处理并根据状态调用对应逻辑。
void SorterController::process(cv::Mat& frame) {
    cv::resize(frame, frame, vision_.image_size);
    cv::Mat hsv = preprocessHsv(frame);

    if (isLineFlag_ == 1) {
        lineFollow(frame);
    } else if (isLineFlag_ == 0) {
        handleColorBlock(frame, hsv);
    } else {
        robot_.stop();
    }

    drawStatus(frame);
}

// 控制小车运动的封装，转发到底层串口发送函数。
void SorterController::carMove(double x, double y, double w) {
    robot_.setVelocity(x, y, w);
}

// 移动机械臂到目标位置（调用逆运动学并发送 IK 命令）。
void SorterController::moveArm(double x, double y, double z, int ms) {
    if (auto joints = arm_.move(x, y, z, ms)) {
        robot_.sendIk(*joints);
    }
}

// 控制爪子旋转与开合。
void SorterController::claw(double spin, double hand, int ms) {
    robot_.sendIk(arm_.claw(spin, hand, ms));
}

// 线跟随逻辑：在配置的多个 ROI 中查找黑线并计算偏航角。
void SorterController::lineFollow(cv::Mat& frame) {
    double weight_sum = 0.0;
    double centroid_sum = 0.0;
    double roi1_area = 0.0;
    double roi2_area = 0.0;
    double roi3_area = 0.0;
    std::vector<cv::Point> centers;

    // 三个 ROI 从近到远看黑线。近处权重大，远处权重小。
    for (const auto& r : vision_.rois) {
        int x = static_cast<int>(r[0]);
        int y = static_cast<int>(r[1]);
        int w = static_cast<int>(r[2]);
        int h = static_cast<int>(r[3]);
        double weight = r[4];
        int roi_id = static_cast<int>(r[5]);
        cv::Rect roi(x, y, w, h);
        roi &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (roi.empty()) continue;

        cv::Mat hsv_roi = preprocessHsv(frame(roi));
        cv::Mat mask;
        cv::inRange(hsv_roi, vision_.black_low, vision_.black_high, mask);
        auto blob = largestBlob(mask, vision_.min_line_area);
        if (!blob.has_value()) continue;

        cv::Point center(blob->center.x + roi.x, blob->center.y + roi.y);
        centers.push_back(center);
        cv::circle(frame, center, 5, cv::Scalar(255, 0, 0), -1);

        if (roi_id == 1) roi1_area = blob->area;
        if (roi_id == 2) roi2_area = blob->area;
        if (roi_id == 3) roi3_area = blob->area;

        if (blob->area < vision_.min_cross_area) {
            centroid_sum += blob->center.x * weight;
            weight_sum += weight;
        }
    }

    for (size_t i = 1; i < centers.size(); ++i) {
        cv::line(frame, centers[i - 1], centers[i], cv::Scalar(0, 255, 0), 2);
    }

    if (overFlag_) {
        if (roi3_area == 0.0 && roi1_area == 0.0 && !midOverFlag_) {
            if (++midOverCnt_ > 5) {
                midOverFlag_ = true;
                midOverCnt_ = 0;
            }
        } else if (roi1_area != 0.0 && roi3_area != 0.0 && midOverFlag_) {
            if (++midOverCnt_ > 20) {
                overFlag_ = false;
                midOverFlag_ = false;
                midOverCnt_ = 0;
            }
        }
        carMove(0.0, 0.0, -0.8);
        return;
    }

    if (!midAdjustPosition_) {
        if (crossingFlag_ == 0 && roi2_area > vision_.min_cross_area) {
            crossingFlag_ = 1;
        } else if (crossingFlag_ == 1 && roi1_area > vision_.min_cross_area) {
            crossingFlag_ = 2;
            ++crossingRecordCnt_;
            timeCnt_ = 0;
        }
    }

    if (crossingFlag_ == 2 && handleCrossing()) {
        return;
    }

    if (weight_sum <= 0.0) {
        carMove(0.0, 0.0, 0.0);
        return;
    }

    double center_pos = centroid_sum / weight_sum;
    double deflection_angle = -std::atan((center_pos - 640.0 / 2.0) / (480.0 / 2.0));
    deflection_angle = deflection_angle * 180.0 / kPi;

    if (midAdjustPosition_) {
        if (std::abs(deflection_angle) < 3.0) {
            carMove(0.0, 0.0, 0.0);
            if (++timeCnt_ > 5) {
                moveX_ = (crossingRecordCnt_ == 3) ? 120.0 : -120.0;
                moveY_ = 80.0;
                moveArm(moveX_, moveY_, armUp_, 1000);
                timeCnt_ = 0;
                isLineFlag_ = 0;
                carBackFlag_ = false;
                midAdjustPosition_ = false;
            }
            return;
        }

        if (roi3_area > vision_.min_cross_area && ++timeCnt_ > 5) {
            carBackFlag_ = true;
        } else if (roi1_area > vision_.min_cross_area) {
            carBackFlag_ = false;
        }
        timeCnt_ = 0;
    }

    double car_x = 0.3 - std::abs(deflection_angle * 0.01);
    car_x = std::max(0.08, car_x);
    double car_w = deflection_angle * 0.03;
    double car_y = 0.0;
    if (carBackFlag_) {
        car_x = -car_x;
        car_w /= 5.0;
        car_y = car_w;
    }
    carMove(car_x, car_y, car_w);
}

// 处理十字路口/交叉口的策略，返回是否已经处理并需要停止后续动作。
bool SorterController::handleCrossing() {
    auto turnFor = [&](double x, double w, int delay) {
        carMove(x, 0.0, w);
        if (++timeCnt_ > delay) {
            crossingFlag_ = 0;
            timeCnt_ = 0;
        }
    };

    if (crossingRecordCnt_ == 2 || crossingRecordCnt_ == 5 || crossingRecordCnt_ == 9) {
        isLineFlag_ = 0;
        crossingFlag_ = 0;
        carMove(0.0, 0.0, 0.0);
        return true;
    }
    if (crossingRecordCnt_ == 3) {
        midAdjustPosition_ = true;
        turnFor(0.1, -0.8, firstTurnDelay_);
        return true;
    }
    if (crossingRecordCnt_ == 4) {
        turnFor(0.1, -0.8, secondTurnDelay_);
        return true;
    }
    if (crossingRecordCnt_ == 6) {
        midAdjustPosition_ = true;
        turnFor(0.1, 0.8, thirdTurnDelay_);
        return true;
    }
    if (crossingRecordCnt_ == 7) {
        turnFor(0.0, 0.8, returnDelay_);
        return true;
    }
    if (crossingRecordCnt_ == 8) {
        turnFor(0.1, 0.8, fourthTurnDelay_);
        return true;
    }
    if (crossingRecordCnt_ == 10) {
        midAdjustPosition_ = true;
        turnFor(0.1, -0.8, fifthTurnDelay_);
        return true;
    }
    if (crossingRecordCnt_ == 11) {
        turnFor(0.1, -0.8, sixthTurnDelay_);
        return true;
    }
    if (crossingRecordCnt_ == 12) {
        carMove(0.0, 0.0, 0.0);
        crossingFlag_ = 1;
        carBackFlag_ = true;
        overFlag_ = true;
        return true;
    }
    if (crossingRecordCnt_ == 13) {
        carMove(0.0, 0.0, 0.0);
        isLineFlag_ = -1;
        return true;
    }

    crossingFlag_ = 0;
    return false;
}

// 识别并处理红/绿/蓝方块的抓取和同色区域放置流程。
void SorterController::handleColorBlock(cv::Mat& frame, const cv::Mat& hsv) {
    BlockColor target_color = capturedColor_;
    auto color_blob = detectColorBlock(hsv, vision_, target_color);
    int block_cx = 320;
    int block_cy = 240;
    bool color_read_success = false;
    std::vector<cv::Point> contour;
    BlockColor visible_color = BlockColor::None;

    if (color_blob.has_value()) {
        block_cx = color_blob->blob.center.x;
        block_cy = color_blob->blob.center.y;
        color_read_success = true;
        contour = color_blob->blob.contour;
        visible_color = color_blob->color;
        cv::Rect box = cv::boundingRect(color_blob->blob.contour);
        cv::rectangle(frame, box, color_blob->draw_color, 2);
        cv::circle(frame, color_blob->blob.center, 5, color_blob->draw_color, -1);
        cv::putText(frame, color_blob->name, cv::Point(box.x, std::max(15, box.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color_blob->draw_color, 1);
    }

    if (moveStatus_ == 0 && color_read_success) {
        if (std::abs(block_cx - 320) > 10) {
            moveX_ += (block_cx > 320) ? -0.5 : 0.5;
        }
        if (std::abs(block_cy - 240) > 10) {
            moveY_ += (block_cy > 240 && moveY_ > 1.0) ? -0.3 : 0.3;
        }
        if (std::abs(block_cx - 320) <= 10 && std::abs(block_cy - 240) <= 10) {
            if (++timeCnt_ > 50) {
                timeCnt_ = 0;
                moveStatus_ = 1;
                capturedColor_ = visible_color;
                spinClaw_ = 0.0;
                double l = std::sqrt(moveX_ * moveX_ + moveY_ * moveY_);
                if (l > 1e-6) {
                    moveX_ = (l + armSkewing_) * moveX_ / l;
                    moveY_ = (l + armSkewing_) * moveY_ / l;
                }
            }
        } else {
            timeCnt_ = 0;
            moveArm(moveX_, moveY_, armUp_, 0);
        }
    } else if (moveStatus_ == 1) {
        ++timeCnt_;
        if (timeCnt_ < 2) {
            spinClaw_ = contourAngleRad(contour);
            claw(spinClaw_, openGripper_, 1000);
        } else if (timeCnt_ < 35) {
            moveArm(moveX_, moveY_, armUp_, 1000);
        } else if (timeCnt_ < 70) {
            moveArm(moveX_, moveY_ + 30.0, graspHeight_, 1000);
        } else if (timeCnt_ >= 105 && timeCnt_ < 140) {
            claw(spinClaw_, closedGripper_, 1000);
        } else if (timeCnt_ >= 175 && timeCnt_ < 210) {
            moveArm(moveX_, moveY_, armUp_, 1000);
        } else if (timeCnt_ >= 245 && timeCnt_ < 280) {
            moveStatus_ = 2;
            moveX_ = armErrX_;
            moveY_ = 150.0;
            spinClaw_ = 0.0;
            moveArm(moveX_, moveY_, armUp_, 1000);
        }
    } else if (moveStatus_ == 2) {
        ++timeCnt_;
        if (!color_read_success) {
            carMove(timeCnt_ < 50 ? 0.1 : -0.1, 0.0, 0.0);
        } else {
            carMove(0.0, 0.0, 0.0);
            moveStatus_ = 3;
            timeCnt_ = 0;
        }
    } else if (moveStatus_ == 3 && color_read_success) {
        if (block_cx - 320 > 50) {
            carMove(crossingRecordCnt_ == 3 ? 0.1 : -0.1, 0.0, 0.0);
        } else if (block_cx - 320 < -50) {
            carMove(crossingRecordCnt_ == 3 ? -0.1 : 0.1, 0.0, 0.0);
        } else if (++timeCnt_ > 40) {
            carMove(0.0, 0.0, 0.0);
            moveStatus_ = 4;
            timeCnt_ = 0;
        }
    } else if (moveStatus_ == 4 && color_read_success) {
        if (std::abs(block_cx - 320) > 10) {
            bool right = block_cx > 320;
            double delta = (crossingRecordCnt_ == 3) ? (right ? 0.5 : -0.5) : (right ? -0.5 : 0.5);
            moveY_ += delta;
        }
        if (std::abs(block_cy - 240) > 10) {
            bool down = block_cy > 240;
            double delta = (crossingRecordCnt_ == 3) ? (down ? -0.3 : 0.3) : (down ? 0.3 : -0.3);
            moveX_ += delta;
        }
        if (std::abs(block_cx - 320) <= 10 && std::abs(block_cy - 240) <= 10) {
            if (++timeCnt_ > 10) {
                timeCnt_ = 0;
                moveStatus_ = 5;
                double l = std::sqrt(moveX_ * moveX_ + moveY_ * moveY_);
                if (l > 1e-6) {
                    moveX_ = (l + armSkewing_) * moveX_ / l * 1.1;
                    moveY_ = (l + armSkewing_) * moveY_ / l * 0.7;
                }
            }
        } else {
            timeCnt_ = 0;
            moveArm(moveX_, moveY_, armUp_, 0);
        }
    } else if (moveStatus_ == 5) {
        ++timeCnt_;
        if (timeCnt_ < 35) {
            moveArm(moveX_, moveY_, armUp_, 1000);
        } else if (timeCnt_ < 70) {
            moveArm(moveX_, moveY_ + 40.0, graspHeight_, 1000);
        } else if (timeCnt_ < 100) {
            claw(0.0, openGripper_, 1000);
        } else if (timeCnt_ >= 135 && timeCnt_ < 170) {
            moveArm(moveX_, moveY_, armUp_, 1000);
        } else if (timeCnt_ >= 200 && timeCnt_ < 235) {
            moveX_ = armErrX_;
            moveY_ = 140.0;
            moveArm(moveX_, moveY_, armUp_, 1000);
        } else if (timeCnt_ >= 270 && timeCnt_ < 300) {
            isLineFlag_ = 1;
            crossingFlag_ = 1;
            moveStatus_ = 0;
            capturedColor_ = BlockColor::None;
            timeCnt_ = 0;
        }
    }
}

// 在图像上绘制状态信息（用于调试/可视化）。
void SorterController::drawStatus(cv::Mat& frame) const {
    std::string text = "line=" + std::to_string(isLineFlag_)
        + " cross=" + std::to_string(crossingRecordCnt_)
        + " move=" + std::to_string(moveStatus_)
        + " color=" + blockColorName(capturedColor_);
    cv::putText(frame, text, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 255), 2);
}

}  // namespace vision_sorter
