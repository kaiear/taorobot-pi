#ifndef VISION_SORTER_SORTER_CONTROLLER_H
#define VISION_SORTER_SORTER_CONTROLLER_H

#include "arm_kinematics.h"
#include "robot_control.h"
#include "vision.h"

#include <opencv2/core.hpp>

namespace vision_sorter {

// SorterController 是整个任务的“总控制器”。
// 它不直接打开摄像头，也不直接配置串口；这些由 main 和 RobotSerial 做。
// 它只根据每一帧图像决定：小车怎么走、机械臂什么时候抓、什么时候放。
class SorterController {
public:
    SorterController(RobotControl& robot, const VisionConfig& vision);

    void initRobot();
    void prepareFaceRecognitionPose();
    void waveArmForFace();
    void process(cv::Mat& frame);

private:
    void carMove(double x, double y, double w);
    void moveArm(double x, double y, double z, int ms);
    void claw(double spin, double hand, int ms);
    void lineFollow(cv::Mat& frame);
    bool handleCrossing();
    void handleColorBlock(cv::Mat& frame, const cv::Mat& hsv);
    void drawStatus(cv::Mat& frame) const;

    RobotControl& robot_;
    const VisionConfig& vision_;
    ArmKinematics arm_;

    int timeCnt_ = 0;
    double moveX_ = 0.0;
    double moveY_ = 150.0;
    int moveStatus_ = 0;
    double spinClaw_ = 0.0;
    int isLineFlag_ = 1;
    int crossingFlag_ = 0;
    int crossingRecordCnt_ = 0;
    bool midAdjustPosition_ = false;
    bool overFlag_ = false;
    bool midOverFlag_ = false;
    int midOverCnt_ = 0;
    bool carBackFlag_ = false;
    BlockColor capturedColor_ = BlockColor::None;

    const double armErrX_ = -5.0;
    const double armUp_ = 170.0;
    const double facePoseX_ = 0.0;
    const double facePoseY_ = 170.0;
    const double facePoseZ_ = 210.0;
    const double faceWaveLowZ_ = 185.0;
    const double faceWaveHighZ_ = 230.0;
    const double armSkewing_ = 30.0;
    const double graspHeight_ = 50.0;
    const double openGripper_ = 0.8;
    const double closedGripper_ = -0.3;
    const int firstTurnDelay_ = 50;
    const int secondTurnDelay_ = 50;
    const int thirdTurnDelay_ = 60;
    const int fourthTurnDelay_ = 50;
    const int fifthTurnDelay_ = 60;
    const int sixthTurnDelay_ = 50;
    const int returnDelay_ = 100;
};

}  // namespace vision_sorter

#endif  // VISION_SORTER_SORTER_CONTROLLER_H
