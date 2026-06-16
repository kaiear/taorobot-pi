#include "args.h"
#include "face_detector.h"
#include "node_common.h"
#include "ros1_robot_control.h"
#include "sorter_controller.h"
#include "vision.h"

#include <chrono>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
    using namespace vision_sorter;

    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    Args args = parseArgs(argc, argv);
    VisionConfig vision;
    loadDefaultVisionConfig(vision);

    cv::VideoCapture cap;
    if (!openConfiguredCamera(args, vision, cap)) {
        return 1;
    }

    Ros1RobotControl robot(nh);
    if (!robot.openPort()) {
        return 1;
    }

    SorterController controller(robot, vision);
    controller.prepareFaceRecognitionPose();

    FaceDetector face_detector;
    bool face_detected = false;
    if (face_detector.ready()) {
        auto start = std::chrono::steady_clock::now();
        ros::Rate face_rate(15.0);
        while (ros::ok()) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                ROS_WARN("camera frame read failed");
                break;
            }
            cv::resize(frame, frame, vision.image_size);

            auto faces = face_detector.detect(frame);
            if (!faces.empty()) {
                face_detected = true;
                controller.waveArmForFace();
                break;
            }

            if (args.show) {
                face_detector.draw(frame, faces);
                cv::imshow("mission_node", frame);
                int key = cv::waitKey(1);
                if (key == 27 || key == 'q') {
                    robot.closePort();
                    return 0;
                }
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start);
            if (elapsed.count() >= 30) {
                break;
            }

            ros::spinOnce();
            face_rate.sleep();
        }
    } else {
        ROS_WARN("face detector unavailable; skip startup face recognition");
    }

    if (!ros::ok()) {
        robot.closePort();
        return 0;
    }

    if (!face_detected) {
        ROS_INFO("no face detected in 30 seconds; start line following");
    }
    controller.initRobot();

    ros::Rate rate(30.0);
    while (ros::ok()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            ROS_WARN("camera frame read failed");
            rate.sleep();
            continue;
        }

        controller.process(frame);

        if (args.show) {
            cv::imshow("mission_node", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    robot.closePort();
    return 0;
}
