#include "apriltag_detector.h"
#include "args.h"
#include "face_detector.h"
#include "node_common.h"
#include "robot_serial.h"
#include "sorter_controller.h"
#include "vision.h"

#include <chrono>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    using namespace vision_sorter;

    installNodeSignalHandlers();

    Args args = parseArgs(argc, argv);
    VisionConfig vision;
    loadDefaultVisionConfig(vision);

    cv::VideoCapture cap;
    if (!openConfiguredCamera(args, vision, cap)) {
        return 1;
    }

    RobotSerial robot(args.serial_port, args.baud, args.dry_run);
    if (!robot.openPort()) {
        return 1;
    }

    SorterController controller(robot, vision);
    controller.prepareFaceRecognitionPose();

    FaceDetector face_detector;
    bool face_detected = false;
    if (face_detector.ready()) {
        auto start = std::chrono::steady_clock::now();
        while (nodeRunningFlag()) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "camera frame read failed\n";
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
                    nodeRunningFlag() = false;
                    break;
                }
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start);
            if (elapsed.count() >= 30) {
                break;
            }
        }
    } else {
        std::cerr << "[face] detector unavailable; skip startup face recognition\n";
    }

    if (!nodeRunningFlag()) {
        robot.closePort();
        return 0;
    }

    if (!face_detected) {
        std::cerr << "[face] no face detected in 30 seconds; start line following\n";
    }
    controller.initRobot();

#ifdef HAVE_APRILTAG
    CameraIntrinsics camera = vision.camera;
    if (args.fx > 0.0 && args.fy > 0.0 && args.cx > 0.0 && args.cy > 0.0) {
        camera.fx = args.fx;
        camera.fy = args.fy;
        camera.cx = args.cx;
        camera.cy = args.cy;
    }
    cv::Mat camera_matrix;
    if (camera.valid()) {
        camera_matrix = camera.matrix();
    } else {
        std::cerr << "[apriltag] camera intrinsics not provided; tag center is detected but pose is not solved\n";
    }
    AprilTagDetector tag_detector(args.tag_size_m, camera_matrix, camera.dist_coeffs);
#else
    std::cerr << "[apriltag] disabled at compile time; using color contour angle only\n";
#endif

    while (nodeRunningFlag()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "camera frame read failed\n";
            break;
        }

#ifdef HAVE_APRILTAG
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        if (auto tag = tag_detector.detect(gray)) {
            cv::circle(frame, tag->center, 8, cv::Scalar(0, 255, 255), 2);
            cv::Point label_org(static_cast<int>(tag->center.x + 8.0), static_cast<int>(tag->center.y - 8.0));
            std::string label = "tag " + std::to_string(tag->id);
            if (tag->has_pose) {
                label += " z=" + std::to_string(tag->tvec[2]).substr(0, 5) + "m";
            }
            cv::putText(frame, label, label_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }
#endif

        controller.process(frame);

        if (args.show) {
            cv::imshow("mission_node", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }
        }
    }

    robot.closePort();
    return 0;
}
