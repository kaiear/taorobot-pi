#include "apriltag_detector.h"
#include "args.h"
#include "face_detector.h"
#include "robot_serial.h"
#include "sorter_controller.h"
#include "vision.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

}  // namespace

// 信号处理函数：收到中断时设置运行标志为 false，主循环将退出。
int main(int argc, char** argv) {
    using namespace vision_sorter;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    Args args = parseArgs(argc, argv);
    VisionConfig vision;
    // 尝试从 config/camera.yaml 加载视觉配置（如果存在则覆盖默认值）
    if (loadVisionConfigFromYaml(std::string("config/camera.yaml"), vision)) {
        std::cerr << "Loaded vision config from config/camera.yaml\n";
    } else {
        std::cerr << "Using default vision config\n";
    }

    // main.cpp 的职责是“把对象组装起来”：
    // 1. 打开摄像头；
    // 2. 打开串口；
    // 3. 创建控制器；
    // 4. 循环读取图像，把图像交给控制器处理。
    cv::VideoCapture cap(args.camera, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cap.open(args.camera);
    }
    if (!cap.isOpened()) {
        std::cerr << "cannot open camera index " << args.camera << std::endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, vision.image_size.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, vision.image_size.height);
    cap.set(cv::CAP_PROP_FPS, 30);

    if (args.face_test) {
        FaceDetector face_detector;
        if (!face_detector.ready()) {
            return 1;
        }
        while (g_running) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "camera frame read failed\n";
                break;
            }
            cv::resize(frame, frame, vision.image_size);
            auto faces = face_detector.detect(frame);
            face_detector.draw(frame, faces);
            cv::imshow("face_test", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }
        }
        return 0;
    }

    RobotSerial serial(args.serial_port, args.baud, args.dry_run);
    if (!serial.openPort()) {
        return 1;
    }

    SorterController controller(serial, vision);
    controller.prepareFaceRecognitionPose();

    FaceDetector face_detector;
    bool face_detected = false;
    if (face_detector.ready()) {
        auto start = std::chrono::steady_clock::now();
        while (g_running) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "camera frame read failed\n";
                break;
            }
            cv::resize(frame, frame, vision.image_size);

            auto faces = face_detector.detect(frame);
            face_detector.draw(frame, faces);
            if (!faces.empty()) {
                face_detected = true;
                controller.waveArmForFace();
                break;
            }

            if (args.show) {
                cv::imshow("vision_sorter", frame);
                int key = cv::waitKey(1);
                if (key == 27 || key == 'q') {
                    g_running = false;
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

    if (!g_running) {
        serial.closePort();
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

    while (g_running) {
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
            cv::imshow("vision_sorter", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }
        }
    }

    serial.closePort();
    return 0;
}
