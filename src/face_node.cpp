#include "args.h"
#include "face_detector.h"
#include "node_common.h"
#include "ros_topics.h"
#include "vision.h"

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

    FaceDetector face_detector;
    if (!face_detector.ready()) {
        return 1;
    }

    while (nodeRunningFlag()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "camera frame read failed\n";
            break;
        }
        cv::resize(frame, frame, vision.image_size);

        auto faces = face_detector.detect(frame);
        bool detected = !faces.empty();
        std::cout << ros_topics::kFaceDetected << "=" << (detected ? "true" : "false") << "\n";

        if (args.show) {
            face_detector.draw(frame, faces);
            cv::putText(frame, detected ? "face detected" : "no face",
                        cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
            cv::imshow("face_node", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }
        }
    }

    return 0;
}
