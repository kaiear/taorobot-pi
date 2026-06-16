#include "args.h"
#include "node_common.h"
#include "ros_topics.h"
#include "vision.h"

#include <iomanip>
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

    while (nodeRunningFlag()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "camera frame read failed\n";
            break;
        }
        cv::resize(frame, frame, vision.image_size);

        ObjectDetection object = detectObjectBlock(frame, vision, BlockColor::None, args.show);
        std::cout << std::fixed << std::setprecision(3)
                  << ros_topics::kObjectDetected << "=" << (object.detected ? "true" : "false") << " "
                  << ros_topics::kObjectColor << "=" << object.color_name << " "
                  << ros_topics::kObjectOffsetX << "=" << object.offset_x << " "
                  << ros_topics::kObjectOffsetY << "=" << object.offset_y
                  << "\n";

        if (args.show) {
            cv::putText(frame, object.color_name,
                        cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
            cv::imshow("object_node", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }
        }
    }

    return 0;
}
