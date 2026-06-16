#include "args.h"
#include "face_detector.h"
#include "node_common.h"
#include "ros_topics.h"
#include "vision.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
    using namespace vision_sorter;

    ros::init(argc, argv, "face_node");
    ros::NodeHandle nh;

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

    auto detected_pub = nh.advertise<std_msgs::Bool>(ros_topics::kFaceDetected, 10);
    ros::Rate rate(15.0);
    while (ros::ok()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            ROS_WARN("camera frame read failed");
            rate.sleep();
            continue;
        }
        cv::resize(frame, frame, vision.image_size);

        auto faces = face_detector.detect(frame);
        std_msgs::Bool detected;
        detected.data = !faces.empty();
        detected_pub.publish(detected);

        if (args.show) {
            face_detector.draw(frame, faces);
            cv::imshow("face_node", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
