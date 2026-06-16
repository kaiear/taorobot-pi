#include "args.h"
#include "node_common.h"
#include "ros_topics.h"
#include "vision.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
    using namespace vision_sorter;

    ros::init(argc, argv, "line_node");
    ros::NodeHandle nh;

    Args args = parseArgs(argc, argv);
    VisionConfig vision;
    loadDefaultVisionConfig(vision);

    cv::VideoCapture cap;
    if (!openConfiguredCamera(args, vision, cap)) {
        return 1;
    }

    auto visible_pub = nh.advertise<std_msgs::Bool>(ros_topics::kLineVisible, 10);
    auto error_pub = nh.advertise<std_msgs::Float32>(ros_topics::kLineError, 10);
    auto intersection_pub = nh.advertise<std_msgs::Bool>(ros_topics::kIntersectionDetected, 10);

    ros::Rate rate(30.0);
    while (ros::ok()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            ROS_WARN("camera frame read failed");
            rate.sleep();
            continue;
        }
        cv::resize(frame, frame, vision.image_size);

        LineDetection line = detectLine(frame, vision, args.show);
        std_msgs::Bool visible;
        std_msgs::Float32 error;
        std_msgs::Bool intersection;
        visible.data = line.visible;
        error.data = static_cast<float>(line.error);
        intersection.data = line.intersection;
        visible_pub.publish(visible);
        error_pub.publish(error);
        intersection_pub.publish(intersection);

        if (args.show) {
            cv::imshow("line_node", frame);
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
