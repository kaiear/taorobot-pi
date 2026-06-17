#include "args.h"
#include "node_common.h"
#include "ros_topics.h"
#include "vision.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
    using namespace vision_sorter;

    ros::init(argc, argv, "object_node");
    ros::NodeHandle nh;

    Args args = parseArgs(argc, argv);
    VisionConfig vision;
    loadDefaultVisionConfig(vision);

    cv::VideoCapture cap;
    if (!openConfiguredCamera(args, vision, cap)) {
        return 1;
    }

    auto detected_pub = nh.advertise<std_msgs::Bool>(ros_topics::kObjectDetected, 10);
    auto color_pub = nh.advertise<std_msgs::String>(ros_topics::kObjectColor, 10);
    auto offset_x_pub = nh.advertise<std_msgs::Float32>(ros_topics::kObjectOffsetX, 10);
    auto offset_y_pub = nh.advertise<std_msgs::Float32>(ros_topics::kObjectOffsetY, 10);
    auto area_pub = nh.advertise<std_msgs::Float32>(ros_topics::kObjectArea, 10);
    auto center_x_pub = nh.advertise<std_msgs::Float32>(ros_topics::kObjectCenterX, 10);
    auto center_y_pub = nh.advertise<std_msgs::Float32>(ros_topics::kObjectCenterY, 10);

    ros::Rate rate(30.0);
    while (ros::ok()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            ROS_WARN("camera frame read failed");
            rate.sleep();
            continue;
        }
        cv::resize(frame, frame, vision.image_size);

        ObjectDetection object = detectObjectBlock(frame, vision, BlockColor::None, args.show);
        std_msgs::Bool detected;
        std_msgs::String color;
        std_msgs::Float32 offset_x;
        std_msgs::Float32 offset_y;
        std_msgs::Float32 area;
        std_msgs::Float32 center_x;
        std_msgs::Float32 center_y;
        detected.data = object.detected;
        color.data = object.color_name;
        offset_x.data = static_cast<float>(object.offset_x);
        offset_y.data = static_cast<float>(object.offset_y);
        area.data = static_cast<float>(object.area);
        center_x.data = static_cast<float>(object.center.x);
        center_y.data = static_cast<float>(object.center.y);
        detected_pub.publish(detected);
        color_pub.publish(color);
        offset_x_pub.publish(offset_x);
        offset_y_pub.publish(offset_y);
        area_pub.publish(area);
        center_x_pub.publish(center_x);
        center_y_pub.publish(center_y);

        if (args.show) {
            cv::imshow("object_node", frame);
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
