#include "apriltag_detector.h"

#ifdef HAVE_APRILTAG

#include <opencv2/calib3d.hpp>

namespace vision_sorter {

// AprilTagDetector 构造：初始化标签族与检测器，保存相机参数（可选）。
AprilTagDetector::AprilTagDetector(double tag_size_m, const cv::Mat& camera_matrix, const cv::Mat& dist_coeff)
    : tag_size_m_(tag_size_m), camera_matrix_(camera_matrix.clone()), dist_coeff_(dist_coeff.clone()) {
    family_ = tag36h11_create();
    detector_ = apriltag_detector_create();
    apriltag_detector_add_family(detector_, family_);
}

// 析构函数：销毁检测器与标签族。
AprilTagDetector::~AprilTagDetector() {
    if (detector_) apriltag_detector_destroy(detector_);
    if (family_) tag36h11_destroy(family_);
}

// 在灰度图上检测 AprilTag，若提供相机参数则计算位姿。
std::optional<TagPose> AprilTagDetector::detect(const cv::Mat& gray) {
    image_u8_t image{
        static_cast<int32_t>(gray.cols),
        static_cast<int32_t>(gray.rows),
        static_cast<int32_t>(gray.step),
        const_cast<uint8_t*>(gray.data)
    };

    zarray_t* detections = apriltag_detector_detect(detector_, &image);
    if (!detections || zarray_size(detections) == 0) {
        if (detections) apriltag_detections_destroy(detections);
        return std::nullopt;
    }

    apriltag_detection_t* det = nullptr;
    zarray_get(detections, 0, &det);

    TagPose pose;
    pose.id = det->id;
    pose.center = cv::Point2d(det->c[0], det->c[1]);

    if (!camera_matrix_.empty() && tag_size_m_ > 0.0) {
        double s = tag_size_m_ / 2.0;
        std::vector<cv::Point3d> object_points{
            {-s, -s, 0.0}, {s, -s, 0.0}, {s, s, 0.0}, {-s, s, 0.0}
        };
        std::vector<cv::Point2d> image_points{
            {det->p[0][0], det->p[0][1]},
            {det->p[1][0], det->p[1][1]},
            {det->p[2][0], det->p[2][1]},
            {det->p[3][0], det->p[3][1]},
        };
        pose.has_pose = cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeff_,
                                     pose.rvec, pose.tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
    }

    apriltag_detections_destroy(detections);
    return pose;
}

}  // namespace vision_sorter

#endif
