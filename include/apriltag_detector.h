#ifndef VISION_SORTER_APRILTAG_DETECTOR_H
#define VISION_SORTER_APRILTAG_DETECTOR_H

#include <optional>

#include <opencv2/core.hpp>

#ifdef HAVE_APRILTAG
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
}
#endif

namespace vision_sorter {

// TagPose 保存 AprilTag 的检测结果。
// 如果没有传入相机内参，只能得到 id 和图像中心点；
// 如果传入 fx/fy/cx/cy，还可以得到 rvec/tvec 位姿。
struct TagPose {
    int id = -1;
    cv::Point2d center;
    cv::Vec3d rvec{0.0, 0.0, 0.0};
    cv::Vec3d tvec{0.0, 0.0, 0.0};
    bool has_pose = false;
};

#ifdef HAVE_APRILTAG

class AprilTagDetector {
public:
    AprilTagDetector(double tag_size_m, const cv::Mat& camera_matrix, const cv::Mat& dist_coeff);
    ~AprilTagDetector();

    std::optional<TagPose> detect(const cv::Mat& gray);

private:
    apriltag_family_t* family_ = nullptr;
    apriltag_detector_t* detector_ = nullptr;
    double tag_size_m_ = 0.05;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;
};

#endif

}  // namespace vision_sorter

#endif  // VISION_SORTER_APRILTAG_DETECTOR_H
