#ifndef VISION_SORTER_FACE_DETECTOR_H
#define VISION_SORTER_FACE_DETECTOR_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>

namespace vision_sorter {

class FaceDetector {
public:
    explicit FaceDetector(const std::string& cascade_path = std::string());

    bool ready() const;
    std::vector<cv::Rect> detect(const cv::Mat& frame);
    void draw(cv::Mat& frame, const std::vector<cv::Rect>& faces) const;

private:
    cv::CascadeClassifier classifier_;
};

}  // namespace vision_sorter

#endif  // VISION_SORTER_FACE_DETECTOR_H
