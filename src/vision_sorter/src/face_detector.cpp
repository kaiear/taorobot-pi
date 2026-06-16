#include "face_detector.h"

#include <algorithm>
#include <iostream>

#include <opencv2/imgproc.hpp>

namespace vision_sorter {

FaceDetector::FaceDetector(const std::string& cascade_path) {
    std::vector<std::string> candidates;
    if (!cascade_path.empty()) {
        candidates.push_back(cascade_path);
    } else {
        candidates.push_back(cv::samples::findFile("haarcascade_frontalface_default.xml", false));
        candidates.push_back("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");
        candidates.push_back("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");
    }

    std::string loaded_path;
    for (const auto& path : candidates) {
        if (!path.empty() && classifier_.load(path)) {
            loaded_path = path;
            break;
        }
    }

    if (classifier_.empty()) {
        std::cerr << "[face] cannot load Haar cascade";
        if (!cascade_path.empty()) {
            std::cerr << " from " << cascade_path;
        }
        std::cerr << std::endl;
    } else {
        std::cerr << "[face] loaded cascade " << loaded_path << std::endl;
    }
}

bool FaceDetector::ready() const {
    return !classifier_.empty();
}

std::vector<cv::Rect> FaceDetector::detect(const cv::Mat& frame) {
    std::vector<cv::Rect> faces;
    if (frame.empty() || classifier_.empty()) {
        return faces;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);
    classifier_.detectMultiScale(gray, faces, 1.1, 4, 0, cv::Size(50, 50));
    return faces;
}

void FaceDetector::draw(cv::Mat& frame, const std::vector<cv::Rect>& faces) const {
    for (const auto& face : faces) {
        cv::rectangle(frame, face, cv::Scalar(0, 255, 255), 2);
        cv::putText(frame, "face", cv::Point(face.x, std::max(15, face.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }
}

}  // namespace vision_sorter
