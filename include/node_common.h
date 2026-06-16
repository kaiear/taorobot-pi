#ifndef VISION_SORTER_NODE_COMMON_H
#define VISION_SORTER_NODE_COMMON_H

#include "args.h"
#include "vision.h"

#include <atomic>
#include <csignal>
#include <iostream>
#include <string>

#include <opencv2/videoio.hpp>

namespace vision_sorter {

inline std::atomic<bool>& nodeRunningFlag() {
    static std::atomic<bool> running{true};
    return running;
}

inline void handleNodeSignal(int) {
    nodeRunningFlag() = false;
}

inline void installNodeSignalHandlers() {
    std::signal(SIGINT, handleNodeSignal);
    std::signal(SIGTERM, handleNodeSignal);
}

inline bool loadDefaultVisionConfig(VisionConfig& vision, const std::string& path = "config/camera.yaml") {
    if (loadVisionConfigFromYaml(path, vision)) {
        std::cerr << "Loaded vision config from " << path << "\n";
        return true;
    }
    std::cerr << "Using default vision config\n";
    return false;
}

inline bool openConfiguredCamera(const Args& args, const VisionConfig& vision, cv::VideoCapture& cap) {
    cap.open(args.camera, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cap.open(args.camera);
    }
    if (!cap.isOpened()) {
        std::cerr << "cannot open camera index " << args.camera << "\n";
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, vision.image_size.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, vision.image_size.height);
    cap.set(cv::CAP_PROP_FPS, 30);
    return true;
}

}  // namespace vision_sorter

#endif  // VISION_SORTER_NODE_COMMON_H
