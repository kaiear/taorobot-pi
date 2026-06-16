#ifndef VISION_SORTER_VISION_H
#define VISION_SORTER_VISION_H

#include <array>
#include <optional>
#include <utility>
#include <vector>
#include <string>

#include <opencv2/core.hpp>

namespace vision_sorter {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);

    bool valid() const {
        return fx > 0.0 && fy > 0.0 && cx > 0.0 && cy > 0.0;
    }

    cv::Mat matrix() const {
        return (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                                          0.0, fy, cy,
                                          0.0, 0.0, 1.0);
    }
};

enum class BlockColor {
    None,
    Red,
    Green,
    Blue
};

struct ColorRange {
    BlockColor color = BlockColor::None;
    std::string name;
    cv::Scalar draw_color{255, 255, 255};
    std::vector<std::pair<cv::Scalar, cv::Scalar>> hsv_ranges;
};

// VisionConfig 集中保存视觉参数。
// 把阈值和 ROI 放在这里，后面调参时不用去状态机代码里到处找。
struct VisionConfig {
    cv::Size image_size{640, 480};
    CameraIntrinsics camera;
    cv::Scalar black_low{0, 0, 0};
    cv::Scalar black_high{179, 255, 85};
    std::vector<ColorRange> color_ranges{
        {BlockColor::Red, "red", cv::Scalar(0, 0, 255), {
            {cv::Scalar(0, 110, 100), cv::Scalar(10, 255, 255)},
            {cv::Scalar(160, 110, 100), cv::Scalar(179, 255, 255)}
        }},
        {BlockColor::Green, "green", cv::Scalar(0, 255, 0), {
            {cv::Scalar(40, 70, 0), cv::Scalar(90, 255, 255)}
        }},
        {BlockColor::Blue, "blue", cv::Scalar(255, 0, 0), {
            {cv::Scalar(100, 150, 70), cv::Scalar(130, 255, 255)}
        }}
    };
    std::vector<std::array<double, 6>> rois{
        {0, 260, 640, 20, 0.25, 1},
        {0, 130, 640, 20, 0.20, 2},
        {0, 0, 640, 20, 0.10, 3}
    };
    int min_line_area = 400;
    int min_cross_area = 4000;
    int min_color_area = 1000;
};

// Blob 表示图像里识别出来的一块区域。
// center 是中心点，area 是面积，contour 是轮廓点。
struct Blob {
    cv::Point center;
    double area = 0.0;
    std::vector<cv::Point> contour;
};

struct ColorBlob {
    Blob blob;
    BlockColor color = BlockColor::None;
    std::string name;
    cv::Scalar draw_color{255, 255, 255};
};

struct LineDetection {
    bool visible = false;
    bool intersection = false;
    double error = 0.0;
    double angle_deg = 0.0;
    std::vector<cv::Point> centers;
};

struct ObjectDetection {
    bool detected = false;
    BlockColor color = BlockColor::None;
    std::string color_name = "unknown";
    double offset_x = 0.0;
    double offset_y = 0.0;
    double area = 0.0;
    cv::Point center{0, 0};
    std::vector<cv::Point> contour;
};

std::optional<Blob> largestBlob(const cv::Mat& mask, double min_area);
cv::Mat preprocessHsv(const cv::Mat& bgr);
std::optional<ColorBlob> detectColorBlock(
    const cv::Mat& hsv,
    const VisionConfig& cfg,
    BlockColor target = BlockColor::None);
LineDetection detectLine(cv::Mat& bgr, const VisionConfig& cfg, bool draw = false);
ObjectDetection detectObjectBlock(
    cv::Mat& bgr,
    const VisionConfig& cfg,
    BlockColor target = BlockColor::None,
    bool draw = false);
double contourAngleRad(const std::vector<cv::Point>& contour);
std::string blockColorName(BlockColor color);

// 从 YAML 文件加载视觉配置（例如 image_size, red_ranges, thresholds 等）。
// 使用 OpenCV 的 FileStorage 来读取 YAML 字段，遇到缺失字段时保留默认值。
bool loadVisionConfigFromYaml(const std::string& path, VisionConfig& cfg);

}  // namespace vision_sorter

#endif  // VISION_SORTER_VISION_H
