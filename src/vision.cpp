#include "vision.h"

#include "common.h"

#include <iostream>
#include <iterator>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

namespace vision_sorter {

namespace {

BlockColor colorFromName(const std::string& name) {
    if (name == "red" || name == "R" || name == "r") return BlockColor::Red;
    if (name == "green" || name == "G" || name == "g") return BlockColor::Green;
    if (name == "blue" || name == "B" || name == "b") return BlockColor::Blue;
    return BlockColor::None;
}

cv::Scalar drawColorFor(BlockColor color) {
    switch (color) {
    case BlockColor::Red: return cv::Scalar(0, 0, 255);
    case BlockColor::Green: return cv::Scalar(0, 255, 0);
    case BlockColor::Blue: return cv::Scalar(255, 0, 0);
    case BlockColor::None: break;
    }
    return cv::Scalar(255, 255, 255);
}

std::optional<ColorBlob> detectWithRange(
    const cv::Mat& hsv,
    const VisionConfig& cfg,
    const ColorRange& color_range,
    const cv::Rect& roi) {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U);
    for (const auto& range : color_range.hsv_ranges) {
        cv::Mat one;
        cv::inRange(hsv, range.first, range.second, one);
        mask |= one;
    }

    auto blob = largestBlob(mask(roi), cfg.min_color_area);
    if (!blob.has_value()) {
        return std::nullopt;
    }

    blob->center.x += roi.x;
    blob->center.y += roi.y;
    for (auto& p : blob->contour) {
        p.x += roi.x;
        p.y += roi.y;
    }

    ColorBlob result;
    result.blob = *blob;
    result.color = color_range.color;
    result.name = color_range.name;
    result.draw_color = color_range.draw_color;
    return result;
}

}  // namespace

std::optional<Blob> largestBlob(const cv::Mat& mask, double min_area) {
// 找出 mask 中面积最大的连通组件（至少 min_area），返回 Blob 描述。
// 若没有满足条件的组件，返回 std::nullopt。
// 注意：返回的 contour 坐标相对于传入的 mask。
//
// 输入：二值化掩码；输出：包含中心、面积与轮廓的 Blob。
//
// 示例用途：用于检测线段或红色区域。
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    int best_idx = -1;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area >= min_area && area > best_area) {
            best_area = area;
            best_idx = i;
        }
    }

    if (best_idx < 0) {
        return std::nullopt;
    }

    cv::Moments m = cv::moments(contours[best_idx]);
    if (m.m00 <= 0.0) {
        return std::nullopt;
    }

    Blob b;
    b.center = cv::Point(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
    b.area = best_area;
    b.contour = contours[best_idx];
    return b;
}

cv::Mat preprocessHsv(const cv::Mat& bgr) {
// 将 BGR 转为 HSV，并进行开闭运算去噪，返回处理后的 HSV 图像。
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 先腐蚀再膨胀可以去掉小噪点。
    // 这里保持原程序的 5x5 核，效率和效果都不变。
    cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
    cv::erode(hsv, hsv, kernel, cv::Point(-1, -1), 1);
    cv::dilate(hsv, hsv, kernel, cv::Point(-1, -1), 1);
    return hsv;
}

std::optional<ColorBlob> detectColorBlock(
    const cv::Mat& hsv,
    const VisionConfig& cfg,
    BlockColor target) {
// 在 hsv 图像中按颜色配置查找色块。target 为 None 时，会在红/绿/蓝
// 中选择面积最大的一个；否则只查找指定颜色，用于把物块放回同色区域。
    cv::Rect roi(0, 0, 640, 300);
    roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
    if (roi.empty()) {
        return std::nullopt;
    }

    std::optional<ColorBlob> best;
    for (const auto& color_range : cfg.color_ranges) {
        if (target != BlockColor::None && color_range.color != target) {
            continue;
        }
        auto current = detectWithRange(hsv, cfg, color_range, roi);
        if (!current.has_value()) {
            continue;
        }
        if (!best.has_value() || current->blob.area > best->blob.area) {
            best = current;
        }
        if (target != BlockColor::None) {
            break;
        }
    }
    return best;
}

LineDetection detectLine(cv::Mat& bgr, const VisionConfig& cfg, bool draw) {
    LineDetection result;
    if (bgr.empty()) {
        return result;
    }

    double weight_sum = 0.0;
    double centroid_sum = 0.0;
    int intersection_hits = 0;

    for (const auto& r : cfg.rois) {
        int x = static_cast<int>(r[0]);
        int y = static_cast<int>(r[1]);
        int w = static_cast<int>(r[2]);
        int h = static_cast<int>(r[3]);
        double weight = r[4];
        cv::Rect roi(x, y, w, h);
        roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);
        if (roi.empty()) {
            continue;
        }

        cv::Mat hsv_roi = preprocessHsv(bgr(roi));
        cv::Mat mask;
        cv::inRange(hsv_roi, cfg.black_low, cfg.black_high, mask);
        auto blob = largestBlob(mask, cfg.min_line_area);
        if (!blob.has_value()) {
            continue;
        }

        result.visible = true;
        cv::Point center(blob->center.x + roi.x, blob->center.y + roi.y);
        result.centers.push_back(center);

        if (blob->area >= cfg.min_cross_area) {
            ++intersection_hits;
        } else {
            centroid_sum += blob->center.x * weight;
            weight_sum += weight;
        }

        if (draw) {
            cv::circle(bgr, center, 5, cv::Scalar(255, 0, 0), -1);
        }
    }

    result.intersection = intersection_hits >= 2;
    if (weight_sum > 0.0) {
        double center_pos = centroid_sum / weight_sum;
        double half_width = static_cast<double>(bgr.cols) / 2.0;
        result.error = (center_pos - half_width) / half_width;
        result.angle_deg = -std::atan((center_pos - half_width) / (static_cast<double>(bgr.rows) / 2.0)) * 180.0 / kPi;
    }

    if (draw) {
        for (size_t i = 1; i < result.centers.size(); ++i) {
            cv::line(bgr, result.centers[i - 1], result.centers[i], cv::Scalar(0, 255, 0), 2);
        }
    }
    return result;
}

ObjectDetection detectObjectBlock(
    cv::Mat& bgr,
    const VisionConfig& cfg,
    BlockColor target,
    bool draw) {
    ObjectDetection result;
    if (bgr.empty()) {
        return result;
    }

    cv::Mat hsv = preprocessHsv(bgr);
    auto color_blob = detectColorBlock(hsv, cfg, target);
    if (!color_blob.has_value()) {
        return result;
    }

    result.detected = true;
    result.color = color_blob->color;
    result.color_name = color_blob->name.empty() ? blockColorName(color_blob->color) : color_blob->name;
    result.area = color_blob->blob.area;
    result.center = color_blob->blob.center;
    result.contour = color_blob->blob.contour;
    result.offset_x = (static_cast<double>(result.center.x) - static_cast<double>(bgr.cols) / 2.0)
        / (static_cast<double>(bgr.cols) / 2.0);
    result.offset_y = (static_cast<double>(result.center.y) - static_cast<double>(bgr.rows) / 2.0)
        / (static_cast<double>(bgr.rows) / 2.0);

    if (draw) {
        cv::Rect box = cv::boundingRect(result.contour);
        cv::rectangle(bgr, box, color_blob->draw_color, 2);
        cv::circle(bgr, result.center, 5, color_blob->draw_color, -1);
        cv::putText(bgr, result.color_name, cv::Point(box.x, std::max(15, box.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color_blob->draw_color, 1);
    }

    return result;
}

double contourAngleRad(const std::vector<cv::Point>& contour) {
// 计算轮廓的最小外接旋转矩形并返回其角度（弧度）。
// 若轮廓点太少则返回 0。
    if (contour.size() < 4) {
        return 0.0;
    }

    cv::RotatedRect rect = cv::minAreaRect(contour);
    double angle = rect.angle;
    if (angle < -45.0) {
        angle += 90.0;
    } else if (angle > 45.0) {
        angle -= 90.0;
    }
    return -angle * kPi / 180.0;
}

std::string blockColorName(BlockColor color) {
    switch (color) {
    case BlockColor::Red: return "red";
    case BlockColor::Green: return "green";
    case BlockColor::Blue: return "blue";
    case BlockColor::None: break;
    }
    return "none";
}

bool loadVisionConfigFromYaml(const std::string& path, VisionConfig& cfg) {
    cv::FileStorage fs;
    try {
        fs.open(path, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        std::cerr << "failed to read vision config " << path << ": " << e.what() << std::endl;
        return false;
    }
    if (!fs.isOpened()) return false;

    auto readScalar = [](const cv::FileNode& node, cv::Scalar& s) {
        if (node.isSeq() && node.size() >= 3) {
            s[0] = static_cast<double>(node[0]);
            s[1] = static_cast<double>(node[1]);
            s[2] = static_cast<double>(node[2]);
        }
    };

    cv::FileNode n = fs["image_size"];
    if (!n.empty()) {
        int w = static_cast<int>(n["width"]);
        int h = static_cast<int>(n["height"]);
        if (w > 0 && h > 0) cfg.image_size = cv::Size(w, h);
    }

    readScalar(fs["black_low"], cfg.black_low);
    readScalar(fs["black_high"], cfg.black_high);

    cv::FileNode camera = fs["camera_matrix"];
    if (!camera.empty()) {
        double fx = static_cast<double>(camera["fx"]);
        double fy = static_cast<double>(camera["fy"]);
        double cx = static_cast<double>(camera["cx"]);
        double cy = static_cast<double>(camera["cy"]);
        if (fx > 0.0) cfg.camera.fx = fx;
        if (fy > 0.0) cfg.camera.fy = fy;
        if (cx > 0.0) cfg.camera.cx = cx;
        if (cy > 0.0) cfg.camera.cy = cy;
    }
    cv::FileNode dist = fs["dist_coeffs"];
    if (!dist.empty() && dist.isSeq()) {
        cv::Mat coeffs = cv::Mat::zeros(1, static_cast<int>(dist.size()), CV_64F);
        for (int i = 0; i < static_cast<int>(dist.size()); ++i) {
            coeffs.at<double>(0, i) = static_cast<double>(dist[i]);
        }
        cfg.camera.dist_coeffs = coeffs;
    }

    cv::FileNode colors = fs["color_ranges"];
    if (!colors.empty()) {
        cfg.color_ranges.clear();
        for (const auto& item : colors) {
            ColorRange color_range;
            std::string name = static_cast<std::string>(item["name"]);
            color_range.color = colorFromName(name);
            color_range.name = name.empty() ? blockColorName(color_range.color) : name;
            color_range.draw_color = drawColorFor(color_range.color);
            readScalar(item["draw_color"], color_range.draw_color);

            cv::FileNode ranges = item["ranges"];
            if (ranges.empty()) ranges = item["hsv_ranges"];
            for (const auto& range : ranges) {
                if (range.size() >= 2) {
                    cv::Scalar a, b;
                    readScalar(range[0], a);
                    readScalar(range[1], b);
                    color_range.hsv_ranges.emplace_back(a, b);
                }
            }

            if (color_range.color != BlockColor::None && !color_range.hsv_ranges.empty()) {
                cfg.color_ranges.push_back(color_range);
            }
        }
    }

    cv::FileNode rr = fs["red_ranges"];
    if (!rr.empty()) {
        auto red = std::find_if(cfg.color_ranges.begin(), cfg.color_ranges.end(),
                                [](const ColorRange& c) { return c.color == BlockColor::Red; });
        if (red == cfg.color_ranges.end()) {
            cfg.color_ranges.push_back({BlockColor::Red, "red", cv::Scalar(0, 0, 255), {}});
            red = std::prev(cfg.color_ranges.end());
        }
        red->hsv_ranges.clear();
        for (const auto& item : rr) {
            if (item.size() >= 2) {
                cv::Scalar a, b;
                readScalar(item[0], a);
                readScalar(item[1], b);
                red->hsv_ranges.emplace_back(a, b);
            }
        }
    }

    cv::FileNode rois = fs["rois"];
    if (!rois.empty()) {
        cfg.rois.clear();
        for (const auto& r : rois) {
            if (r.size() >= 6) {
                std::array<double, 6> arr;
                for (int i = 0; i < 6; ++i) arr[i] = static_cast<double>(r[i]);
                cfg.rois.push_back(arr);
            }
        }
    }

    if (fs["min_line_area"].isInt()) cfg.min_line_area = static_cast<int>(fs["min_line_area"]);
    if (fs["min_cross_area"].isInt()) cfg.min_cross_area = static_cast<int>(fs["min_cross_area"]);
    if (fs["min_color_area"].isInt()) cfg.min_color_area = static_cast<int>(fs["min_color_area"]);
    if (fs["min_red_area"].isInt()) cfg.min_color_area = static_cast<int>(fs["min_red_area"]);

    return true;
}

}  // namespace vision_sorter
