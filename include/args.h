#ifndef VISION_SORTER_ARGS_H
#define VISION_SORTER_ARGS_H

#include <string>

namespace vision_sorter {

// Args 这个结构体专门保存命令行参数。
// 结构体和类很像，也可以有成员变量。这里用结构体是因为它只负责“装数据”。
struct Args {
    int camera = 0;
    std::string serial_port = "/dev/serial0";
    int baud = 115200;
    bool dry_run = false;
    bool show = false;
    bool face_test = false;
    double tag_size_m = 0.05;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

Args parseArgs(int argc, char** argv);

}  // namespace vision_sorter

#endif  // VISION_SORTER_ARGS_H
