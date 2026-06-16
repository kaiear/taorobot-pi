#include "args.h"

#include <cstdlib>
#include <iostream>

namespace vision_sorter {

Args parseArgs(int argc, char** argv) {
    Args args;

    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];

        // 这个 lambda 用来读取某个参数后面的值。
        // 例如 --camera 0，读到 --camera 后，下一项 0 就是它的值。
        auto needValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << std::endl;
                std::exit(2);
            }
            return argv[++i];
        };

        if (key == "--camera") {
            args.camera = std::stoi(needValue("--camera"));
        } else if (key == "--serial") {
            args.serial_port = needValue("--serial");
        } else if (key == "--baud") {
            args.baud = std::stoi(needValue("--baud"));
        } else if (key == "--dry-run") {
            args.dry_run = true;
        } else if (key == "--show") {
            args.show = true;
        } else if (key == "--face-test") {
            args.face_test = true;
        } else if (key == "--tag-size") {
            args.tag_size_m = std::stod(needValue("--tag-size"));
        } else if (key == "--fx") {
            args.fx = std::stod(needValue("--fx"));
        } else if (key == "--fy") {
            args.fy = std::stod(needValue("--fy"));
        } else if (key == "--cx") {
            args.cx = std::stod(needValue("--cx"));
        } else if (key == "--cy") {
            args.cy = std::stod(needValue("--cy"));
        } else if (key == "--help" || key == "-h") {
            std::cout << "usage: " << argv[0]
                      << " [--camera 0] [--serial /dev/serial0] [--baud 115200]"
                      << " [--tag-size 0.05 --fx FX --fy FY --cx CX --cy CY]"
                      << " [--dry-run] [--show] [--face-test]\n";
            std::exit(0);
        }
    }

    return args;
}

}  // namespace vision_sorter
