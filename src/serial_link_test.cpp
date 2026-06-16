#include "robot_serial.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {

struct Args {
    std::string serial = "/dev/serial0";
    int baud = 115200;
    bool beep = false;
    bool reset = false;
    bool move_test = false;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        auto needValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if (key == "--serial") {
            args.serial = needValue("--serial");
        } else if (key == "--baud") {
            args.baud = std::stoi(needValue("--baud"));
        } else if (key == "--beep") {
            args.beep = true;
        } else if (key == "--reset") {
            args.reset = true;
        } else if (key == "--move-test") {
            args.move_test = true;
        } else if (key == "--help" || key == "-h") {
            std::cout << "usage: " << argv[0]
                      << " [--serial /dev/serial0] [--baud 115200]"
                      << " [--beep] [--reset] [--move-test]\n";
            std::exit(0);
        } else {
            std::cerr << "unknown argument: " << key << "\n";
            std::exit(2);
        }
    }
    return args;
}

}  // namespace

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);
    vision_sorter::RobotSerial serial(args.serial, args.baud, false);

    if (!serial.openPort()) {
        return 1;
    }

    std::cout << "opened " << args.serial << " @" << args.baud << "\n";
    std::cout << "send stop frame\n";
    serial.stop();

    if (args.beep) {
        std::cout << "send beep frame\n";
        serial.beep(2, 100, 100);
    }

    if (args.reset) {
        std::cout << "send reset frame\n";
        serial.resetRobot();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    if (args.move_test) {
        std::cerr << "MOVE TEST: lift wheels before running this command.\n";
        std::cout << "send 0.05 m/s forward for 0.5 s\n";
        serial.setVelocity(0.05, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        serial.stop();
    }

    serial.closePort();
    return 0;
}
