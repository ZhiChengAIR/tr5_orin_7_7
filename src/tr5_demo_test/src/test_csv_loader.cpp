#include "csv_input_loader.h"
#include <iostream>
#include <iomanip>

int main() {
    const std::string filename = "../build/mujoco_log.csv";
    const int jointNum = 30;  // 修改成你的关节数

    try {
        CSVInputLoader loader(filename, jointNum);
        MockSensorFrame frame;

        int frameIdx = 0;
        while (loader.nextFrame(frame)) {
            std::cout << "Frame " << frameIdx++ << ":" << std::endl;

            std::cout << "  motor_pos: ";
            for (double p : frame.motor_pos) std::cout << std::fixed << std::setprecision(3) << p << " ";
            std::cout << std::endl;

            std::cout << "  motor_vel: ";
            for (double v : frame.motor_vel) std::cout << std::fixed << std::setprecision(3) << v << " ";
            std::cout << std::endl;

            std::cout << "  baseQuat: ";
            for (int i = 0; i < 4; ++i) std::cout << std::fixed << std::setprecision(3) << frame.baseQuat[i] << " ";
            std::cout << std::endl;

            std::cout << "  basePos: ";
            for (int i = 0; i < 3; ++i) std::cout << frame.basePos[i] << " ";
            std::cout << std::endl;

            std::cout << "----------------------------" << std::endl;

            // 只测试前10帧
            if (frameIdx >= 10) break;
        }

    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    return 0;
}
