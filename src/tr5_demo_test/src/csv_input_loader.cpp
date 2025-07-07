#include "csv_input_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

CSVInputLoader::CSVInputLoader(const std::string& filename, int jointNum)
    : jointNum(jointNum)
{
    file.open(filename);
    if (!file.is_open()) {
        std::cerr << "[CSVInputLoader] Failed to open file: " << filename << std::endl;
        throw std::runtime_error("CSV open failed");
    }

    std::string header;
    std::getline(file, header);  // 跳过表头
}

CSVInputLoader::~CSVInputLoader() {
    if (file.is_open()) {
        file.close();
    }
}

bool CSVInputLoader::nextFrame(const std::shared_ptr<MockSensorFrame>& frame) {
    std::string line;
    if (!std::getline(file, line)) return false;

    std::stringstream ss(line);
    std::string item;
    std::vector<double> values;

    while (std::getline(ss, item, ',')) {
        values.push_back(std::stod(item));
    }

    // ✅ 检查长度是否够用
    if (values.size() < 155) {
        std::cerr << "[CSVInputLoader] ❌ Line too short, size = " << values.size() << std::endl;
        return false;
    }

    // ========== 数据字段读取 ==========

    int idx = 0;
    frame->time = values[idx++];
    std::cout << "[CSV] time = " << frame->time << std::endl;

    frame->motor_pos.assign(values.begin() + idx, values.begin() + idx + jointNum);
    idx += jointNum;

    frame->motor_vel.assign(values.begin() + idx, values.begin() + idx + jointNum);
    idx += jointNum;

    frame->tau.assign(values.begin() + idx, values.begin() + idx + jointNum);
    idx += jointNum;

    // ===== 🔥 精准读取 baseQuat (列152~155) 🔥 =====
    frame->baseQuat[0] = values[151];  // quat_x
    frame->baseQuat[1] = values[152];  // quat_y
    frame->baseQuat[2] = values[153];  // quat_z
    frame->baseQuat[3] = values[154];  // quat_w

    std::cout << "[CSV] baseQuat = "
              << frame->baseQuat[0] << " "
              << frame->baseQuat[1] << " "
              << frame->baseQuat[2] << " "
              << frame->baseQuat[3] << std::endl;

    // ====== 继续读取 base 状态 ======
    int base_pos_start = 155;
    for (int i = 0; i < 3; ++i) frame->basePos[i]     = values[base_pos_start + i];
    for (int i = 0; i < 3; ++i) frame->baseLinVel[i]  = values[base_pos_start + 3 + i];
    for (int i = 0; i < 3; ++i) frame->baseAcc[i]     = values[base_pos_start + 6 + i];
    for (int i = 0; i < 3; ++i) frame->baseAngVel[i]  = values[base_pos_start + 9 + i];

#ifdef CSV_DEBUG_PRINT_TAU
    std::cout << "[Frame] τ = [";
    for (int i = 0; i < frame.tau.size(); ++i) {
        std::cout << std::fixed << std::setprecision(5) << frame.tau[i];
        if (i < frame.tau.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
#endif

    return true;
}


