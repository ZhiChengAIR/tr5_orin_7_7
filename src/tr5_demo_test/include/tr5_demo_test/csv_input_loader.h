#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>

// 表示一帧仿真输入数据（对应 Mujoco 导出的每一行）
struct MockSensorFrame {
    std::vector<double> motor_pos;
    std::vector<double> motor_vel;
    std::vector<double> tau;
    double baseQuat[4];
    double basePos[3];
    double baseLinVel[3];
    double baseAcc[3];
    double baseAngVel[3];
    double time;
};

class CSVInputLoader {
public:
    CSVInputLoader(const std::string& filename, int jointNum);
    ~CSVInputLoader();

    // 每次调用返回一帧数据，直到文件结尾
    bool nextFrame(const std::shared_ptr<MockSensorFrame>& frame);

private:
    std::ifstream file;
    int jointNum;
};
