#pragma once
#include <vector>
#include <memory>
#include "data_bus.h"
#include "csv_input_loader.h"

class Mock_Interface {
public:
    Mock_Interface(int dof);

    void updateSensorValues();                    // 更新模拟传感器数据
    void setMotorsTorque(std::vector<double> &tauIn);  // 接收控制器输出的力矩
    void loadFromFrame(const std::shared_ptr<MockSensorFrame>& frame);
    const double* getBaseQuat() const { return baseQuat; }
    void dataBusWrite(std::shared_ptr<DataBus> busIn);            // 写入总线接口，供控制器读取状态
    void logToCSV(double timeNow, const std::vector<double>& tauToLog, const std::vector<double>& wbc_tauJointRes_to_log, const std::vector<double>& motors_tor_des);

private:
    std::ofstream csvLogFile;
    bool logHeaderWritten = false;

    int jointNum;
    double timeStep = 0.001; // 默认模拟频率 1kHz

    // 关节状态
    std::vector<double> motor_pos;     // 当前关节位置
    std::vector<double> motor_vel;     // 当前关节速度
    std::vector<double> motor_pos_Old; // 上一次的位置，用于计算速度
    std::vector<double> motor_tau;     // 控制器输出的力矩（用于记录）

    // 基座状态
    double baseQuat[4];        // 四元数 (x, y, z, w)
    double rpy[3];             // 欧拉角 (Roll, Pitch, Yaw)
    double basePos[3];         // 基座位置
    double baseLinVel[3];      // 基座线速度
    double baseAcc[3];         // 基座线加速度
    double baseAngVel[3];      // 基座角速度

    // 足底力传感器（模拟用）
    double fL[3];              // 左脚力
    double fR[3];              // 右脚力
};
