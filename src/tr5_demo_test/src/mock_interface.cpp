#include "mock_interface.h"
#include "debug_utiles.h"
#include <cmath>
#include <iostream>
#include <fstream>

Mock_Interface::Mock_Interface(int dof) {
    jointNum = dof;
    motor_pos.resize(jointNum, 0.0);
    motor_vel.resize(jointNum, 0.0);
    motor_tau.resize(jointNum, 0.0);
    motor_pos_Old.resize(jointNum, 0.0);

    for (int i = 0; i < 3; ++i) {
        rpy[i] = 0.0;
        basePos[i] = 0.0;
        baseAcc[i] = 0.0;
        baseAngVel[i] = 0.0;
        baseLinVel[i] = 0.0;
        fL[i] = 0.0;
        fR[i] = 0.0;
    }
    timeStep = 0.001; // 模拟 1kHz
}

void Mock_Interface::updateSensorValues() {
    static double t = 0;
    t += timeStep;

    // ====== 模拟四元数（绕 Z 轴旋转） ======
    double angle = 0.05 * sin(2 * M_PI * 0.1 * t); // yaw 角
    baseQuat[0] = 0;                    // x
    baseQuat[1] = 0;                    // y
    baseQuat[2] = sin(angle / 2);       // z
    baseQuat[3] = cos(angle / 2);       // w

    // ====== 四元数转 RPY（与 Mujoco 接口相同） ======
    // 注意顺序：quat = [x y z w]（Mujoco 输出是 xyzw）
    double x = baseQuat[0], y = baseQuat[1], z = baseQuat[2], w = baseQuat[3];

    rpy[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));  // Roll
    rpy[1] = asin(2 * (w * y - z * x));                           // Pitch
    rpy[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)); // Yaw

    for (int i = 0; i < jointNum; ++i) {
        motor_pos_Old[i] = motor_pos[i];
        motor_pos[i] += 0.001;  // 模拟匀速前进
        motor_vel[i] = (motor_pos[i] - motor_pos_Old[i]) / timeStep;
    }

    // 模拟 base 位置
    double prevPos[3] = {basePos[0], basePos[1], basePos[2]};
    basePos[0] = 0.1 * t;
    basePos[1] = 0.0;
    basePos[2] = 1.0;

    for (int i = 0; i < 3; ++i) {
        baseLinVel[i] = (basePos[i] - prevPos[i]) / timeStep;
    }

    // 模拟角速度 = rpy 微分
    static double rpy_old[3] = {0};
    for (int i = 0; i < 3; ++i) {
        baseAngVel[i] = (rpy[i] - rpy_old[i]) / timeStep;
        rpy_old[i] = rpy[i];
    }

    // 模拟加速度：线速度变化
    static double linVel_old[3] = {0};
    for (int i = 0; i < 3; ++i) {
        baseAcc[i] = (baseLinVel[i] - linVel_old[i]) / timeStep;
        linVel_old[i] = baseLinVel[i];
    }

    // 模拟触地力（假设静止站立）
    fL[2] = 300.0; // 左脚支持力
    fR[2] = 300.0; // 右脚支持力
}

void Mock_Interface::setMotorsTorque(std::vector<double> &tauIn) {
    motor_tau = tauIn;
}

void Mock_Interface::dataBusWrite(std::shared_ptr<DataBus> busIn) {
    busIn->motors_pos_cur = motor_pos;
    busIn->motors_vel_cur = motor_vel;

    for (int i = 0; i < 3; ++i) {
        busIn->rpy[i] = rpy[i];
        busIn->basePos[i] = basePos[i];
        busIn->baseLinVel[i] = baseLinVel[i];
        busIn->baseAcc[i] = baseAcc[i];
        busIn->baseAngVel[i] = baseAngVel[i];
        busIn->fL[i] = fL[i];
        busIn->fR[i] = fR[i];
    }

    busIn->setBaseQuat(baseQuat);
    busIn->updateQ_fromRPY();
}

void Mock_Interface::loadFromFrame(const std::shared_ptr<MockSensorFrame>& frame) {
    motor_pos = frame->motor_pos;
    motor_vel = frame->motor_vel;

    std::copy(frame->baseQuat, frame->baseQuat + 4, baseQuat);
    std::copy(frame->basePos, frame->basePos + 3, basePos);
    std::copy(frame->baseLinVel, frame->baseLinVel + 3, baseLinVel);
    std::copy(frame->baseAcc, frame->baseAcc + 3, baseAcc);
    std::copy(frame->baseAngVel, frame->baseAngVel + 3, baseAngVel);

    Eigen::Quaterniond q(frame->baseQuat[3], frame->baseQuat[0], frame->baseQuat[1], frame->baseQuat[2]); // wxyz
    // Eigen::Vector3d rpy_vec = q.toRotationMatrix().eulerAngles(0, 1, 2);
    rpy[0]= atan2(2*(baseQuat[3]*baseQuat[0]+baseQuat[1]*baseQuat[2]),1-2*(baseQuat[0]*baseQuat[0]+baseQuat[1]*baseQuat[1]));
    rpy[1]= asin(2*(baseQuat[3]*baseQuat[1]-baseQuat[0]*baseQuat[2]));
    rpy[2]= atan2(2*(baseQuat[3]*baseQuat[2]+baseQuat[0]*baseQuat[1]),1-2*(baseQuat[1]*baseQuat[1]+baseQuat[2]*baseQuat[2]));
    // for (int i = 0; i < 3; ++i) {
    //     rpy[i] = rpy_vec[i];
    // }
}

void Mock_Interface::logToCSV(double timeNow, const std::vector<double>& tauToLog, const std::vector<double>& wbc_tauJointRes_to_log, const std::vector<double>& motors_tor_des) {
    if (!csvLogFile.is_open()) {
        csvLogFile.open("mock_log.csv", std::ios::out);
        if (!csvLogFile.is_open()) {
            std::cerr << "[Mock_Interface] Failed to open log file!" << std::endl;
            return;
        }
    }

    if (!logHeaderWritten) {
        csvLogFile << "time";
        for (int i = 0; i < jointNum; ++i)
            csvLogFile << ",pos_" << i;
        for (int i = 0; i < jointNum; ++i)
            csvLogFile << ",vel_" << i;
        for (int i = 0; i < jointNum; ++i)
            csvLogFile << ",tau_" << i;  // ✅ header 用 tau_*
        for (int i = 0; i < jointNum; ++i)
            csvLogFile << ",tor_des_" << i;
        for (int i = 0; i < jointNum; ++i)
        csvLogFile << ",wbc_tauJointRes_" << i;
        csvLogFile << ",quat_x,quat_y,quat_z,quat_w";
        for (int i = 0; i < 3; ++i)
            csvLogFile << ",basePos_" << i;
        for (int i = 0; i < 3; ++i)
            csvLogFile << ",baseVel_" << i;
        for (int i = 0; i < 3; ++i)
            csvLogFile << ",acc_" << i;
        for (int i = 0; i < 3; ++i)
            csvLogFile << ",angVel_" << i;
        csvLogFile << std::endl;
        logHeaderWritten = true;
    }

    csvLogFile << timeNow;
    for (int i = 0; i < jointNum; ++i)
        csvLogFile << "," << motor_pos[i];
    for (int i = 0; i < jointNum; ++i)
        csvLogFile << "," << motor_vel[i];
    for (int i = 0; i < jointNum; ++i)
        csvLogFile << "," << tauToLog[i];  // ✅ 使用控制器输出的 torque  
    for (int i = 0; i < jointNum; ++i)
        csvLogFile << "," << motors_tor_des[i];
    for (int i = 0; i < jointNum; ++i)
        csvLogFile << "," << wbc_tauJointRes_to_log[i];

    for (int i = 0; i < 4; ++i)
        csvLogFile << "," << baseQuat[i];
    for (int i = 0; i < 3; ++i)
        csvLogFile << "," << basePos[i];
    for (int i = 0; i < 3; ++i)
        csvLogFile << "," << baseLinVel[i];
    for (int i = 0; i < 3; ++i)
        csvLogFile << "," << baseAcc[i];
    for (int i = 0; i < 3; ++i)
        csvLogFile << "," << baseAngVel[i];

    csvLogFile << std::endl;
}
