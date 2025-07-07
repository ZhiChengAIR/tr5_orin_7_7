/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once

#include <Eigen/Dense>
#include "data_bus.h"

class FootPlacement {
public:
    double kp_vx{0}, kp_vy{0}, kp_wz{0}; //x，y，z轴位置控制增益
    double legLength{1}; //腿长
    double stepHeight{0.1}; //迈步的最高z轴位置
    double phi{0};    // phase varialbe for trajectory generation, must between 0 and 1
    double tSwing{0.4}; // swing time，之前是0.4
    Eigen::Vector3d posStart_W, posDes_W, hipPos_W, STPos_W; //STPos_W是世界坐标系下的脚步位置
    Eigen::Vector3d desV_W, curV_W;//desired velocity and current velocity in world coordinate
    double desWz_W; //desired angular velocity projected on z-axis
    Eigen::Vector3d base_pos;
    double Trajectory(double phase, double des1, double des2); //通过贝塞尔曲线生成摆动腿z方向的位置，des1为脚步的最高点，des2为走步的最终位置，phase==phi的时候脚的高度最高
    void getSwingPos();//计算摆动脚的目的位置，存储在pDesCur中
    void dataBusRead(const std::shared_ptr<DataBus>& robotState);
    void dataBusWrite(std::shared_ptr<DataBus>& robotState);
    DataBus::LegState legState;//record the leg state, used to make swing motion inward

private:
    double pDesCur[3]{0};
    double yawCur;
    double theta0;
    double omegaZ_W;
    double hip_width;
};
