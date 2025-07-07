/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://gitee.com/panda_23/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once
#include <memory>
#include <iostream>
#include <Eigen/Dense>
#include "data_bus.h"
#include "qpOASES.hpp"

const uint16_t  mpc_N = 10; //预测步长
const uint16_t  ch = 3; //控制步长
const uint16_t  nx = 12; //State 的数量为12
const uint16_t  nu = 13; //Input 的数量为13

const uint16_t  ncfr_single = 4; //单个足的摩擦约束个数
const uint16_t  ncfr = ncfr_single*2; //双足的摩擦约束个数

const uint16_t  ncstxya = 1; 
const uint16_t  ncstxy_single = ncstxya*4; //单个足的扭矩力矩约束和防足底旋转约束个数
const uint16_t  ncstxy = ncstxy_single*2; //双足的扭矩力矩约束和防足底旋转约束个数

const uint16_t  ncstza = 2; //Z方向半封闭约束
const uint16_t  ncstz_single = ncstza*4; //单个足Z方向半封闭约束个数
const uint16_t  ncstz = ncstz_single*2; //双足Z方向半封闭约束个数
const uint16_t  nc = ncfr + ncstxy + ncstz; //总约束个数

class MPC{
public:
    MPC(double dtIn);

    void    set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag); //设置状态权重L，输入权重K以及输入权重系数u_weight
    void    cal(); //求解MPC优化结果
    void    dataBusRead(const std::shared_ptr<DataBus>& Data); //从总线中读取状态
    void    dataBusWrite(std::shared_ptr<DataBus>& Data); //向总线写入控制力矩

    void    enable(); 
    void    disable();
    bool    get_ENA();

private:
    void    copy_Eigen_to_real_t(qpOASES::real_t* target, Eigen::MatrixXd source, int nRows, int nCols); //将矩阵source中的元素以行向量的方式存储在target中

    bool    EN = false;

    //single rigid body model
    Eigen::Matrix<double,nx,nx>   Ac[mpc_N], A[mpc_N]; //存储预测时域内每个步长的状态矩阵，Ac代表连续时间，A代表离散时间
    Eigen::Matrix<double,nx,nu>   Bc[mpc_N], B[mpc_N]; //存储预测时域内每个步长的控制矩阵，Bc代表连续时间，B代表离散时间
    Eigen::Matrix<double,nx,1>    Cc, C; //状态方程中的常数项，Cc代表连续时间，C代表离散时间

    Eigen::Matrix<double,nx*mpc_N,nx>         Aqp; //将初始状态x0传递到所有预测步的累积状态转移矩阵
    Eigen::Matrix<double,nx*mpc_N,nx*mpc_N>   Aqp1; //多步状态转移的块对角矩阵（通常用于约束或动态模型展开）
    Eigen::Matrix<double,nx*mpc_N,nu*mpc_N>   Bqp1; //完整预测时域内所有控制输入对状态的影响矩阵（块下三角结构）
    Eigen::Matrix<double,nx*mpc_N,nu*ch>      Bqp; //控制时域（ch）内的输入影响矩阵，后续输入固定为零或重复最后一值。
    Eigen::Matrix<double,nx*mpc_N,1>          Cqp1; //预测模型中的 累积常数项（如扰动或线性化偏移），完整预测时域内的常数项
    Eigen::Matrix<double,nx*mpc_N,1>          Cqp; //与控制时域对齐的常数项，用于优化问题的预测模型

    Eigen::Matrix<double,nu*ch,1>           Ufe; // 优化变量中的未来控制输入序列（维度 nu*ch），需通过求解QP确定
    Eigen::Matrix<double,nu,1>              Ufe_pre; // 上一时刻的预测输入序列或初始猜测，用于热启动或输入变化率约束
    Eigen::Matrix<double,nx*mpc_N,1>        Xd; // 参考轨迹
    Eigen::Matrix<double,nx,1>              X_cur; // 当前状态（用于初始化预测）
    Eigen::Matrix<double,nx,1>              X_cal; // 在线计算过程中预测的状态序列（可能用于约束检查或迭代更新）
    Eigen::Matrix<double,nx,1>              X_cal_pre; //上一迭代步的预测状态（用于收敛性判断或初始化）
    Eigen::Matrix<double,nx,1>              dX_cal; //状态预测的变化量（可能用于终止条件或动态调整步长）

    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>    L; //状态误差权重矩阵
    Eigen::Matrix<double,nu*ch, nu*ch>            K, M; //K为控制输入权重矩阵，M为控制增量权重矩阵
    double alpha;
    Eigen::Matrix<double,nu*ch, nu*ch>          H; //目标函数中的二次项矩阵
    Eigen::Matrix<double,nu * ch, 1>              c; //目标函数中的线性矩阵，通常与参考轨迹偏差相关

    Eigen::Matrix<double,nu*ch,1>               u_low, u_up; //控制输入序列的上下限
    Eigen::Matrix<double,nc*ch, nu*ch>          As; //不等式约束的系数矩阵
    Eigen::Matrix<double,nc*ch,1>               bs; //不等式约束的右侧项常数
    double      max[6], min[6];

    double m, g, miu, delta_foot[4];
    Eigen::Matrix<double,3,1>   pCoM;
    Eigen::Matrix<double,6,1>   pf2com, pf2comd, pe;
    Eigen::Matrix<double,6,1>   pf2comi[mpc_N];
    Eigen::Matrix<double,3,3>   Ic;
    Eigen::Matrix<double,3,3>   R_curz[mpc_N];
    Eigen::Matrix<double,3,3>   R_cur;
    Eigen::Matrix<double,3,3>   R_w2f, R_f2w;

    int legStateCur;
    int legStateNext;
    int legState[10];
    double  dt;

    //qpOASES
    qpOASES::QProblem QP;
    qpOASES::real_t qp_H[nu*ch * nu*ch];
    qpOASES::real_t qp_As[nc*ch * nu*ch];
    qpOASES::real_t qp_c[nu*ch];
    qpOASES::real_t qp_lbA[nc*ch];
    qpOASES::real_t qp_ubA[nc*ch];
    qpOASES::real_t qp_lu[nu*ch];
    qpOASES::real_t qp_uu[nu*ch];
    qpOASES::int_t nWSR=100;
    qpOASES::real_t cpu_time=0.1;
    qpOASES::real_t xOpt_iniGuess[nu*ch];

	double			qp_cpuTime;
    int 			qp_Status, qp_nWSR;
};

