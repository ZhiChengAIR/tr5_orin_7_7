/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "gait_scheduler.h"

// Note: no double-support here, swing time always equals to stance time
GaitScheduler::GaitScheduler(double tSwingIn, double dtIn) {
    tSwing=tSwingIn; //摆动时间
    dt=dtIn; //时间步长，用于离散化控制，也是步态的更新频率
    phi=0; //相位，用来跟踪步态周期进度
    isIni=false; //是否初始化
    legState=DataBus::RSt; //当前支撑腿，初始化为右腿
    motionState=DataBus::Stand; //运动状态（行走，站立，行走转站立），初始化为站立
    enableNextStep= false;
    FLest.resize(6);
    FRest.resize(6);
}

void GaitScheduler::dataBusRead(const std::shared_ptr<DataBus>& robotState) {
    model_nv = robotState->model_nv;
    torJoint = Eigen::VectorXd::Zero(model_nv - 6);
    
    for (int i = 0; i < model_nv - 6; i++) {
        torJoint[i] = robotState->motors_tor_cur[i];
    }
    
    dyn_M = robotState->dyn_M;
    dyn_Non = robotState->dyn_Non;
    J_l = robotState->J_l;
    dJ_l = robotState->dJ_l;
    J_r = robotState->J_r;
    dJ_r = robotState->dJ_r;
    Fz_L_m = robotState->fL[2];
    Fz_R_m = robotState->fR[2];
    hip_l_pos_W = robotState->hip_l_pos_W;
    hip_r_pos_W = robotState->hip_r_pos_W;
    fe_r_pos_W = robotState->fe_r_pos_W;
    fe_l_pos_W = robotState->fe_l_pos_W;
    fe_l_rot_W = robotState->fe_l_rot_W;
    fe_r_rot_W = robotState->fe_r_rot_W;
    dq = robotState->dq;
    motionState = robotState->motionState;
}


void GaitScheduler::dataBusWrite(std::shared_ptr<DataBus>& robotState) {
    robotState->tSwing = tSwing;
    robotState->swingStartPos_W = swingStartPos_W;
    robotState->stanceDesPos_W = stanceStartPos_W;
    robotState->posHip_W = posHip_W;
    robotState->posST_W = posST_W;
    robotState->theta0 = theta0;
    robotState->legState = legState;
    robotState->legStateNext = legStateNext;
    robotState->phi = phi;
    robotState->FL_est = FLest;
    robotState->FR_est = FRest;
    
    if (legState == DataBus::LSt) {
        robotState->stance_fe_pos_cur_W = fe_l_pos_W;
        robotState->stance_fe_rot_cur_W = fe_l_rot_W;
    } else {
        robotState->stance_fe_pos_cur_W = fe_r_pos_W;
        robotState->stance_fe_rot_cur_W = fe_r_rot_W;
    }
    
    robotState->motionState = motionState;
}


void GaitScheduler::step(std::array<std::array<double, 6>, 2> force_data_in) {
    Eigen::VectorXd tauAll;
    tauAll=Eigen::VectorXd::Zero(model_nv);
    tauAll.block(6,0,model_nv-6,1)=torJoint;
    //std::cout << "11111" << std::endl;
    //FLest= -pseudoInv_SVD(J_l * dyn_M.inverse() * J_l.transpose()) * (J_l * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_l * dq);
    //FRest= -pseudoInv_SVD(J_r * dyn_M.inverse() * J_r.transpose()) * (J_r * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_r * dq);
    for (int i = 0; i < 6; i++) {
	FLest[i] = force_data_in[0][i];
	FRest[i] = force_data_in[1][i];
//	std::cout << "gait FLest " << FLest[i] << std::endl;
  //      std::cout << "gait FRest " << FRest[i] << std::endl;	
    }
    //std::cout << "2222222" << std::endl;
    //FLest= force_data_in[0];
    //FRest= force_data_in[1];
    double dPhi{0};

    if (motionState==DataBus::Walk2Stand)
    {
        enableNextStep= false;
        if (touchDown)
            motionState=DataBus::Stand;
    }

    if (motionState==DataBus::Stand){
        dPhi=0;
        phi=0;  // need to refined
        isIni= false;
        legState=DataBus::RSt;
        enableNextStep = false;
    }
    else if (motionState==DataBus::Walk)
    {
        enableNextStep = true;
        dPhi=1.0/tSwing*dt;
    }
    else if (motionState==DataBus::Walk2Stand)
        dPhi=1.0/tSwing*dt;

    phi+=dPhi;
    if (enableNextStep)
        touchDown=false;

    if (!isIni){
        isIni=true;
        if (legState == DataBus::LSt){      // here define which leg support first
            swingStartPos_W=fe_r_pos_W;     // 如果左脚是支撑腿，那么右脚的位置就是摆腿的起点
            stanceStartPos_W=fe_l_pos_W;    // 那么左脚的位置就是支撑腿的起点
        }
        else{
            swingStartPos_W=fe_l_pos_W;     // 如果右脚是支撑腿，那么左脚的位置就是摆腿的起点
            stanceStartPos_W=fe_r_pos_W;    // 那么右脚的位置就是支撑腿的起点
        }
    }
    //std::cout << "333333" << std::endl;

    if (legState == DataBus::LSt && FRest[2] >= FzThrehold && phi>=0.6){
        if (enableNextStep){
            legState = DataBus::RSt;
            swingStartPos_W=fe_l_pos_W;
            stanceStartPos_W=fe_r_pos_W;
            phi=0;
        }
    }
    else if(legState == DataBus::RSt && FLest[2] >= FzThrehold && phi>=0.6){
        if (enableNextStep) {
            legState = DataBus::LSt;
            swingStartPos_W = fe_r_pos_W;
            stanceStartPos_W = fe_l_pos_W;
            phi = 0;
        }
    }

    if (!enableNextStep)
    {
        if (legState == DataBus::LSt && FRest[2] >= 200) { //原来是200
            touchDown = true;
        }
        if (legState == DataBus::RSt && FLest[2] >= 200) { //原来是200
            touchDown = true;
        }
    }

    if (phi>=1)
    {
        phi=1;
    }
    if (legState == DataBus::LSt) {
        posHip_W = hip_r_pos_W;
        posST_W = fe_l_pos_W;
        theta0 = -3.1415 * 0.5;
        legStateNext = DataBus::RSt;
    } else {
        posHip_W = hip_l_pos_W;
        posST_W = fe_r_pos_W;
        theta0 = 3.1415 * 0.5;
        legStateNext = DataBus::LSt;
    }
//    FRest.clear();
   // FLest.clear();
}
















