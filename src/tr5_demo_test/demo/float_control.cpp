/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "foot_placement.h"
#include "gait_scheduler.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"



// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("../models/scene_float.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);


//************************
// main function
int main(int argc, const char** argv)
{
    // ini classes
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco，部署在实机需要修改为执行器和传感器的interface
    Pin_KinDyn kinDynSolver("../models/TR5.urdf"); // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    PVT_Ctr pvtCtr(mj_model->opt.timestep,"../common/joint_ctrl_config.json");// PVT joint control
    DataLogger logger("../record/datalog.log"); // data logger

    // variables ini
    double stand_legLength = 1.07; //-0.95; // desired baselink height
    double foot_height =0.07; // distance between the foot ankel joint and the bottom
    double xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.229;
    //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv=kinDynSolver.model_nv;

    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_des(model_nv-6,0);
    std::vector<double> motors_pos_cur(model_nv-6,0);
    std::vector<double> motors_vel_des(model_nv-6,0);
    std::vector<double> motors_vel_cur(model_nv-6,0);
    std::vector<double> motors_tau_des(model_nv-6,0);
    std::vector<double> motors_tau_cur(model_nv-6,0);
    Eigen::Vector3d fe_l_pos_L_des={0, 0, 0};
    Eigen::Vector3d fe_r_pos_L_des={0, 0, 0};
    Eigen::Vector3d fe_l_eul_L_des={0, 0, 0};
    Eigen::Vector3d fe_r_eul_L_des={0, 0, 0};
    Eigen::Matrix3d fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_L_des={0, 0, 0};
    Eigen::Vector3d hd_r_pos_L_des={0, 0, 0};
    Eigen::Vector3d hd_l_eul_L_des={0, 0, 0};
    Eigen::Vector3d hd_r_eul_L_des={0, 0, 0};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

    // std::cout << "motors_pos_des:"; 
    // for (double value : motors_pos_des){
    //     std::cout << value << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "motors_pos_cur:"; 
    // for (double value : motors_pos_cur){
    //     std::cout << value << " ";
    // }
    // std::cout << std::endl;


    // std::cout << "fe_l_rot_des: " << fe_l_rot_des << std::endl;
    // std::cout << "fe_l_pos_L_des: " << fe_l_pos_L_des << std::endl;
    // std::cout << "fe_r_rot_des: " << fe_r_rot_des << std::endl;
    // std::cout << "fe_r_pos_L_des: " << fe_r_pos_L_des << std::endl;
    auto resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
    auto resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);
    
    // std::cout << "resLeg.jointPosRes: " << resLeg.jointPosRes.transpose() << std::endl;
    // std::cout << "resHand.jointPosRes: " << resHand.jointPosRes.transpose() << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(10)); // 暂停 2 秒

    // register variable name for data logger
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << "------------------------- data logger --------------------------" << std::endl;
    logger.addIterm("simTime", 1);
    logger.addIterm("motors_pos_cur",model_nv-6);
    logger.addIterm("motors_pos_des",model_nv-6);
    logger.addIterm("motors_tau_cur",model_nv-6);
    logger.addIterm("motors_vel_des",model_nv-6);
    logger.addIterm("motors_vel_cur",model_nv-6);
    logger.finishItermAdding();

    /// ----------------- sim Loop ---------------
    double simEndTime=20;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double startSteppingTime=2;
    double startWalkingTime=3;

    // init UI: GLFW
    // std::cout << "----------------------------------------------------------" << std::endl;
    // std::cout << "------------------------- init UI: GLFW --------------------------" << std::endl;
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    // std::cout << "！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！" << std::endl;
    uiController.createWindow("Demo",false);
    // std::cout << "----------------------- Demo end -----------------------" << std::endl;
    // std::cout << "----------------------------------------------------------" << std::endl;
    // std::cout << "------------------------- while --------------------------" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(10)); // 暂停 2 秒
    while( !glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart=mj_data->time;
        while( mj_data->time - simstart < 1.0/60.0 )
        {
            mj_step(mj_model, mj_data);

            simTime=mj_data->time;
            // printf("-------------%.3f s------------\n",simTime);
            mj_interface.updateSensorValues(); //部署在实机上应改为实机的sensorbus read
            mj_interface.dataBusWrite(RobotState);//部署在实机上应改为实机的sensorbus write

            
            // inverse kinematics
            // 双脚末端位置（x, y, z）、姿态(rpy)
            fe_l_pos_L_des<< 0.01, 0.13, -stand_legLength;      // 向前x，向右y，向上z。 base_link为中心
            fe_r_pos_L_des<< -0.01, -0.13, -stand_legLength;      // 向前x，向右y，向上z。 base_link为中心
            fe_l_eul_L_des<<-0.00, -0.018, -0.000;
            fe_r_eul_L_des<< 0.00, -0.018, 0.000;
            fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));  // roll, pitch, yaw
            fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

            // 双手末端位置（x, y, z）、姿态(rpy)
            // hd_l_pos_L_des={0, 0.4, -0.2};       // 向前x，向右y，向上z。 base_link为中心
            // hd_r_pos_L_des={0, -0.4, -0.2};      // 向前x，向右y，向上z。 base_link为中心
            hd_l_pos_L_des={0, 0.3, -0.3};        // og
            hd_r_pos_L_des={0, -0.3, -0.3};       // og
            hd_l_eul_L_des={0, 0, 0};
            hd_r_eul_L_des={0, 0, 0};
            // hd_l_eul_L_des={-1.253, 0.122, -1.732};     // og
            // hd_r_eul_L_des={1.253, 0.122, 1.732};       // og
            hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
            hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

            resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
            resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);
            // for (i=0; i <= resLeg.size();i++)
            // std::cout << "resLeg: " << resLeg <<std::endl;
            // std::cout<<resHand.itr<<std::endl;
            // std::cout<<resHand.err.transpose()<<std::endl;

            // Enter here functions to send actuator commands, like:
            // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
            // get the final joint command
            RobotState.motors_pos_des= eigen2std(resLeg.jointPosRes+resHand.jointPosRes);
            RobotState.motors_vel_des=motors_vel_des;
            RobotState.motors_tor_des=motors_tau_des;
            std::cout << "RobotState.motors_pos_des.size(): " << RobotState.motors_pos_des.size() << std::endl;
            std::cout << "hands postions: " << std::endl;
            for (int i=0; i<14; i++){
                std::cout << RobotState.motors_pos_des[i] << ", ";
            }
            std::cout << std::endl;
            std::cout << "feet postions: " << std::endl;
            for (int i=16; i<=RobotState.motors_pos_des.size(); i++){
                std::cout << RobotState.motors_pos_des[i] << ", ";
            }
            std::cout << std::endl;
//            Eigen::VectorXd tmp=resLeg.jointPosRes+resHand.jointPosRes;
//            std::cout<<tmp.transpose()<<std::endl;
//            std::cout<<resHand.itr<<std::endl;
//            std::cout<<resHand.err.transpose()<<std::endl;

            pvtCtr.dataBusRead(RobotState);
            if (simTime<=3)
            {
                pvtCtr.calMotorsPVT(100.0/1000.0/180.0*3.1415);  // limit velocity
            }
            else
            {
                // pvtCtr.setJointPD(100,10,"Joint-ankel-l-pitch");
                // pvtCtr.setJointPD(100,10,"Joint-ankel-l-roll");
                // pvtCtr.setJointPD(100,10,"Joint-ankel-r-pitch");
                // pvtCtr.setJointPD(100,10,"Joint-ankel-r-roll");
                // pvtCtr.setJointPD(1000,100,"Joint-knee-l-pitch");
                // pvtCtr.setJointPD(1000,100,"Joint-knee-r-pitch");
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);//部署在实机上应改为传感器databus write
            
            // std::cout << "RobotState.motors_tor_out" << RobotState.motors_tor_out.size() << ": ";
            // for (int i = 0; i < RobotState.motors_tor_out.size(); ++i) {
            //     std::cout << RobotState.motors_tor_out[i] << " ";   //std::vector 中的元素访问应该使用 operator[] 或 .at() 方法
            // }
            // std::cout << std::endl;
            mj_interface.setMotorsTorque(RobotState.motors_tor_out); //实机部署应改为电机力矩输入
            // std::this_thread::sleep_for(std::chrono::seconds(100)); // 暂停 10 秒

            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
            logger.recItermData("motors_pos_des",RobotState.motors_pos_des);
            logger.recItermData("motors_tau_cur",RobotState.motors_tor_out);
            logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
            logger.recItermData("motors_vel_des",RobotState.motors_vel_des);
            logger.finishLine();

            std::cout << " ---------------- Current motors_tor_cur -------------------- " << RobotState.motors_tor_cur.size() <<std::endl;
            printf("Waist_motors_tor_cur (roll、yaw)= [%.5f, %.5f]\n", RobotState.motors_tor_cur[16], RobotState.motors_tor_cur[17]);
            printf("hip_l_motors_tor_cur (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_tor_cur[18], RobotState.motors_tor_cur[19], RobotState.motors_pos_des[20]);
            printf("hip_r_motors_tor_cur (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_tor_cur[24], RobotState.motors_tor_cur[25], RobotState.motors_pos_des[26]);
            printf("knee_l, knee_r_motors_tor_cur = [%.5f, %.5f]\n", RobotState.motors_tor_cur[21], RobotState.motors_tor_cur[27]);
            printf("ankle_l_motors_tor_cur (pitch、 roll) =[%.5f, %.5f]\n", RobotState.motors_tor_cur[22], RobotState.motors_tor_cur[23]);
            printf("ankle_r_motors_tor_cur (pitch、 roll) =[%.5f, %.5f]\n\n", RobotState.motors_tor_cur[28], RobotState.motors_tor_cur[29]);

            std::cout << " ---------------- Current motors_vel_cur -------------------- " << RobotState.motors_vel_cur.size() <<std::endl;
            printf("Waist_motors_vel_cur (roll、yaw)= [%.5f, %.5f]\n", RobotState.motors_vel_cur[16], RobotState.motors_vel_cur[17]);
            printf("hip_l_motors_vel_cur (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_vel_cur[18], RobotState.motors_vel_cur[19], RobotState.motors_pos_des[20]);
            printf("hip_r_motors_vel_cur (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_vel_cur[24], RobotState.motors_vel_cur[25], RobotState.motors_pos_des[26]);
            printf("knee_l, knee_r_motors_vel_cur = [%.5f, %.5f]\n", RobotState.motors_vel_cur[21], RobotState.motors_vel_cur[27]);
            printf("ankle_l_motors_vel_cur (pitch、 roll) =[%.5f, %.5f]\n", RobotState.motors_vel_cur[22], RobotState.motors_vel_cur[23]);
            printf("ankle_r_motors_vel_cur (pitch、 roll) =[%.5f, %.5f]\n\n", RobotState.motors_vel_cur[28], RobotState.motors_vel_cur[29]);


            std::cout << " ---------------- Desire motors_pos_des -------------------- " << RobotState.motors_pos_des.size() <<std::endl;
            printf("Waist_motors_pos_des (roll、yaw)= [%.5f, %.5f]\n", RobotState.motors_pos_des[16], RobotState.motors_pos_des[17]);
            printf("hip_l_motors_pos_des (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_pos_des[18], RobotState.motors_pos_des[19], RobotState.motors_pos_des[20]);
            printf("hip_r_motors_pos_des (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_pos_des[24], RobotState.motors_pos_des[25], RobotState.motors_pos_des[26]);
            printf("knee_l, knee_r_motors_pos_des = [%.5f, %.5f]\n", RobotState.motors_pos_des[21], RobotState.motors_pos_des[27]);
            printf("ankle_l_motors_pos_des (pitch、 roll) =[%.5f, %.5f]\n", RobotState.motors_pos_des[22], RobotState.motors_pos_des[23]);
            printf("ankle_r_motors_pos_des (pitch、 roll) =[%.5f, %.5f]\n", RobotState.motors_pos_des[28], RobotState.motors_pos_des[29]);

            std::cout << " ---------------- Current motors_pos_cur -------------------- " << RobotState.motors_pos_cur.size() <<std::endl;
            printf("Waist_motors_pos_cur (roll、yaw)= [%.5f, %.5f]\n", RobotState.motors_pos_cur[16], RobotState.motors_pos_cur[17]);
            printf("hip_l_motors_pos_cur (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_pos_cur[18], RobotState.motors_pos_cur[19], RobotState.motors_pos_des[20]);
            printf("hip_r_motors_pos_cur (roll、yaw、pitch) = [%.5f, %.5f, %.5f]\n", RobotState.motors_pos_cur[24], RobotState.motors_pos_cur[25], RobotState.motors_pos_des[26]);
            printf("knee_l, knee_r_motors_pos_cur = [%.5f, %.5f]\n", RobotState.motors_pos_cur[21], RobotState.motors_pos_cur[27]);
            printf("ankle_l_motors_pos_cur (pitch、 roll) =[%.5f, %.5f]\n", RobotState.motors_pos_cur[22], RobotState.motors_pos_cur[23]);
            printf("ankle_r_motors_pos_cur (pitch、 roll) =[%.5f, %.5f]\n\n", RobotState.motors_pos_cur[28], RobotState.motors_pos_cur[29]);
            // std::this_thread::sleep_for(std::chrono::seconds(10));
            // printf("hd_l_pos_L=[%.5f, %.5f, %.5f]\n", RobotState.hd_l_pos_W(0), RobotState.hd_l_pos_W(1), RobotState.hd_l_pos_W(2));
            // printf("hd_r_pos_L=[%.5f, %.5f, %.5f]\n", RobotState.hd_r_pos_W(0), RobotState.hd_r_pos_W(1), RobotState.hd_r_pos_W(2));
            // printf("hd_l_rot_L=[%.5f, %.5f, %.5f]\n", RobotState.hd_l_rot_W(0), RobotState.hd_l_rot_W(1), RobotState.hd_l_rot_W(2));
            // printf("hd_r_rot_L=[%.5f, %.5f, %.5f]\n", RobotState.hd_r_rot_W(0), RobotState.hd_r_rot_W(1), RobotState.hd_r_rot_W(2));
            // printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
            // printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1], RobotState.baseLinVel[2]);
        }

     

        uiController.updateScene();
    }

//    // free visualization storage
    uiController.Close();

    // free MuJoCo model and data, deactivate
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    return 0;
}