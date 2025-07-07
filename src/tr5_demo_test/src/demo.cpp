/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include "wbc_priority.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include <string>
#include <iostream>
#include "debug_utiles.h"

const   double  dt = 0.001;
const   double  dt_200Hz = 0.005;
// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("/home/zjy/tr5_test/src/tr5_demo_test/models/scene.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);

int main(int argc, char **argv) {
    // initialize classes
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("/home/zjy/tr5_test/src/tr5_demo_test/models/TR5.urdf"); // kinematics and dynamics solver
    std::cout << "1111" << std::endl;
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
    MPC MPC_solv(dt_200Hz);  // mpc controller
    GaitScheduler gaitScheduler(0.35, mj_model->opt.timestep); // gait scheduler
    std::cout << "2222" << std::endl;
    PVT_Ctr pvtCtr(mj_model->opt.timestep,"../common/joint_ctrl_config.json");// PVT joint control
    FootPlacement footPlacement; // foot-placement planner
    std::cout << "3333" << std::endl;
    JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator
    std::cout << "4444" << std::endl;
    DataLogger logger("/home/zjy/tr5_test/src/tr5_demo_test/record/datalog.log"); // data logger
    std::cout << "5555" << std::endl;
    // initialize UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false); // NOTE: if the saveVideo is set to true, the raw recorded file could be 2.5 GB for 15 seconds!

    // initialize variables
    double stand_legLength = 1.08;//0.97;// desired baselink height
    double foot_height =0.07; // distance between the foot ankel joint and the bottom
    double xv_des = 1.2;  // desired velocity in x direction
	int model_nv=kinDynSolver.model_nv;

    RobotState.width_hips = 0.24;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.03;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.15;
    footPlacement.legLength=stand_legLength;

    // mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco

    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);

    // ini position and posture for foot-end and hand
    Eigen::Vector3d fe_l_pos_L_des={-0.015, 0.13, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des={-0.015, -0.13, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des={-0.000, -0.00, -0.000};
    Eigen::Vector3d fe_r_eul_L_des={0.000, -0.00, 0.000};
    Eigen::Matrix3d fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_L_des={0, 0.3, -0.3};
    Eigen::Vector3d hd_r_pos_L_des={0, -0.3, -0.3};
    Eigen::Vector3d hd_l_eul_L_des={0, 0, 0};
    Eigen::Vector3d hd_r_eul_L_des={0, 0, 0};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

    auto resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
    auto resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);
    Eigen::VectorXd qIniDes=Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1)= resLeg.jointPosRes + resHand.jointPosRes;
    WBC_solv.setQini(qIniDes,RobotState.q);
    std::cout << "RobotState.q.size: " << RobotState.q.size() << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(10));

    // register variable name for data logger
    logger.addIterm("simTime",1);
    logger.addIterm("motor_pos_des",model_nv - 6);
    logger.addIterm("motor_pos_cur",model_nv - 6);
    logger.addIterm("motor_vel_des",model_nv - 6);
    logger.addIterm("motor_vel_cur",model_nv - 6);
    logger.addIterm("motor_tor_des",model_nv - 6);
    logger.addIterm("wbc_tauJointRes", model_nv - 6);
    logger.addIterm("rpyVal",3);
    logger.addIterm("base_omega_W",3);
    logger.addIterm("gpsVal",3);
    logger.addIterm("base_vel",3);
	logger.addIterm("dX_cal",12);
	logger.addIterm("Ufe",12);
	logger.addIterm("Xd",12);
	logger.addIterm("X_cur",12);
	logger.addIterm("X_cal",12);
    logger.finishItermAdding();

    //// -------------------------- main loop --------------------------------

    int  MPC_count = 0; // count for controlling the mpc running period

    double startSteppingTime=0.5;
    double startWalkingTime=1;
    double simEndTime=2;

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    while (!glfwWindowShouldClose(uiController.window)) {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) { // press "1" to pause and resume, "2" to step the simulation
            mj_step(mj_model, mj_data);
            simTime=mj_data->time;
            printf("-------------%.3f s------------\n",simTime);
            // Read the sensors:
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            logger.recItermData("motor_pos_des", RobotState.motors_pos_des);
            logger.recItermData("motor_pos_cur", RobotState.motors_pos_cur);
            logger.recItermData("motor_vel_des", RobotState.motors_vel_des);
            logger.recItermData("motor_vel_cur", RobotState.motors_vel_cur);
            logger.recItermData("motor_tor_des", RobotState.motors_tor_des);
            // logger.recItermData("wbc_tauJointRes", RobotState.wbc_tauJointRes);
            logger.recItermData("rpyVal", RobotState.rpy);
            logger.recItermData("base_omega_W", RobotState.base_omega_W);
            logger.recItermData("gpsVal", RobotState.basePos);
            logger.recItermData("base_vel", RobotState.dq.block<3, 1>(0, 0));
			logger.recItermData("dX_cal",RobotState.dX_cal);
			logger.recItermData("Ufe",RobotState.Fr_ff);
			logger.recItermData("Xd",RobotState.Xd);
			logger.recItermData("X_cur",RobotState.X_cur);
			logger.recItermData("X_cal",RobotState.X_cal);
			// logger.finishLine();

            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            // joint number: arm-l: 0-6, arm-r: 7-13, head: 14, waist: 15-17, leg-l: 18-23, leg-r: 24-29

            if (simTime > startWalkingTime) {
                jsInterp.setWzDesLPara(0, 1);
                jsInterp.setVxDesLPara(xv_des, 2.0); // jsInterp.setVxDesLPara(0.9,1);
				RobotState.motionState = DataBus::Walk; // start walking
            } else
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            jsInterp.step();
            RobotState.js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
            jsInterp.dataBusWrite(RobotState); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.
            // === Debug 对比 Xd 与 X_cur ===
            if (fabs(simTime - 0.160) < 1e-5) {
                std::cout << "[ASSERT] simTime = " << simTime << std::endl;
                std::cout << "[ASSERT] Xd.head(12):     " << RobotState.Xd.head(12).transpose() << std::endl;
                std::cout << "[ASSERT] X_cur.head(12):  " << RobotState.X_cur.head(12).transpose() << std::endl;
                std::cout << "[ASSERT] norm(Xd - X_cur) = " 
                        << (RobotState.Xd.head(12) - RobotState.X_cur.head(12)).norm() << std::endl;
            }

            if (simTime >= startSteppingTime) {
                // gait scheduler
                std::cout << "test 1111" << std::endl;
                gaitScheduler.dataBusRead(RobotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(RobotState);
                std::cout << "test 22222" << std::endl;
                footPlacement.dataBusRead(RobotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(RobotState);
            }

            // ------------- MPC ------------
			MPC_count = MPC_count + 1;
            if (MPC_count > (dt_200Hz / dt-1)) {
                std::cout << "test 22222111" << std::endl;
                MPC_solv.dataBusRead(RobotState);
                std::cout << "test 22222bbbb" << std::endl;
                MPC_solv.cal();
                std::cout << "test 33333" << std::endl;
                MPC_solv.dataBusWrite(RobotState);
                MPC_count = 0;
            }

            // ------------- WBC ------------
            // WBC Calculation
            WBC_solv.dataBusRead(RobotState);
            WBC_solv.computeDdq(kinDynSolver);
            WBC_solv.computeTau();
            WBC_solv.dataBusWrite(RobotState);
            // get the final joint command
            if (simTime <= startSteppingTime) {
                RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
            } else {
                std::cout << "test 44444" << std::endl;
                MPC_solv.enable();
                std::cout << "test 55555" << std::endl;
                Eigen::Matrix<double, 1, nx>  L_diag;
                Eigen::Matrix<double, 1, nu>  K_diag;
                L_diag <<
                       1.0, 1.0, 1.0,//eul
                        1.0, 200.0,  1.0,//pCoM
                        1e-7, 1e-7, 1e-7,//w
                        100.0, 10.0, 1.0;//vCoM
                K_diag <<
                       1.0, 1.0, 1.0,//fl
                        1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0,//fr
                        1.0, 1.0, 1.0,1.0;
                MPC_solv.set_weight(1e-6, L_diag, K_diag);
                std::cout << "test 66666" << std::endl;

                Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
                RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
                std::cout << "test 77777" << std::endl;
            }

            // joint PVT controller
            pvtCtr.dataBusRead(RobotState);
            if (simTime <= startWalkingTime) {
                pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
            } else {
                pvtCtr.setJointPD(100,10,"J_ankle_l_pitch");
                pvtCtr.setJointPD(100,10,"J_ankle_l_roll");
                pvtCtr.setJointPD(100,10,"J_ankle_r_pitch");
                pvtCtr.setJointPD(100,10,"J_ankle_r_roll");
                pvtCtr.setJointPD(1000,100,"J_knee_l_pitch");
                pvtCtr.setJointPD(1000,100,"J_knee_r_pitch");
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            // give the joint torque command to Webots
            mj_interface.setMotorsTorque(RobotState.motors_tor_out);
            // stor the input/output data in .CSV
            mj_interface.logToCSV(mj_data->time, RobotState.motors_tor_out, eigen2std(RobotState.wbc_tauJointRes), RobotState.motors_tor_des);
            // print info to the console
//            printf("f_L=[%.3f, %.3f, %.3f]\n", RobotState.fL[0], RobotState.fL[1], RobotState.fL[2]);
//            printf("f_R=[%.3f, %.3f, %.3f]\n", RobotState.fR[0], RobotState.fR[1], RobotState.fR[2]);
//
//            printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
//            printf("basePos=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);

            // data save
            // logger.startNewLine();
            // logger.recItermData("simTime", simTime);
            // logger.recItermData("motor_pos_des", RobotState.motors_pos_des);
            // logger.recItermData("motor_pos_cur", RobotState.motors_pos_cur);
            // logger.recItermData("motor_vel_des", RobotState.motors_vel_des);
            // logger.recItermData("motor_vel_cur", RobotState.motors_vel_cur);
            // logger.recItermData("motor_tor_des", RobotState.motors_tor_des);
            logger.recItermData("wbc_tauJointRes", RobotState.wbc_tauJointRes);
            // logger.recItermData("rpyVal", RobotState.rpy);
            // logger.recItermData("base_omega_W", RobotState.base_omega_W);
            // logger.recItermData("gpsVal", RobotState.basePos);
            // logger.recItermData("base_vel", RobotState.dq.block<3, 1>(0, 0));
			// logger.recItermData("dX_cal",RobotState.dX_cal);
			// logger.recItermData("Ufe",RobotState.Fr_ff);
			// logger.recItermData("Xd",RobotState.Xd);
			// logger.recItermData("X_cur",RobotState.X_cur);
			// logger.recItermData("X_cal",RobotState.X_cal);
			logger.finishLine();
                                                     
/*             printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
            printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
            printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1], RobotState.baseLinVel[2]);
            printf("tau0= %.5f\n", RobotState.motors_tor_out[0]); */

        }

        if (mj_data->time>=simEndTime)
            break;

        uiController.updateScene();
    };
    // free visualization storage
    uiController.Close();

    return 0;
}
