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
#include "mock_interface.h"
#include "csv_input_loader.h"
#include "debug_utiles.h"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "Hip_Knee_Kin_Dyn.hpp"
#include "Ankle_Kin_Dyn.hpp"
#include "tr5_interfaces/msg/multi_dof_command.hpp"
#include "tr5_interfaces/msg/multi_dof_state.hpp"
#include "tr5_interfaces/msg/single_dof_state.hpp"
#include "tr5_interfaces/msg/ankle_length.hpp"
#include "StateEst.h"


const   double  dt = 0.001;
const   double  dt_200Hz = 0.005;

class RobotControlNode : public rclcpp::Node {
    public:
        RobotControlNode()
        : Node("robot_control_node") {
            kinDynSolver_ = std::make_shared<Pin_KinDyn>("/home/ubantu/tr5_test/src/tr5_demo_test/models/TR5.urdf");
            model_nv = kinDynSolver_->model_nv;
            robotState_ = std::make_shared<DataBus>(model_nv);
            mk_interface_ = std::make_shared<Mock_Interface>(model_nv - 6);
            WBC_solv_ = std::make_shared<WBC_priority>(model_nv, 18, 22, 0.7, dt);
            MPC_solv_ = std::make_shared<MPC>(dt_200Hz);
            gaitScheduler_ = std::make_shared<GaitScheduler>(0.35, dt);
            footPlacement_ = std::make_shared<FootPlacement>();
            jsInterp_ = std::make_shared<JoyStickInterpreter>(dt);
            logger_ = std::make_shared<DataLogger>("/home/ubantu/tr5_test/src/tr5_demo_test/record/datalog_local.log");
            frame_ = std::make_shared<MockSensorFrame>();
            loader_ = std::make_shared<CSVInputLoader>("/home/ubantu/tr5_test/mujoco_log_new.csv", 30);
            pvtCtr_ = std::make_shared<PVT_Ctr>(dt,"/home/ubantu/tr5_test/src/tr5_demo_test/src/joint_ctrl_config.json");
            calMF_ = std::make_shared<Ankle_Kin_Dyn>();
            calMF_hip_knee_ = std::make_shared<Hip_Knee_Kin_Dyn>();
            calMF_ankle_ = std::make_shared<Ankle_Kin_Dyn>();
            StateModule_ = std::make_shared<StateEst>(0.001);
            

            stand_legLength = 1.07;
            foot_height =0.07; // distance between the foot ankel joint and the bottom
            xv_des = 1.05;
            hip_alpha_offset = 3.1415 / 2.0;
            knee_alpha_offset = 43.68 * 3.1415 / 180.0;
            for (int i = 1; i < 30; i++) {
                calMF_ankle_->pitch = i * 3.1415 /180;
                calMF_ankle_->roll = 0;
                Eigen::Vector2d force = calMF_ankle_->getForceAC(14,14);
                // std::cout << "F_AC1: " << force(0) << std::endl;
                std::cout << "F_AC2: " << force(1) << std::endl;
            }
            std::cout << "test 1111" << std::endl;
            

            for (int i = 10; i < 60; i++) {
                double rad = i * 3.1415 /180;
                double test_knee_before = calMF_hip_knee_->calKneeForce(rad, 0);
                // std::cout << "testtttt " << test_hip_before << std::endl;
            }
            
            
            robotState_->width_hips = 0.24;
            footPlacement_->kp_vx = 0.03;
            footPlacement_->kp_vy = 0.03;
            footPlacement_->kp_wz = 0.03;
            footPlacement_->stepHeight = 0.15;
            footPlacement_->legLength=stand_legLength;

            motors_pos_des.resize(model_nv - 6, 0);
            motors_pos_cur.resize(model_nv - 6, 0);
            motors_vel_des.resize(model_nv - 6, 0);
            motors_vel_cur.resize(model_nv - 6, 0);
            motors_tau_des.resize(model_nv - 6, 0);
            motors_tau_cur.resize(model_nv - 6, 0);

            fe_l_pos_L_des={-0.015, 0.13, -stand_legLength};
            fe_r_pos_L_des={-0.015, -0.13, -stand_legLength};
            fe_l_eul_L_des={-0.000, -0.00, -0.000};
            fe_r_eul_L_des={0.000, -0.00, 0.000};
            fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
            fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

            hd_l_pos_L_des={0, 0.3, -0.3};
            hd_r_pos_L_des={0, -0.3, -0.3};
            // hd_l_pos_L_des={-0.01, 0.227, -0.26};
            // hd_r_pos_L_des={-0.01, -0.227, -0.26};
            hd_l_eul_L_des={0.0, 0.0, 0.0};
            hd_r_eul_L_des={0.0, 0.0, 0.0};
            hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
            hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));
            std::cout << "test 2222" << std::endl;
            resLeg=kinDynSolver_->computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
            resHand=kinDynSolver_->computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);
            std::cout << "resLeg.jointPosRes" << resLeg.jointPosRes << std::endl;
            std::cout << "resHand.jointPosRes" << resHand.jointPosRes << std::endl;
            std::cout << "resLeg.status" << resLeg.status << std::endl;
            std::cout << "resHand.status" << resHand.status << std::endl;
            std::cout << "resLeg.itr" << resLeg.itr << std::endl;
            std::cout << "resHand.itr" << resHand.itr << std::endl;
            qIniDes = Eigen::VectorXd::Zero(model_nv + 1);
            qIniDes.block(7, 0, model_nv - 6, 1) = resLeg.jointPosRes + resHand.jointPosRes;
            WBC_solv_->setQini(qIniDes,robotState_->q);
            std::cout << "RobotState.q.size: " << robotState_->q.size() << std::endl;
           
            test_imu = 0;
            test_motor = 0;
            MPC_count = 0; // count for controlling the mpc running period
            simTime = 0;
            startSteppingTime=2;
            startWalkingTime=2.5;
            simEndTime=5;
            std::cout << "test 33333" << std::endl;
            logger_->addIterm("simTime",1);
            logger_->addIterm("motor_pos_des",model_nv - 6);
            logger_->addIterm("motor_pos_cur",model_nv - 6);
            logger_->addIterm("motor_vel_des",model_nv - 6);
            logger_->addIterm("motor_vel_cur",model_nv - 6);
            logger_->addIterm("motor_tor_des",model_nv - 6);
            logger_->addIterm("motor_tor_out",model_nv - 6);
            logger_->addIterm("wbc_tauJointRes", model_nv - 6);
            logger_->addIterm("rpyVal",3);
            logger_->addIterm("base_omega_W",3);
            logger_->addIterm("gpsVal",3);
            logger_->addIterm("base_vel",3);
            logger_->addIterm("dX_cal",12);
            logger_->addIterm("Ufe",12);
            logger_->addIterm("Xd",12);
            logger_->addIterm("X_cur",12);
            logger_->addIterm("X_cal",12);
            logger_->finishItermAdding();
            
            // timer_ = this->create_wall_timer(
            //     std::chrono::milliseconds(static_cast<int>(dt_200Hz * 2000)),
            //     std::bind(&RobotControlNode::controlLoop, this)
            // );
            pose_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/motor_pos_data_topic", 10, std::bind(&RobotControlNode::motor_pose_callback, this, std::placeholders::_1));
            vel_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/motor_vel_data_topic", 10, std::bind(&RobotControlNode::motor_vel_callback, this, std::placeholders::_1));
            tau_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tau_data_topic", 10, std::bind(&RobotControlNode::tau_callback, this, std::placeholders::_1));
            baseQuat_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/baseQuat_data_topic", 10, std::bind(&RobotControlNode::baseQuat_callback, this, std::placeholders::_1));
            // basePos_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            //     "/basePos_data_topic", 10, std::bind(&RobotControlNode::basePos_callback, this, std::placeholders::_1));
            // baseLinVel_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            //     "/baseLinVel_data_topic", 10, std::bind(&RobotControlNode::baseLinVel_callback, this, std::placeholders::_1));
            baseAcc_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/baseAcc_data_topic", 10, std::bind(&RobotControlNode::baseAcc_callback, this, std::placeholders::_1));
            baseAngVel_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/baseAngVel_data_topic", 10, std::bind(&RobotControlNode::baseAngVel_callback, this, std::placeholders::_1));
            time_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/time_data_topic", 10, std::bind(&RobotControlNode::time_callback, this, std::placeholders::_1));
            flag_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/flag_topic", 10, std::bind(&RobotControlNode::flag_callback, this, std::placeholders::_1));
            imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", 10, std::bind(&RobotControlNode::imu_callback, this, std::placeholders::_1));
            motor_subscription_ = this->create_subscription<tr5_interfaces::msg::MultiDofState>(
                "multi_dof_state", 10, std::bind(&RobotControlNode::motor_callback, this, std::placeholders::_1));
            motor_publisher_ = this->create_publisher<tr5_interfaces::msg::MultiDofCommand>(
                    "target_multi_dof_command",   // 话题名，自行根据需要修改
                    1               // QoS 队列深度
                  );
        }
    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            std::cout << "received imu" << std::endl;
        }
        void motor_callback(const tr5_interfaces::msg::MultiDofState::SharedPtr msg) {
            std::cout << "received MultiDofState" << std::endl;
        }
        void flag_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "start control loop ");
            controlLoop();
        }
        void time_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector0: ");
            time.assign(msg->data.begin(), msg->data.end());
            // for (const auto& val : time)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        }
        void motor_pose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector1: ");
            motor_pos.assign(msg->data.begin(), msg->data.end());
            // for (const auto& val : motor_pos)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        }
        void motor_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector2: ");
            motor_vel.assign(msg->data.begin(), msg->data.end());
            // for (const auto& val : motor_vel)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        }
        void tau_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector3: ");
            motor_tau.assign(msg->data.begin(), msg->data.end());
            // for (const auto& val : motor_tau)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        }
        void baseQuat_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector4: ");
            // baseQuat.assign(msg->data.begin(), msg->data.end());
            std::copy(msg->data.begin(), msg->data.begin() + 4, baseQuat);
            // Eigen::Quaterniond q(baseQuat[3], baseQuat[0], baseQuat[1], baseQuat[2]); // wxyz
            rpy[0]= atan2(2*(baseQuat[3]*baseQuat[0]+baseQuat[1]*baseQuat[2]),1-2*(baseQuat[0]*baseQuat[0]+baseQuat[1]*baseQuat[1]));
            rpy[1]= asin(2*(baseQuat[3]*baseQuat[1]-baseQuat[0]*baseQuat[2]));
            rpy[2]= atan2(2*(baseQuat[3]*baseQuat[2]+baseQuat[0]*baseQuat[1]),1-2*(baseQuat[1]*baseQuat[1]+baseQuat[2]*baseQuat[2]));
            // Eigen::Vector3d rpy_vec = q.toRotationMatrix().eulerAngles(0, 1, 2);
            // for (int i = 0; i < 3; ++i) {
            //     rpy[i] = rpy_vec[i];
            // }
            // for (const auto& val : baseQuat)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        }
        // void basePos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector5: ");
            // basePos.assign(msg->data.begin(), msg->data.end());
            // std::copy(msg->data.begin(), msg->data.begin() + 3, basePos);
            // for (const auto& val : basePos)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        // }
        // void baseLinVel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector6: ");
            // baseLinVel.assign(msg->data.begin(), msg->data.end());
            // std::copy(msg->data.begin(), msg->data.begin() + 3, baseLinVel);
            // for (const auto& val : baseLinVel)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        // }
        void baseAcc_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector7: ");
            // baseAcc.assign(msg->data.begin(), msg->data.end());
            std::copy(msg->data.begin(), msg->data.begin() + 3, baseAcc);
            // for (const auto& val : baseAcc)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        }
        void baseAngVel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received vector8: ");
            // baseAngVel.assign(msg->data.begin(), msg->data.end());
            std::copy(msg->data.begin(), msg->data.begin() + 3, baseAngVel);
            // for (const auto& val : baseAngVel)
            // {
            //     std::cout << val << " ";
            // }
            // std::cout << std::endl;
        }

        void controlLoop() {
            // std::cout << "start" << std::endl;
            // mk_interface_->dataBusWrite(robotState_);
            if (!loader_->nextFrame(frame_)) {
                std::cerr << "[Main] CSV 文件读取完毕，提前结束仿真" << std::endl;
                return;
            }
            mk_interface_->loadFromFrame(frame_);

            simTime += 0.001;
            robotState_->motors_pos_cur = motor_pos;
            robotState_->motors_vel_cur = motor_vel;

            for (int i = 0; i < 3; ++i) {
                robotState_->rpy[i] = rpy[i];
                // robotState_->basePos[i] = basePos[i];
                // robotState_->baseLinVel[i] = baseLinVel[i];
                robotState_->baseAcc[i] = baseAcc[i];
                robotState_->baseAngVel[i] = baseAngVel[i];
                robotState_->fL[i] = 0;
                robotState_->fR[i] = 0;
            }

            robotState_->setBaseQuat(baseQuat);
            robotState_->updateQ_fromRPY();
            
            if (simTime > 1 && StateModule_->flag_init)
            {
                std::cout << "init state module" << std::endl;
                StateModule_->init(robotState_);
            }
            StateModule_->set(robotState_);
            StateModule_->update();
            StateModule_->get(simTime, robotState_);
            
            kinDynSolver_->dataBusRead(robotState_);
            kinDynSolver_->computeJ_dJ();
            kinDynSolver_->computeDyn();
            kinDynSolver_->dataBusWrite(robotState_);

            StateModule_->setF(robotState_);
            StateModule_->updateF();
            StateModule_->getF(robotState_);
            

            if (simTime > startWalkingTime) {
                jsInterp_->setWzDesLPara(0, 1);
                jsInterp_->setVxDesLPara(xv_des, 2.0); // jsInterp.setVxDesLPara(0.9,1);
				robotState_->motionState = DataBus::Walk; // start walking
            } else
                jsInterp_->setIniPos(robotState_->q(0), robotState_->q(1), robotState_->base_rpy(2));
            jsInterp_->step();
            robotState_->js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
            jsInterp_->dataBusWrite(robotState_); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.

            if (fabs(simTime - 0.160) < 1e-5) {
                std::cout << "[ASSERT] simTime = " << simTime << std::endl;
                std::cout << "[ASSERT] Xd.head(12):     " << robotState_->Xd.head(12).transpose() << std::endl;
                std::cout << "[ASSERT] X_cur.head(12):  " << robotState_->X_cur.head(12).transpose() << std::endl;
                std::cout << "[ASSERT] norm(Xd - X_cur) = " 
                        << (robotState_->Xd.head(12) - robotState_->X_cur.head(12)).norm() << std::endl;
            }

            if (simTime > startSteppingTime) {
                // gait scheduler
                gaitScheduler_->dataBusRead(robotState_);
                gaitScheduler_->step();
                gaitScheduler_->dataBusWrite(robotState_);

                footPlacement_->dataBusRead(robotState_);
                footPlacement_->getSwingPos();
                footPlacement_->dataBusWrite(robotState_);
            }

            if (simTime <= startSteppingTime || robotState_->motionState == DataBus::Walk2Stand)
            {
                WBC_solv_->setQini(qIniDes, robotState_->q);
                WBC_solv_->fe_l_pos_des_W = robotState_->fe_l_pos_W;
                WBC_solv_->fe_r_pos_des_W = robotState_->fe_r_pos_W;
                WBC_solv_->fe_l_rot_des_W = robotState_->fe_l_rot_W;
                WBC_solv_->fe_r_rot_des_W = robotState_->fe_r_rot_W;
                WBC_solv_->pCoMDes = robotState_->pCoM_W;
            }

            MPC_count = MPC_count + 1;
            if (MPC_count > (dt_200Hz / dt-1)) {
                MPC_solv_->dataBusRead(robotState_);
                MPC_solv_->cal();
                MPC_solv_->dataBusWrite(robotState_);
                MPC_count = 0;
            }

            WBC_solv_->dataBusRead(robotState_);
            WBC_solv_->computeDdq(kinDynSolver_);
            WBC_solv_->computeTau();
            WBC_solv_->dataBusWrite(robotState_);

            if (simTime <= startSteppingTime) {
                robotState_->motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                robotState_->motors_vel_des = motors_vel_des;
                robotState_->motors_tor_des = motors_tau_des;
            } else {
                MPC_solv_->enable();
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
                // L_diag << 1.0, 1.0, 1.0, // eul 100 100 100
				// 		1e-3, 100.0, 1.0,   // pCoM 50 100 20
				// 		1e-3, 1e-3, 1e-3,    // w 1
				// 		1, 100.0, 1.0;   // vCoM 3
				// K_diag << 1.0, 1.0, 1.0, // fl //1e-4
				// 		1.0, 1, 1.0,         // 0.01
				// 		1.0, 1.0, 1.0,       // fr
				// 		1.0, 1, 1.0,
				// 		1.0;
                MPC_solv_->set_weight(1e-6, L_diag, K_diag);

                Eigen::VectorXd pos_des = kinDynSolver_->integrateDIY(robotState_->q, robotState_->wbc_delta_q_final);
                robotState_->motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                robotState_->motors_vel_des = eigen2std(robotState_->wbc_dq_final);
                robotState_->motors_tor_des = eigen2std(robotState_->wbc_tauJointRes);
            }
            std::cout << "temp_pos before " << robotState_->motors_pos_des[20] << std::endl;
            // joint PVT controller
            pvtCtr_->dataBusRead(robotState_);
            if (simTime <= startWalkingTime) {
                pvtCtr_->calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
            } else {
                pvtCtr_->setJointPD(100,10,"J_ankle_l_pitch");
                pvtCtr_->setJointPD(100,10,"J_ankle_l_roll");
                pvtCtr_->setJointPD(100,10,"J_ankle_r_pitch");
                pvtCtr_->setJointPD(100,10,"J_ankle_r_roll");
                pvtCtr_->setJointPD(1000,100,"J_knee_l_pitch");
                pvtCtr_->setJointPD(1000,100,"J_knee_r_pitch");

                // double kp = 1.;
                // double kd = 1.;

                // pvtCtr_->setJointPD(400 * kp, 15 * kd, "J_hip_l_roll");
                // pvtCtr_->setJointPD(200 * kp, 10 * kd, "J_hip_l_yaw");
                // pvtCtr_->setJointPD(300 * kp, 10 * kd, "J_hip_l_pitch");
                // pvtCtr_->setJointPD(300 * kp, 14 * kd, "J_knee_l_pitch");
                // pvtCtr_->setJointPD(300 * kp, 18 * kd, "J_ankle_l_pitch");
                // pvtCtr_->setJointPD(300 * kp, 16 * kd, "J_ankle_l_roll");

                // pvtCtr_->setJointPD(400 * kp, 15 * kd, "J_hip_r_roll");
                // pvtCtr_->setJointPD(200 * kp, 10 * kd, "J_hip_r_yaw");
                // pvtCtr_->setJointPD(300 * kp, 10 * kd, "J_hip_r_pitch");
                // pvtCtr_->setJointPD(300 * kp, 14 * kd, "J_knee_r_pitch");
                // pvtCtr_->setJointPD(300 * kp, 18 * kd, "J_ankle_r_pitch");
                // pvtCtr_->setJointPD(300 * kp, 16 * kd, "J_ankle_r_roll");
                pvtCtr_->calMotorsPVT();
            }
            pvtCtr_->dataBusWrite(robotState_);

            mk_interface_->logToCSV(simTime, robotState_->motors_tor_out, eigen2std(robotState_->wbc_tauJointRes), robotState_->motors_tor_des);
            std::vector<double> hip_alphas;
            hip_alphas.push_back(robotState_->motors_pos_des[20]);
            hip_alphas.push_back(robotState_->motors_pos_des[26]);
            std::vector<double> knee_alphas;
            knee_alphas.push_back(robotState_->motors_pos_des[21]);
            knee_alphas.push_back(robotState_->motors_pos_des[27]);
            std::vector<double> ankle_pitch_alphas;
            ankle_pitch_alphas.push_back(robotState_->motors_pos_des[22]);
            ankle_pitch_alphas.push_back(robotState_->motors_pos_des[28]);
            std::vector<double> ankle_roll_alphas;
            ankle_roll_alphas.push_back(robotState_->motors_pos_des[23]);
            ankle_roll_alphas.push_back(robotState_->motors_pos_des[29]);

            std::vector<double> hip_torques;
            hip_torques.push_back(robotState_->motors_tor_out[20]);
            hip_torques.push_back(robotState_->motors_tor_out[26]);
            std::vector<double> knee_torques;
            knee_torques.push_back(robotState_->motors_tor_out[21]);
            knee_torques.push_back(robotState_->motors_tor_out[27]);
            std::vector<double> ankle_pitch_torques;
            ankle_pitch_torques.push_back(robotState_->motors_tor_out[22]);
            ankle_pitch_torques.push_back(robotState_->motors_tor_out[28]);
            std::vector<double> ankle_roll_torques;
            ankle_roll_torques.push_back(robotState_->motors_tor_out[23]);
            ankle_roll_torques.push_back(robotState_->motors_tor_out[29]);

            std::vector<double> hip_forces;
            std::vector<double> knee_forces;
            std::vector<double> ankle_pitch_forces;
            std::vector<double> ankle_roll_forces;
 
            for (int i = 0 ; i < hip_alphas.size(); i++) {
                
                double temp_pos = hip_alphas[i];
                std::cout << "temp_pos " << temp_pos << std::endl;
                double temp_torque = hip_torques[i];
                double temp_force = calMF_hip_knee_->calHipForce(hip_alpha_offset + temp_pos, temp_torque);
                hip_forces.push_back(temp_force);
            }
            for (int i = 0 ; i < knee_alphas.size(); i++) {
                double temp_pos = knee_alphas[i];
                double temp_torque = knee_torques[i];
                double temp_force = calMF_hip_knee_->calKneeForce(knee_alpha_offset - temp_pos, temp_torque);
                knee_forces.push_back(temp_force);
            }

            for (int i = 0 ; i < ankle_pitch_alphas.size(); i++) {
                calMF_ankle_->pitch = ankle_pitch_alphas[i];
                calMF_ankle_->roll = ankle_roll_alphas[i];
                Eigen::Vector2d force = calMF_ankle_->getForceAC(ankle_pitch_torques[i],ankle_roll_torques[i]);
                ankle_pitch_forces.push_back(force(0));
                ankle_roll_forces.push_back(force(1));
            }

            command_.joint_name = "l_hip_pitch";
            command_.joint_torque = hip_forces[0];
            out_msg_.header.stamp = this->now();
            out_msg_.header.frame_id = "";
            out_msg_.multi_dof_command.push_back(command_);


            // calMF_->pitch = ankle_pitch_alphas[0];
            // calMF_->roll = ankle_roll_alphas[0];
            // double Mp = robotState_->motors_tor_out[22];
            // double Mr = robotState_->motors_tor_out[23];
            // Eigen::Vector2d test = calMF_->getForceAC(Mp, Mr);
            // std::cout << "motors_pos_des " << std::endl;
            // std::cout << robotState_->motors_pos_des[20] << std::endl;
            // std::cout << "motors_pos_des " << std::endl;
            // std::cout << robotState_->motors_pos_des[21] << std::endl;
            // std::cout << "motors_tor_out second " << std::endl;
            // std::cout << robotState_->motors_tor_out[20] << std::endl;
            // std::cout << "motors_tor_out second " << std::endl;
            // std::cout << robotState_->motors_tor_out[21] << std::endl;
            
            // double test_hip = calMF_hip_knee_->calHipForce(robotState_->motors_pos_des[20], robotState_->motors_tor_out[20]);
            // double test_knee = calMF_hip_knee_->calKneeForce(robotState_->motors_pos_des[21], robotState_->motors_tor_out[21]);
            // std::cout << "getForceACgetForceAC " << std::endl;
            // std::cout << test(0) << std::endl;
            // std::cout << "getForceACgetForceAC second " << std::endl;
            // std::cout << test(1) << std::endl;
            // std::cout << "test_hip " << std::endl;
            // std::cout << test_hip << std::endl;
            // std::cout << "test_knee " << std::endl;
            // std::cout << test_knee << std::endl;

            // print info to the console
//            printf("f_L=[%.3f, %.3f, %.3f]\n", RobotState.fL[0], RobotState.fL[1], RobotState.fL[2]);
//            printf("f_R=[%.3f, %.3f, %.3f]\n", RobotState.fR[0], RobotState.fR[1], RobotState.fR[2]);
//
//            printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
//            printf("basePos=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);

            // data save
            logger_->startNewLine();
            logger_->recItermData("simTime", simTime);
            logger_->recItermData("motor_pos_des", robotState_->motors_pos_des);
            logger_->recItermData("motor_pos_cur", robotState_->motors_pos_cur);
            logger_->recItermData("motor_vel_des", robotState_->motors_vel_des);
            logger_->recItermData("motor_vel_cur", robotState_->motors_vel_cur);
            logger_->recItermData("motor_tor_des", robotState_->motors_tor_des);
            logger_->recItermData("wbc_tauJointRes", robotState_->wbc_tauJointRes);
            logger_->recItermData("motor_tor_out", robotState_->motors_tor_out);
            logger_->recItermData("rpyVal", robotState_->rpy);
            logger_->recItermData("base_omega_W", robotState_->base_omega_W);
            logger_->recItermData("gpsVal", robotState_->basePos);
            logger_->recItermData("base_vel", robotState_->dq.block<3, 1>(0, 0));
			logger_->recItermData("dX_cal",robotState_->dX_cal);
			logger_->recItermData("Ufe",robotState_->Fr_ff);
			logger_->recItermData("Xd",robotState_->Xd);
			logger_->recItermData("X_cur",robotState_->X_cur);
			logger_->recItermData("X_cal",robotState_->X_cal);
			logger_->finishLine();
                                                     
/*             printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
            printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
            printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1], RobotState.baseLinVel[2]);
            printf("tau0= %.5f\n", RobotState.motors_tor_out[0]); */
            if (fabs(simTime - 0.160) < 1e-5) {
                std::cout << "\n[DEBUG] simTime = " << simTime << std::endl;
                std::cout << "[DEBUG] base_rpy = " << robotState_->base_rpy.transpose() << std::endl;
                std::cout << "[DEBUG] base_rot:\n" << robotState_->base_rot << std::endl;
                std::cout << "[DEBUG] norm(R - I) = " 
                          << (robotState_->base_rot - Eigen::Matrix3d::Identity()).norm() << std::endl;
                isRotationIdentity(robotState_->base_rot);
            }
            
            // simTime += 0.01;

        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr baseQuat_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr basePos_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr baseLinVel_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr baseAcc_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr baseAngVel_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr time_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr flag_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        rclcpp::Subscription<tr5_interfaces::msg::MultiDofState>::SharedPtr motor_subscription_;
        rclcpp::Publisher<tr5_interfaces::msg::MultiDofCommand>::SharedPtr motor_publisher_;

        std::shared_ptr<Pin_KinDyn> kinDynSolver_;
        std::shared_ptr<DataBus> robotState_;
        std::shared_ptr<Mock_Interface> mk_interface_;
        std::shared_ptr<WBC_priority> WBC_solv_;
        std::shared_ptr<MPC> MPC_solv_;
        std::shared_ptr<GaitScheduler> gaitScheduler_;
        std::shared_ptr<FootPlacement> footPlacement_;
        std::shared_ptr<JoyStickInterpreter> jsInterp_;
        std::shared_ptr<DataLogger> logger_;
        std::shared_ptr<CSVInputLoader> loader_;
        std::shared_ptr<MockSensorFrame> frame_;
        std::shared_ptr<PVT_Ctr> pvtCtr_;
        std::shared_ptr<Ankle_Kin_Dyn> calMF_;
        std::shared_ptr<Hip_Knee_Kin_Dyn> calMF_hip_knee_;
        std::shared_ptr<Ankle_Kin_Dyn> calMF_ankle_;
        std::shared_ptr<StateEst> StateModule_;
        tr5_interfaces::msg::MultiDofCommand out_msg_;
        tr5_interfaces::msg::SingleDofCommand command_;



        int model_nv;

        std::vector<double> motors_pos_des;
        std::vector<double> motors_pos_cur;
        std::vector<double> motors_vel_des;
        std::vector<double> motors_vel_cur;
        std::vector<double> motors_tau_des;
        std::vector<double> motors_tau_cur;

        double stand_legLength;//0.97;// desired baselink height
        double foot_height; // distance between the foot ankel joint and the bottom
        double xv_des;  // desired velocity in x direction

        Eigen::Vector3d fe_l_pos_L_des;
        Eigen::Vector3d fe_r_pos_L_des;
        Eigen::Vector3d fe_l_eul_L_des;
        Eigen::Vector3d fe_r_eul_L_des;
        Eigen::Matrix3d fe_l_rot_des;
        Eigen::Matrix3d fe_r_rot_des;

        Eigen::Vector3d hd_l_pos_L_des;
        Eigen::Vector3d hd_r_pos_L_des;
        Eigen::Vector3d hd_l_eul_L_des;
        Eigen::Vector3d hd_r_eul_L_des;
        Eigen::Matrix3d hd_l_rot_des;
        Eigen::Matrix3d hd_r_rot_des;

        // 关节状态
        std::vector<double> motor_pos;     // 当前关节位置
        std::vector<double> motor_vel;     // 当前关节速度
        std::vector<double> motor_pos_Old; // 上一次的位置，用于计算速度
        std::vector<double> motor_tau;     // 控制器输出的力矩（用于记录）
        std::vector<double> time;

        // 基座状态
        double baseQuat[4];        // 四元数 (x, y, z, w)
        double rpy[3];             // 欧拉角 (Roll, Pitch, Yaw)
        double basePos[3];         // 基座位置
        double baseLinVel[3];      // 基座线速度
        double baseAcc[3];         // 基座线加速度
        double baseAngVel[3];      // 基座角速度
        double test_imu;
        double test_motor;

        // 足底力传感器（模拟用）
        double fL[3];              // 左脚力
        double fR[3];              // 右脚力
        double hip_alpha_offset;
        double knee_alpha_offset;

        Pin_KinDyn::IkRes resLeg;
        Pin_KinDyn::IkRes resHand;
        Eigen::VectorXd qIniDes;
        int  MPC_count; // count for controlling the mpc running period
        double simTime;
        double startSteppingTime;
        double startWalkingTime;
        double simEndTime;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlNode>());
    rclcpp::shutdown();
    return 0;
}

