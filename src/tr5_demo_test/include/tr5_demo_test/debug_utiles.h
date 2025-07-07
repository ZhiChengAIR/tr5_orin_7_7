// debug_utils.h
#pragma once

#include <iostream>
#include "data_bus.h"

inline void printWBCState(const DataBus& state, const std::string& tag = "") {
    std::cout << "\n========== [WBC DEBUG " << tag << "] ==========\n";
    
    std::cout << "[q.head(7)]        : " << state.q.head(7).transpose() << "\n";
    std::cout << "[q.tail(30)]       : " << state.q.tail(30).transpose() << "\n";
    std::cout << "[dq.head(6)]       : " << state.dq.head(6).transpose() << "\n";

    std::cout << "[js_pos_des]       : " << state.js_pos_des.transpose() << "\n";
    std::cout << "[js_vel_des]       : " << state.js_vel_des.transpose() << "\n";
    std::cout << "[motionState]      : " << state.motionState << "\n";

    std::cout << "[dyn_M.norm]       : " << state.dyn_M.norm() << "\n";
    std::cout << "[dyn_G.norm]       : " << state.dyn_G.norm() << "\n";
    std::cout << "[dyn_C.norm]       : " << state.dyn_C.norm() << "\n";

    std::cout << "[base_rpy]         : " << state.base_rpy.transpose() << "\n";
    std::cout << "[base_rot.row(0)]  : " << state.base_rot.row(0) << "\n";
    std::cout << "[base_rot.row(1)]  : " << state.base_rot.row(1) << "\n";
    std::cout << "[base_rot.row(2)]  : " << state.base_rot.row(2) << "\n";

    std::cout << "[delta_q_final]    : " << state.wbc_delta_q_final.transpose() << "\n";
    std::cout << "[tauJointRes]      : " << state.wbc_tauJointRes.transpose() << "\n";
    
    std::cout << "=========================================\n\n";
}

inline void printWBCInternal(const DataBus& state, const std::string& tag = "") {
    std::cout << "\n========== [WBC INTERNAL " << tag << "] ==========\n";

    std::cout << "[Xd.head(12)]      : " << state.Xd.head(12).transpose() << "\n";
    std::cout << "[X_cur.head(12)]   : " << state.X_cur.head(12).transpose() << "\n";
    std::cout << "[X_cal.head(12)]   : " << state.X_cal.head(12).transpose() << "\n";
    std::cout << "[dX_cal.head(12)]  : " << state.dX_cal.head(12).transpose() << "\n";

    std::cout << "[Fr_ff.head(12)]   : " << state.Fr_ff.head(12).transpose() << "\n";
    std::cout << "[wbc_delta_q_final]: " << state.wbc_delta_q_final.transpose() << "\n";
    std::cout << "[wbc_dq_final]     : " << state.wbc_dq_final.transpose() << "\n";
    std::cout << "[wbc_ddq_final]    : " << state.wbc_ddq_final.transpose() << "\n";
    std::cout << "[wbc_tauJointRes]  : " << state.wbc_tauJointRes.transpose() << "\n";
    std::cout << "[wbc_FrRes]        : " << state.wbc_FrRes.transpose() << "\n";

    std::cout << "[dyn_M.norm]       : " << state.dyn_M.norm() << "\n";
    std::cout << "[dyn_C.norm]       : " << state.dyn_C.norm() << "\n";
    std::cout << "[dyn_G.norm]       : " << state.dyn_G.norm() << "\n";

    std::cout << "===============================================\n\n";
}

inline bool isRotationIdentity(const Eigen::Matrix3d& R, double threshold = 1e-3) {
    // 判断 base_rot 是否“接近”单位矩阵（identity rotation）
    double angle = std::acos(std::clamp((R.trace() - 1.0) / 2.0, -1.0, 1.0)); // 旋转角
    std::cout << "[DEBUG] isRotationIdentity: angle from identity = " << angle << " rad" << std::endl;
    return std::abs(angle) < threshold;
}
