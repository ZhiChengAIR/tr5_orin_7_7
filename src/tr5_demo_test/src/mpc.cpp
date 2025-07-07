/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://gitee.com/panda_23/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "mpc.h"
#include "useful_math.h"


MPC::MPC(double dtIn):QP(nu*ch, nc*ch) {
    m = 75;     // m = 77.35;
    g = -9.8;
    miu = 0.5;
    delta_foot[0] = 0.073;
    delta_foot[1] = 0.125;
    delta_foot[2] = 0.025;
    delta_foot[3] = 0.025;

    max[0] = 300.0;  max[1] = 300.0; max[2] = -1.1*m*g;
    max[3] = 20.0;  max[4] = 80.0; max[5] = 100.0;

    min[0] = -300.0;  min[1] = -300.0; min[2] = 0.0;
    min[3] = -20.0;  min[4] = -80.0; min[5] = -100.0;

    //single rigid body model
    for (int i = 0; i < (mpc_N); i ++){
        Ac[i].setZero();
        Bc[i].setZero();
        A[i].setZero();
        B[i].setZero();
    }
    Cc.setZero();
    C.setZero();

    Aqp.setZero();
    Aqp1.setZero();
    Bqp1.setZero();
    Bqp.setZero();
    Cqp1.setZero();
    Cqp.setZero();

    Ufe.setZero();
    Ufe_pre.setZero();

    Xd.setZero();
    X_cur.setZero();
    X_cal.setZero();
    dX_cal.setZero();

    L = Eigen::MatrixXd::Zero(nx*mpc_N, nx*mpc_N);
    K.setZero(); M.setZero();
    alpha = 0.0;
    H.setZero();
    c.setZero();

    u_low.setZero(); u_up.setZero();
    As.setZero();
    bs.setZero();

    pCoM.setZero();
    pf2com.setZero(); pe.setZero();
    R_cur.setZero();
    R_w2f.setZero(); R_f2w.setZero();

    qpOASES::Options  option;
    option.printLevel = qpOASES::PL_LOW;
    QP.setOptions(option);

    dt = dtIn;
}

/***
配置状态误差权重矩阵L和控制输入权重矩阵K并通过旋转矩阵 R_curz 对权重进行动态坐标系变换。
u_weight（double 类型）
标量权重系数，通常用于调节控制输入项的全局惩罚强度（例如整体缩放输入权重）。
赋值给 alpha，可能用于后续优化问题中的正则化项或权重组合。

L_diag（Eigen::MatrixXd 类型）
状态跟踪误差权重矩阵 L 的对角线元素，维度应为 1 × nx（nx 为状态维度）。
表示每个状态分量的独立权重（如位置、速度、角速度、姿态的误差权重）。

K_diag（Eigen::MatrixXd 类型）
控制输入权重矩阵 K 的对角线元素，维度应为 1 × nu（nu 为控制输入维度）。
表示每个输入分量的独立权重（如推力、力矩的惩罚强度）。
***/

void MPC::set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag) {
    Eigen::MatrixXd   L_diag_N = Eigen::MatrixXd::Zero(1, nx*mpc_N); //将单步权重L_diag扩展至整个预测时域和控制时域
    Eigen::MatrixXd   K_diag_N = Eigen::MatrixXd::Zero(1, nu*ch); //将单步权重K_diag扩展至整个预测时域和控制时域

    L = Eigen::MatrixXd::Zero(nx*mpc_N, nx*mpc_N); //最终将构造为块对角矩阵，每个块对应不同时间步的权重。
    K = Eigen::MatrixXd::Zero(nu*ch, nu*ch); //最终将构造为块对角矩阵，每个块对应不同时间步的权重。

    alpha = u_weight;
    for (int i = 0; i < mpc_N; i++) {
        L_diag_N.block<1,nx>(0, i*nx) = L_diag;
    } //将 L_diag 沿预测时域重复 mpc_N 次，形成 L_diag_N

    for (int i = 0; i < ch; i++) {
        K_diag_N.block<1,nu>(0, i*nu) = K_diag;
    } //将 K_diag 沿预测时域重复 mpc_N 次，形成 K_diag_N

    for (int i = 0; i < nx*mpc_N; i++) {
        L(i,i) = L_diag_N(0,i);
    } //将L_diag_N填充到L的对角线上

    for (int i = 0; i < nu*ch; i++) {
        K(i,i) = K_diag_N(0,i);
    } //将K_diag_N填充到K的对角线上

	for (int i = 0; i < mpc_N; i++){
		L.block<3,3>(i*nx + 3,i*nx + 3) = R_curz[i]*L.block<3,3>(i*nx + 3,i*nx + 3)*R_curz[i].transpose();
		L.block<3,3>(i*nx + 6,i*nx + 6) = R_curz[i]*L.block<3,3>(i*nx + 6,i*nx + 6)*R_curz[i].transpose();
		L.block<3,3>(i*nx + 9,i*nx + 9) = R_curz[i]*L.block<3,3>(i*nx + 9,i*nx + 9)*R_curz[i].transpose();
	} //位姿变换，将权重矩阵L的局部坐标系转换到全局坐标系，确保权重在不同姿态下的一致性。

    for (int i = 0; i < ch; i++){
        K.block<3,3>(i*nu,i*nu) = R_curz[i]*K.block<3,3>(i*nu,i*nu)*R_curz[i].transpose();
        K.block<3,3>(i*nu + 3,i*nu + 3) = R_curz[i]*K.block<3,3>(i*nu + 3,i*nu + 3)*R_curz[i].transpose();
        K.block<3,3>(i*nu + 6,i*nu + 6) = R_curz[i]*K.block<3,3>(i*nu + 6,i*nu + 6)*R_curz[i].transpose();
        K.block<3,3>(i*nu + 9,i*nu + 9) = R_curz[i]*K.block<3,3>(i*nu + 9,i*nu + 9)*R_curz[i].transpose();
    } //位姿变换，将权重矩阵K的局部坐标系转换到全局坐标系，确保权重在不同姿态下的一致性。
}


void MPC::dataBusRead(const std::shared_ptr<DataBus>& Data) {
    // Set value
    X_cur.block<3, 1>(0, 0) = Data->base_rpy; // 将基座的欧拉角存储在当前状态中
    X_cur.block<3, 1>(3, 0) = Data->q.block<3, 1>(0, 0); // 将关节角存储在当前状态中
    X_cur.block<3, 1>(6, 0) = Data->dq.block<3, 1>(3, 0); // 将关节角速度存储在当前状态中
    X_cur.block<3, 1>(9, 0) = Data->dq.block<3, 1>(0, 0); // 将关节速度存储在当前状态中
    
    if (EN) {
        // Set Xd, Xd 是目标状态向量，存储 MPC 预测控制的目标状态
        for (int i = 0; i < (mpc_N - 1); i++)
            Xd.block<nx, 1>(nx * i, 0) = Xd.block<nx, 1>(nx * (i + 1), 0); // 将之前的预测目标状态向前移动一个时间步
        for (int j = 0; j < 3; j++)
            Xd(nx * (mpc_N - 1) + j) = Data->js_eul_des(j); // 将最后一个目标状态的位姿设为 js_eul_des(j)
        for (int j = 0; j < 3; j++)
            Xd(nx * (mpc_N - 1) + 3 + j) = Data->js_pos_des(j); // 将最后一个目标状态的位置设为 js_pos_des(j)
        for (int j = 0; j < 3; j++)
            Xd(nx * (mpc_N - 1) + 6 + j) = Data->js_omega_des(j); // 将最后一个目标状态的角速度设为 js_omega_des(j)
        for (int j = 0; j < 3; j++)
            Xd(nx * (mpc_N - 1) + 9 + j) = Data->js_vel_des(j); // 将最后一个目标状态的速度设为 js_vel_des(j)
    }
    else {
        for (int i = 0; i < mpc_N; i++) { // 如果 EN 为假，则目标状态不移动，将所有目标状态设为当前状态 X_cur
            for (int j = 0; j < 3; j++)
                Xd(nx * i + j) = X_cur(j); // Data.js_eul_des(j);
            for (int j = 0; j < 3; j++)
                Xd(nx * i + 3 + j) = X_cur(3 + j); // Data.js_pos_des(j);
            for (int j = 0; j < 3; j++)
                Xd(nx * i + 6 + j) = X_cur(6 + j); // Data.js_omega_des(j);
            for (int j = 0; j < 3; j++)
                Xd(nx * i + 9 + j) = X_cur(9 + j); // Data.js_vel_des(j);
        }
    }

    R_cur = eul2Rot(X_cur(0), X_cur(1), X_cur(2)); // Data.base_rot; R_cur 是当前基座的旋转矩阵，通过欧拉角计算得到
    for (int i = 0; i < mpc_N; i++) {
        R_curz[i] = Rz3(X_cur(2)); // R_curz 是一个数组，存储每个时间步的 z 轴旋转矩阵
    }
    pCoM = X_cur.block<3, 1>(3, 0); // 更新质心位置
    pe.block<3, 1>(0, 0) = Data->fe_l_pos_W; // 更新左脚位置
    pe.block<3, 1>(3, 0) = Data->fe_r_pos_W; // 更新右脚位置

    pf2com.block<3, 1>(0, 0) = pe.block<3, 1>(0, 0) - pCoM; // 计算当前左脚到质心的向量
    pf2com.block<3, 1>(3, 0) = pe.block<3, 1>(3, 0) - pCoM; // 计算当前右脚到质心的向量
    pf2comd.block<3, 1>(0, 0) = pe.block<3, 1>(0, 0) - Xd.block<3, 1>(3, 0); // 计算左脚到目标位置的向量
    pf2comd.block<3, 1>(3, 0) = pe.block<3, 1>(3, 0) - Xd.block<3, 1>(3, 0); // 计算右脚到目标位置的向量
    //std::cout << "in mpc aaaa " << pf2com.block<3, 1>(0, 0) << std::endl;
    //std::cout << "in mpc bbbb " << pf2com.block<3, 1>(3, 0) << std::endl;
    // Ic = Data.inertia;
    Ic << 12.61, 0, 0.37,
          0, 11.15, 0.01,
          0.37, 0.01, 2.15;

    legStateCur = Data->legState; // 当前腿部状态
    legStateNext = Data->legStateNext; // 下一个腿部状态
    for (int i = 0; i < mpc_N; i++) {
        double aa;
        aa = i * dt / 0.4;
        double phip;
        phip = Data->phi + aa;
        if (phip > 1)
            legState[i] = legStateNext;
        else
            legState[i] = legStateCur;
    }

    Eigen::Matrix<double, 3, 3> R_slop;
    R_slop = eul2Rot(Data->slop(0), Data->slop(1), Data->slop(2));
    if (legStateCur == DataBus::RSt)
        R_f2w = Data->fe_r_rot_W;
    else if (legStateCur == DataBus::LSt)
        R_f2w = Data->fe_l_rot_W;
    else
        R_f2w = R_slop;
    R_w2f = R_f2w.transpose();
}


void MPC::cal() {
    if (EN) {
        //qp pre
        // std::cout << "ininini 11111" << std::endl;
		for (int i = 0; i < mpc_N; i++) { //将A矩阵离散化
			Ac[i].block<3, 3>(0, 6) = R_curz[i].transpose();
			Ac[i].block<3, 3>(3, 9) = Eigen::MatrixXd::Identity(3,3);
			A[i] = Eigen::MatrixXd::Identity(nx,nx) + dt * Ac[i];
		}
		for (int i = 0; i < mpc_N; i++) { //将B矩阵离散化
			pf2comi[i] = pf2com;
			Eigen::Matrix3d Ic_W_inv;
			Ic_W_inv = (R_curz[i] * Ic * R_curz[i].transpose()).inverse();
			Bc[i].block<3, 3>(6, 0) = Ic_W_inv * CrossProduct_A(pf2comi[i].block<3, 1>(0, 0));
			Bc[i].block<3, 3>(6, 3) = Ic_W_inv;
			Bc[i].block<3, 3>(6, 6) = Ic_W_inv * CrossProduct_A(pf2comi[i].block<3, 1>(3, 0));
			Bc[i].block<3, 3>(6, 9) = Ic_W_inv;
			Bc[i].block<3, 3>(9, 0) = Eigen::MatrixXd::Identity(3,3)/ m;
			Bc[i].block<3, 3>(9, 6) = Eigen::MatrixXd::Identity(3,3)/ m;
			Bc[i]((nx - 1), (nu - 1)) = 1.0 / m;
			B[i] = dt * Bc[i];
		}
        // std::cout << "ininini 22222" << std::endl;
		for (int i = 0; i < mpc_N; i++) 
			Aqp.block<nx, nx>(i * nx, 0) = Eigen::MatrixXd::Identity(nx,nx);
		for (int i = 0; i < mpc_N; i++)
			for (int j = 0; j < i + 1; j++)
				Aqp.block<nx, nx>(i * nx, 0) = A[j] * Aqp.block<nx, nx>(i * nx, 0);
        // std::cout << "ininini 33333" << std::endl;
		for (int i = 0; i < mpc_N; i++)
			for (int j = 0; j < i + 1; j++)
				Aqp1.block<nx, nx>(i * nx, j * nx) = Eigen::MatrixXd::Identity(nx,nx);
        for (int i = 1; i < mpc_N; i++)
            for (int j = 0; j < i; j++)
                for (int k = j + 1; k < (i + 1); k++)
                    Aqp1.block<nx, nx>(i * nx, j * nx) = A[k] * Aqp1.block<nx, nx>(i * nx, j * nx); //描述系统在多个时间步内的状态转移
        // std::cout << "ininini 44444" << std::endl;
        for (int i = 0; i < mpc_N; i++)
            Bqp1.block<nx, nu>(i * nx, i * nu) = B[i]; 
        Eigen::MatrixXd Bqp11 = Eigen::MatrixXd::Zero(nu * mpc_N, nu * ch);
        Bqp11.setZero();
        Bqp11.block<nu * ch, nu * ch>(0, 0) = Eigen::MatrixXd::Identity(nu * ch, nu * ch);
        for (int i = 0; i < (mpc_N - ch); i++)
            Bqp11.block<nu, nu>(nu * ch + i * nu, nu * (ch - 1)) = Eigen::MatrixXd::Identity(nu, nu);
        // std::cout << "ininini 555555" << std::endl;
        Eigen::MatrixXd B_tmp = Eigen::MatrixXd::Zero(nx * mpc_N, nu * ch);
        B_tmp = Bqp1 * Bqp11;
        Bqp = Aqp1 * B_tmp; //描述控制输入在多个时间步内的累积影响

		Eigen::Matrix<double, nu*ch, 1>		delta_U;
		delta_U.setZero();
		for (int i = 0; i < ch; i++){
			if (legState[i] == DataBus::LSt)
				delta_U(nu*i + 2) = m*g;
			else if (legState[i] == DataBus::RSt)
				delta_U(nu*i + 8) = m*g;
			else{
				delta_U(nu*i + 2) = 0.5*m*g;
				delta_U(nu*i + 8) = 0.5*m*g;
			}
		}

        H = 2 * (Bqp.transpose() * L * Bqp + alpha * K) + 1e-10*Eigen::MatrixXd::Identity(nx*mpc_N, nx*mpc_N);
		c = 2 * Bqp.transpose() * L * (Aqp * X_cur - Xd) + 2 * alpha * K * delta_U; //目标函数的线性项

        
        //friction constraint
        Eigen::Matrix<double, ncfr_single, 3> Asfr111, Asfr11;
        Eigen::Matrix<double, ncfr, nu> Asfr1;
        Eigen::Matrix<double, ncfr * ch, nu * ch> Asfr;
        Asfr111.setZero();
        Asfr1.setZero();
        Asfr.setZero();
        Asfr111 <<
                -1.0, 0.0, -1.0 / sqrt(2.0) * miu,
                1.0, 0.0, -1.0 / sqrt(2.0) * miu,
                0.0, -1.0, -1.0 / sqrt(2.0) * miu,
                0.0, 1.0, -1.0 / sqrt(2.0) * miu;
        Asfr11 = Asfr111 * R_w2f; //摩擦力约束
        Asfr1.block<ncfr_single, 3>(0, 0) = Asfr11;
        Asfr1.block<ncfr_single, 3>(ncfr_single, 6) = Asfr11;

        
        for (int i = 0; i < ch; i++)
            Asfr.block<ncfr, nu>(ncfr * i, i * nu) = Asfr1;

        //moment constraint x y
        double sign_xy[4]{1.0, -1.0, -1.0, 1.0};
        Eigen::Matrix<double, 3, 1> gxyz[4];
        gxyz[0] << 0.0, 1.0, 0.0;
        gxyz[1] << 0.0, 1.0, 0.0;
        gxyz[2] << 1.0, 0.0, 0.0;
        gxyz[3] << 1.0, 0.0, 0.0;
        Eigen::Matrix<double, 3, 1> r[4];
        Eigen::Matrix<double, 3, 1> p[4];
        Eigen::Matrix<double, ncstxya, 6> Astxy_r[4];
        Eigen::Matrix<double, ncstxy_single, 6> Astxy11;
        Eigen::Matrix<double, ncstxy, nu> Astxy1;
        Eigen::Matrix<double, ncstxy * ch, nu * ch> Astxy;
        Astxy_r[0].setZero();
        Astxy_r[1].setZero();
        Astxy_r[2].setZero();
        Astxy_r[3].setZero();
        Astxy11.setZero();
        Astxy1.setZero();
        Astxy.setZero();

        // std::cout << "ininini 88888" << std::endl;
        r[0] << 0.0, 1.0, 0.0;
        r[1] << 0.0, 1.0, 0.0;
        r[2] << 1.0, 0.0, 0.0;
        r[3] << 1.0, 0.0, 0.0;

        p[0] << delta_foot[0], 0.0, 0.0;
        p[1] << -delta_foot[1], 0.0, 0.0;
        p[2] << 0.0, delta_foot[2], 0.0;
        p[3] << 0.0, -delta_foot[3], 0.0;

        // std::cout << "ininini 99999" << std::endl;
        for (int i = 0; i < 4; i++) {
            Astxy_r[i].block<1, 3>(0, 0) =
                    sign_xy[i] * gxyz[i].transpose() * R_w2f * R_f2w * r[i] * (R_f2w * r[i]).transpose() *
                    CrossProduct_A(R_f2w * p[i]);
            Astxy_r[i].block<1, 3>(0, 3) = sign_xy[i] * gxyz[i].transpose() * R_w2f;
            Astxy11.block<ncstxya, 6>(i * ncstxya, 0) = Astxy_r[i];
        }
        Astxy1.block<ncstxy_single, 6>(0, 0) = Astxy11;
        Astxy1.block<ncstxy_single, 6>(ncstxy_single, 6) = Astxy11;
        for (int i = 0; i < ch; i++)
            Astxy.block<ncstxy, nu>(ncstxy * i, nu * i) = Astxy1;

        //moment constraint z
        Eigen::Matrix<double, ncstza, 6> Astz_r[4];
        Eigen::Matrix<double, ncstz_single, 6> Astz11;
        Eigen::Matrix<double, ncstz, nu> Astz1;
        Eigen::Matrix<double, ncstz * ch, nu * ch> Astz;
        Astz_r[0].setZero();
        Astz_r[1].setZero();
        Astz_r[2].setZero();
        Astz_r[3].setZero();
        Astz11.setZero();
        Astz1.setZero();
        Astz.setZero();

        // std::cout << "ininini 1010101010" << std::endl;
        for (int i = 0; i < 4; i++) {
            Astz_r[i].block<1, 3>(0, 0) =  -sqrt(p[i](0) * p[i](0) + p[i](1) * p[i](1) + p[i](2) * p[i](2)) * miu *
                                           Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0) * R_w2f;
            // std::cout << "Astz_r" << std::endl;
            // std::cout << "Astz_r" << std::endl;
            Astz_r[i].block<1, 3>(0, 3) = Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0) * R_w2f;
            Astz_r[i].block<1, 3>(1, 0) = Astz_r[i].block<1, 3>(0, 0);
            Astz_r[i].block<1, 3>(1, 3) = -1*Astz_r[i].block<1, 3>(0, 3);
            // std::cout << Astz_r << std::endl;
            Astz11.block<ncstza, 6>(i * ncstza, 0) = Astz_r[i];
        }
        Astz1.block<ncstz_single, 6>(0, 0) = Astz11;
        Astz1.block<ncstz_single, 6>(ncstz_single, 6) = Astz11;
        for (int i = 0; i < ch; i++)
            Astz.block<ncstz, nu>(ncstz * i, nu * i) = Astz1;

        As.block<ncfr * ch, nu * ch>(0, 0) = Asfr; //摩擦力约束
        As.block<ncstxy * ch, nu * ch>(ncfr * ch, 0) = Astxy; //xy方向力矩约束
        As.block<ncstz * ch, nu * ch>(ncfr * ch + ncstxy * ch, 0) = Astz; //z方向力矩约束

        bs.setZero();

        // std::cout << "ininini aaaaa" << std::endl;
        //qp
        Eigen::Matrix<double, nu * ch, 1> Guess_value;
        Guess_value.setZero();
        for (int i = 0; i < ch; i++){
            if (legState[i] == DataBus::DSt) {
                Guess_value(i * nu + 2) = -0.5 * m * g;
                Guess_value(i * nu + 8) = -0.5 * m * g;
                Guess_value(i * nu + 12) = m * g;
                for (int j = 0; j < 6; j++) {
                    u_low(i * nu + j) = min[j];
                    u_low(i * nu + j + 6) = min[j];
                    u_up(i * nu + j) = max[j];
                    u_up(i * nu + j + 6) = max[j];
                }
                u_low(i * nu + 12) = m * g;
                u_up(i * nu + 12) = m * g;
            } else if (legState[i] == DataBus::LSt) {
                Guess_value(i * nu + 2) = -m * g;
				Guess_value(i * nu + 8) = 0.0;
                Guess_value(i * nu + 12) = m * g;
                for (int j = 0; j < 6; j++) {
                    u_low(i * nu + j) = min[j];
                    u_low(i * nu + j + nu / 2) = 0.0;
                    u_up(i * nu + j) = max[j];
                    u_up(i * nu + j + nu / 2) = 0.0;
                }

                u_low(i * nu + 12) = m * g;
                u_up(i * nu + 12) = m * g;
            } else if (legState[i] == DataBus::RSt) {
				Guess_value(i * nu + 2) = 0.0;
                Guess_value(i * nu + 8) = -m * g;
                Guess_value(i * nu + 12) = m * g;
                for (int j = 0; j < 6; j++) {
                    u_low(i * nu + j) = 0.0;
                    u_low(i * nu + j + nu / 2) = min[j];
                    u_up(i * nu + j) = 0.0;
                    u_up(i * nu + j + nu / 2) = max[j];
                }
                u_low(i * nu + 12) = m * g;
                u_up(i * nu + 12) = m * g;
            }
        }

        // std::cout << "ininini bbbbbbbb" << std::endl;
        qpOASES::returnValue res;
        nWSR = 1000000;
        cpu_time = dt;

        Eigen::Matrix<double, nc * ch, 1> lbA, ubA, one_ch_1;
        one_ch_1.setOnes();
        lbA = -1e7 * one_ch_1;
        ubA = 1e7 * one_ch_1;

        for (int i = 0; i < ch; i++){
            if(legState[i] == DataBus::DSt){
                ubA.block<ncfr, 1>(ncfr * i, 0).setZero();
                ubA.block<ncstxy, 1>(ncfr * ch + ncstxy * i, 0).setZero();
                ubA.block<ncstz, 1>(ncfr * ch + ncstxy * ch + ncstz * i, 0).setZero();
            }
            else if (legState[i] == DataBus::LSt) {
                ubA.block<ncfr_single, 1>(ncfr * i, 0).setZero();
                ubA.block<ncstxy_single, 1>(ncfr * ch + ncstxy * i, 0).setZero();
                ubA.block<ncstz_single, 1>(ncfr * ch + ncstxy * ch + ncstz * i, 0).setZero();
            }
            else if (legState[i] == DataBus::RSt) {
                ubA.block<ncfr_single, 1>(ncfr * i + ncfr_single, 0).setZero();
                ubA.block<ncstxy_single, 1>(ncfr * ch + ncstxy * i + ncstxy_single, 0).setZero();
                ubA.block<ncstz_single, 1>(ncfr * ch + ncstxy * ch + ncstz * i + ncstz_single, 0).setZero();
            }
        }

        copy_Eigen_to_real_t(qp_H, H, nu * ch, nu * ch);
        std::cout << "nu * ch " << nu * ch << std::endl;
        copy_Eigen_to_real_t(qp_c, c, nu * ch, 1);
        copy_Eigen_to_real_t(qp_As, As, nc * ch, nu * ch);
        copy_Eigen_to_real_t(qp_lbA, lbA, nc * ch, 1);
        copy_Eigen_to_real_t(qp_ubA, ubA, nc * ch, 1);
        copy_Eigen_to_real_t(qp_lu, u_low, nu * ch, 1);
        copy_Eigen_to_real_t(qp_uu, u_up, nu * ch, 1);
        copy_Eigen_to_real_t(xOpt_iniGuess, Guess_value, nu * ch, 1);
        res = QP.init(qp_H, qp_c, qp_As, qp_lu, qp_uu, qp_lbA, qp_ubA, nWSR, &cpu_time, xOpt_iniGuess);

        qp_Status = qpOASES::getSimpleStatus(res);
        qp_nWSR = nWSR;
        qp_cpuTime = cpu_time;

		if (res!=qpOASES::SUCCESSFUL_RETURN)
		{
			printf("failed!!!!!!!!!!!!!\n");
		}

        qpOASES::real_t xOpt[nu * ch];
        QP.getPrimalSolution(xOpt);
        if (qp_Status == 0) {
            for (int i = 0; i < nu * ch; i++)
                Ufe(i) = xOpt[i];
        }

        dX_cal = Ac[0] * X_cur + Bc[0] * Ufe.block<nu,1>(0,0); //计算状态变化量
        Eigen::Matrix<double, nx, 1>    delta_X;
        delta_X.setZero();
        for (int i = 0; i < 3; i++){
            delta_X(i) = 0.5*dX_cal(i+6)*dt*dt;
            delta_X(i+3) = 0.5*dX_cal(i+9)*dt*dt;
            delta_X(i+6) = dX_cal(i+6)*dt;
            delta_X(i+9) = dX_cal(i+9)*dt;
        }

        X_cal = (Aqp * X_cur + Bqp * Ufe).block<nx,1>(nx*0,0) + delta_X; //更新系统状态

        Ufe_pre = Ufe.block<nu, 1>(0, 0); //保存当前的控制输入
        QP.reset(); //充值QP求解器
    }
}

void MPC::dataBusWrite(std::shared_ptr<DataBus>& Data) {
    Data->Xd = Xd;
    Data->X_cur = X_cur;
    Data->fe_react_tau_cmd = Ufe;
    Data->X_cal = X_cal;
    Data->dX_cal = dX_cal;

    Data->qp_nWSR_MPC = nWSR;
    Data->qp_cpuTime_MPC = cpu_time;
    Data->qpStatus_MPC = qp_Status;

    Data->Fr_ff = Ufe.block<12, 1>(0, 0);

    double k = 5;
    Data->des_ddq.block<2, 1>(0, 0) << dX_cal(9), dX_cal(10);

    Data->des_ddq(5) = k * (Xd(6 + 2) - Data->dq(5));

    Data->des_dq.block<3, 1>(0, 0) << Xd(9 + 0), Xd(9 + 1), Xd(9 + 2);
    Data->des_dq.block<2, 1>(3, 0) << 0.0, 0.0;
    Data->des_dq(5) = Xd(6 + 2);

    Data->des_delta_q.block<2, 1>(0, 0) = Data->des_dq.block<2, 1>(0, 0) * dt;
    Data->des_delta_q(5) = Data->des_dq(5) * dt;

    Data->base_rpy_des << 0.005, 0.00, Xd(2);
    Data->base_pos_des << Xd(3 + 0), Xd(3 + 1), Xd(3 + 2);
}


void    MPC::enable(){
    EN = true;
}
void    MPC::disable(){
    EN = false;
}

bool    MPC::get_ENA(){
    return EN;
}

void MPC::copy_Eigen_to_real_t(qpOASES::real_t* target, Eigen::MatrixXd source, int nRows, int nCols) {
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows)
    // real_t is stored by rows, same to C array
    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count] = source(i, j);
            count++;
        }
    }
}

/***

为了提高MPC控制的精确性，可以重点调整以下参数：
1.预测范围（mpc_N）：增大预测范围。
2.采样时间（dt）：减小采样时间。
3.权重矩阵（L 和 K）：调整权重矩阵的值。
4.摩擦力和力矩约束参数（miu 等）：调整摩擦系数和约束参数。
5.控制输入的上下限（u_low 和 u_up）：调整控制输入的范围。
6.QP求解器的参数（nWSR 和初始猜测值）：增大迭代次数，调整初始猜测值。
7.系统矩阵和控制矩阵的精度：确保模型更接近实际系统。
8.目标状态的平滑性：避免目标状态变化过快。

***/

