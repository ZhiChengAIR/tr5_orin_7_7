#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <ceres/ceres.h>

//---------------------------------------------------------------------
// Ankle_Kin_Dyn Class
//---------------------------------------------------------------------
class Ankle_Kin_Dyn {
public:
    // Public members for setting rotation angles (in degrees)
    double pitch{0.0}; 
    double roll{0.0};   

    Ankle_Kin_Dyn();
    Eigen::Vector2d getDistanceAC();
    Eigen::Vector2d getForceAC(double Mpitch, double Mroll);
    Eigen::Vector2d getTorquePR(double F_AC1, double F_AC2);

private:
    // Transformation parameters
    double Z_BD{0.0};
    double X_DC{0.0};
    double Y_DC1{0.0};
    double Y_DC2{0.0};
    double Z_DC{0.0};

    // Points (homogeneous coordinates)
    Eigen::Vector4d PointA1 = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointA2 = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointB  = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointC1 = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointC2 = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointD  = Eigen::Vector4d::Zero();
    
    // Transformation matrices
    Eigen::Matrix4d transformMatrix_B_to_C1 = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d transformMatrix_B_to_C2 = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d transformMatrix_B_to_D = Eigen::Matrix4d::Zero();

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Ones(); 
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Ones();
    Eigen::Matrix4d T3 = Eigen::Matrix4d::Ones();
    Eigen::Matrix4d T4 = Eigen::Matrix4d::Ones(); 
    Eigen::Matrix4d T5 = Eigen::Matrix4d::Ones(); 

    double Rad2Degree(double rad);
    double Degree2Rad(double degree);
    void calTransMatrices();
};

inline Ankle_Kin_Dyn::Ankle_Kin_Dyn() {
    // Initialize fixed points
    PointA1 << 25.84, 35, 315.83, 1;
    PointA2 << 25.84, -35, 315.83, 1;
    PointB  << 0, 0, 0, 1;

    // Initialize transformation parameters
    Z_BD  = -33.5;
    X_DC  = -35;
    Y_DC1 = 22;
    Y_DC2 = -22;
    Z_DC  = 33.5;
}

inline Eigen::Vector2d Ankle_Kin_Dyn::getDistanceAC() {
    Eigen::Vector2d Len;
    calTransMatrices();

    // Transform the base point into two different coordinate systems
    PointC1 = transformMatrix_B_to_C1 * PointB;
    PointC2 = transformMatrix_B_to_C2 * PointB;

    // Calculate distances from A1/A2 to C1/C2
    double len_AC1 = (PointA1 - PointC1).norm();
    double len_AC2 = (PointA2 - PointC2).norm();

    Len << len_AC1, len_AC2;
    return Len;
}

inline Eigen::Vector2d Ankle_Kin_Dyn::getForceAC(double Mpitch, double Mroll) {

    calTransMatrices();

    PointC1 = transformMatrix_B_to_C1 * PointB;
    PointC2 = transformMatrix_B_to_C2 * PointB;
    PointD = transformMatrix_B_to_D * PointB;

    Eigen::Vector3d u1 = ((PointC1 - PointA1).block<3, 1>(0, 0)).normalized();
    Eigen::Vector3d u2 = ((PointC2 - PointA2).block<3, 1>(0, 0)).normalized();

    double m1_pitch = ((PointC1 - PointB).block<3, 1>(0, 0)).cross(u1)(1);
    double m2_pitch = ((PointC2 - PointB).block<3, 1>(0, 0)).cross(u2)(1);
    double m1_roll = ((PointC1 - PointD).block<3, 1>(0, 0)).cross(u1).dot(T1.block<3, 3>(0, 0) * Eigen::Vector3d::UnitX());
    double m2_roll = ((PointC2 - PointD).block<3, 1>(0, 0)).cross(u2).dot(T1.block<3, 3>(0, 0) * Eigen::Vector3d::UnitX());
    // std::cout << "m1_pitch " << m1_pitch << std::endl;
    // std::cout << "m2_pitch " << m2_pitch << std::endl;
    // std::cout << "m1_roll " << m1_roll << std::endl;
    // std::cout << "m2_roll " << m2_roll << std::endl;
    // std::cout << "Mpitch " << Mpitch << std::endl;
    // std::cout << "Mroll " << Mroll << std::endl;

    Eigen::Matrix2d M;
    M << m1_pitch / 1000.0, m2_pitch / 1000.0,
        m1_roll/1000.0, m2_roll/1000.0;

    // std::cout << M << std::endl;

    Eigen::Vector2d torque(Mpitch, Mroll);
    Eigen::Vector2d force = M.colPivHouseholderQr().solve(torque);

    return force;
}

inline Eigen::Vector2d Ankle_Kin_Dyn::getTorquePR(double F_AC1, double F_AC2) {
    
    calTransMatrices();

    PointC1 = transformMatrix_B_to_C1 * PointB;
    PointC2 = transformMatrix_B_to_C2 * PointB;
    PointD = transformMatrix_B_to_D * PointB;

    Eigen::Vector3d u1 = ((PointC1 - PointA1).block<3, 1>(0, 0)).normalized();
    Eigen::Vector3d u2 = ((PointC2 - PointA2).block<3, 1>(0, 0)).normalized();

    double m1_pitch = ((PointC1 - PointB).block<3, 1>(0, 0)).cross(u1)(1);
    double m2_pitch = ((PointC2 - PointB).block<3, 1>(0, 0)).cross(u2)(1);
    double m1_roll = ((PointC1 - PointD).block<3, 1>(0, 0)).cross(u1).dot(T1.block<3, 3>(0, 0) * Eigen::Vector3d::UnitX());
    double m2_roll = ((PointC2 - PointD).block<3, 1>(0, 0)).cross(u2).dot(T1.block<3, 3>(0, 0) * Eigen::Vector3d::UnitX());

    Eigen::Matrix2d M;
    M << m1_pitch, m2_pitch,
        m1_roll, m2_roll;

    // std::cout << M << std::endl;

    Eigen::Vector2d force(F_AC1, F_AC2);
    Eigen::Vector2d torque = M * force;
    return torque;
}

inline void Ankle_Kin_Dyn::calTransMatrices() {
    // Convert angles from degrees to radians
    pitch = pitch;
    roll  = roll;

    // Build the transformation matrices
    T1 << cos(pitch),   0, sin(pitch), 0,
          0,             1, 0,          0,
          -sin(pitch),   0, cos(pitch), 0,
          0,             0, 0,          1;

    T2 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, Z_BD,
          0, 0, 0, 1;

    T3 << 1,         0,          0, 0, 
          0, cos(roll), -sin(roll), 0,
          0, sin(roll),  cos(roll), 0,
          0,         0,          0, 1;

    T4 << 1, 0, 0, X_DC,     
          0, 1, 0, Y_DC1,
          0, 0, 1, Z_DC,
          0, 0, 0, 1;

    T5 << 1, 0, 0, X_DC,     
          0, 1, 0, Y_DC2,
          0, 0, 1, Z_DC,
          0, 0, 0, 1;    

    // Calculate the full transformation matrices
    transformMatrix_B_to_C1 = T1 * T2 * T3 * T4;
    transformMatrix_B_to_C2 = T1 * T2 * T3 * T5;
    transformMatrix_B_to_D = T1 * T2 * T3;
}

inline double Ankle_Kin_Dyn::Rad2Degree(double rad) {
    return rad * (180.0 / M_PI);
}

inline double Ankle_Kin_Dyn::Degree2Rad(double degree) {
    return degree * (M_PI / 180.0);
}

//---------------------------------------------------------------------
// AnkleIKSolver Class
//---------------------------------------------------------------------
class AnkleIKSolver {
public:
    AnkleIKSolver(double pitch0, double roll0);
        
    void setMeasurements(double AC1, double AC2);
    bool solve();
    
    double getAngleP() const;
    double getAngleR() const;
    void AngleInit();
    
private:
    // Cost functor for Ceres optimization
    struct CostFunctor {
        CostFunctor(double A, double B, double C, double D, double AC1, double AC2);

        template <typename T>
        bool operator()(const T* const angles, T* residuals) const;

        const double A_, B_, C_, D_, AC1_, AC2_;
    };
    
    double A_, B_, C_, D_;
    double AC1_, AC2_;
    double angles_[2];
};

inline AnkleIKSolver::AnkleIKSolver(double pitch0, double roll0)
    : A_(-35), B_(22), C_(33.5), D_(-33.5), AC1_(0), AC2_(0) {
    angles_[0] = 0.1;
    angles_[1] = 0.1;
}

inline void AnkleIKSolver::setMeasurements(double AC1, double AC2) {
    AC1_ = AC1;
    AC2_ = AC2;
}

inline bool AnkleIKSolver::solve() {
    ceres::Problem problem;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CostFunctor, 2, 2>(
            new CostFunctor(A_, B_, C_, D_, AC1_, AC2_)), nullptr, angles_);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return summary.IsSolutionUsable();
}

inline double AnkleIKSolver::getAngleP() const {
    return angles_[0];
}

inline double AnkleIKSolver::getAngleR() const {
    return angles_[1];
}

inline void AnkleIKSolver::AngleInit() {
    angles_[0] = 0;
    angles_[1] = 0;
}

//--------------------------
// CostFunctor Definitions
//--------------------------
inline AnkleIKSolver::CostFunctor::CostFunctor(double A, double B, double C, double D, double AC1, double AC2)
: A_(A), B_(B), C_(C), D_(D), AC1_(AC1), AC2_(AC2) {}

template <typename T>
bool AnkleIKSolver::CostFunctor::operator()(const T* const angles, T* residuals) const {
    T p = angles[0];
    T r = angles[1];

    T A1[3] = {T(25.84), T(35), T(315.83)};
    T A2[3] = {T(25.84), T(-35), T(315.83)};

    T C1[3] = {
        A_ * cos(p) + B_ * sin(p) * sin(r) + C_ * sin(p) * cos(r) + D_ * sin(p),
        B_ * cos(r) - C_ * sin(r),
        -A_ * sin(p) + B_ * cos(p) * sin(r) + C_ * cos(p) * cos(r) + D_ * cos(p)
    };

    T C2[3] = {
        A_ * cos(p) - B_ * sin(p) * sin(r) + C_ * sin(p) * cos(r) + D_ * sin(p),
        -B_ * cos(r) - C_ * sin(r),
        -A_ * sin(p) - B_ * cos(p) * sin(r) + C_ * cos(p) * cos(r) + D_ * cos(p)
    };

    residuals[0] = ceres::sqrt(pow(C1[0]-A1[0],2) + pow(C1[1]-A1[1],2) + pow(C1[2]-A1[2],2)) - AC1_;
    residuals[1] = ceres::sqrt(pow(C2[0]-A2[0],2) + pow(C2[1]-A2[1],2) + pow(C2[2]-A2[2],2)) - AC2_;

    return true;
}

// Explicit template instantiations
template bool AnkleIKSolver::CostFunctor::operator()<double>(const double* const, double*) const;
template bool AnkleIKSolver::CostFunctor::operator()<ceres::Jet<double, 2>>(const ceres::Jet<double, 2>* const, ceres::Jet<double, 2>*) const;
/*
Formulas used:
For C1:
  ( A*cos(p) + B*sin(p)*sin(r) + C*sin(p)*cos(r) + D*sin(p),
    B*cos(r) - C*sin(r),
   -A*sin(p) + B*cos(p)*sin(r) + C*cos(p)*cos(r) + D*cos(p) )
For C2:
  ( A*cos(p) - B*sin(p)*sin(r) + C*sin(p)*cos(r) + D*sin(p),
   -B*cos(r) - C*sin(r),
   -A*sin(p) - B*cos(p)*sin(r) + C*cos(p)*cos(r) + D*cos(p) )
*/

