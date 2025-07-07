#include "Transformation.h"

KinTransformation::KinTransformation() {

    PointA1 << 25.84, 35, 315.83, 1;
    PointA2 << 25.84, -35, 315.83, 1;
    PointB << 0, 0, 0, 1;

    Z_BD = -33.5;
    X_DC = -35;
    Y_DC1 = 22;
    Y_DC2 = -22;
    Z_DC = 33.5; 
}

Eigen::Vector2d KinTransformation::getDistanceAC() {

    Eigen::Vector2d Len;

    calTransMatrices();

    PointC1 = transformMatrix_B_to_C1 * PointB;
    PointC2 = transformMatrix_B_to_C2 * PointB;

    double len_AC1 = (PointA1 - PointC1).norm();
    double len_AC2 = (PointA2 - PointC2).norm();

    Len << len_AC1, len_AC2;

    return Len;
}

void KinTransformation::calTransMatrices() {

    pitch = Degree2Rad(pitch);
    roll = Degree2Rad(roll);

    T1 << cos(pitch),  0, sin(pitch),  0, 
          0,           1,          0,  0,
          -sin(pitch), 0, cos(pitch),  0,
          0,           0,          0,  1;

    T2 << 1, 0, 0,    0,            
          0, 1, 0,    0,
          0, 0, 1, Z_BD,
          0, 0, 0,    1;

    T3 << 1,         0,          0,  0, 
          0, cos(roll), -sin(roll),  0,
          0, sin(roll),  cos(roll),  0,
          0,         0,          0,  1;

    T4 << 1, 0, 0,  X_DC,     
          0, 1, 0, Y_DC1,
          0, 0, 1,  Z_DC,
          0, 0, 0,     1;

    T5 << 1, 0, 0,  X_DC,     
          0, 1, 0, Y_DC2,
          0, 0, 1,  Z_DC,
          0, 0, 0,     1;    

    transformMatrix_B_to_C1 = T1*T2*T3*T4;
    transformMatrix_B_to_C2 = T1*T2*T3*T5;
}

double KinTransformation::Rad2Degree(double rad) {
    return rad * (180.0 / M_PI);
}

double KinTransformation::Degree2Rad(double degree) {
    return degree * (M_PI / 180.0);
}


/*求解IKin*/

AnkleIKSolver::AnkleIKSolver(double pitch0, double roll0)
    : A_(-35), B_(22), C_(33.5), D_(-33.5), AC1_(0), AC2_(0) {
    angles_[0] = 0;
    angles_[1] = 0;
}

// 设置测量长度
void AnkleIKSolver::setMeasurements(double AC1, double AC2) {
    AC1_ = AC1;
    AC2_ = AC2;
}

// 执行求解
bool AnkleIKSolver::solve() {
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

double AnkleIKSolver::getAngleP() const {
    return angles_[0] * 180.0 / M_PI;
    //return angles_[0];
}

double AnkleIKSolver::getAngleR() const {
    return angles_[1] * 180.0 / M_PI;
    //return angles_[1];
}

void AnkleIKSolver::AngleInit() {
    angles_[0] = 0;
    angles_[1] = 0;
}

/* double AnkleIKSolver::getVelocityP() const {
    return 
}

double AnkleIKSolver::getVelocityR() const {

} */

// CostFunctor构造函数
AnkleIKSolver::CostFunctor::CostFunctor(double A, double B, double C, double D, double AC1, double AC2)
    : A_(A), B_(B), C_(C), D_(D), AC1_(AC1), AC2_(AC2) {}

// CostFunctor 运算符定义
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

// 显式实例化模板 (必须)
template bool AnkleIKSolver::CostFunctor::operator()<double>(const double* const, double*) const;
template bool AnkleIKSolver::CostFunctor::operator()<ceres::Jet<double, 2>>(const ceres::Jet<double, 2>* const, ceres::Jet<double, 2>*) const;
