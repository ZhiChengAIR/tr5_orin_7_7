#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <ceres/ceres.h>


class KinTransformation {
public:
    double pitch{0.0};  // 俯仰角
    double roll{0.0};   // 滚动角

    KinTransformation();
    Eigen::Vector2d getDistanceAC();

private:
    double Z_BD{0.0};
    double X_DC{0.0};
    double Y_DC1{0.0};
    double Y_DC2{0.0};
    double Z_DC{0.0};

    Eigen::Vector4d PointA1 = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointA2 = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointB = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointC1 = Eigen::Vector4d::Zero();
    Eigen::Vector4d PointC2 = Eigen::Vector4d::Zero();
    
    Eigen::Matrix4d transformMatrix_B_to_C1 = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d transformMatrix_B_to_C2 = Eigen::Matrix4d::Zero();

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Ones(); //旋转矩阵 B -> B'
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Ones(); //平移矩阵 B' -> D
    Eigen::Matrix4d T3 = Eigen::Matrix4d::Ones(); //旋转矩阵 D -> D'
    Eigen::Matrix4d T4 = Eigen::Matrix4d::Ones(); //平移矩阵 D' -> C1
    Eigen::Matrix4d T5 = Eigen::Matrix4d::Ones(); //平移矩阵 D' -> C2

    double Rad2Degree(double rad);
    double Degree2Rad(double degree);
    void calTransMatrices();

};


class AnkleIKSolver {
    public:
        AnkleIKSolver(double pitch0, double roll0);
        
        void setMeasurements(double AC1, double AC2);
        bool solve();
    
        double getAngleP() const;
        double getAngleR() const;
        void AngleInit();
    
    private:
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
