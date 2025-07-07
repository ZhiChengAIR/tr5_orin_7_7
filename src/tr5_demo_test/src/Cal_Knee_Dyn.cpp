/*
Calculate Kinemic of knee

Run step:
cd ./path_of_this_file
g++ Cal_Knee_Kin.cpp -o Cal_Knee_Kin
./Cal_Knee_Kin
*/

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "Hip_Knee_Kin_Dyn.hpp"

using namespace Eigen;


int main() {
    int choice;
    std::cout << "选择操作:\n"
              << "1. 给定角度alpha，计算DE长度\n"
              << "2. 给定DE长度，计算角度alpha\n"
              << "3. 生成数据\n"
              << "4. 通过两次长度反求当前角度和角速度\n"
              << "5. 给定推力F_DE，计算关节力矩\n";
    std::cin >> choice;

    if (choice == 1) {
        double alpha;
        std::cout << "请输入角度alpha（度）: ";
        std::cin >> alpha;
        double len_DE = Hip_Knee_Kin_Dyn::calKneeLength(alpha);
        std::cout << "DE长度: " << len_DE << std::endl;

    } else if (choice == 2) {
        double len_DE;
        std::cout << "请输入DE长度: ";
        std::cin >> len_DE;
        double alpha = Hip_Knee_Kin_Dyn::calKneeAngle(len_DE);
        if (alpha >= 0) {
            std::cout << "计算得到的角度alpha（度）: " << alpha << std::endl;
        } else {
            std::cout << "未能找到合适的alpha值。" << std::endl;
        }

    } else if (choice == 3) {
        std::ofstream outputFile("Kinemics_Knee.csv");
        if (!outputFile.is_open()) {
            std::cerr << "无法打开文件!" << std::endl;
            return 1;
        }
        outputFile << "DataSet,alpha(degrees),Distance_DE\n";

        double alpha;
        double resolotion = 0.01407154;
        double numalpha = 10000;
        for (int i = 0; i <= numalpha; ++i) {
            alpha = -77.153 + resolotion * i;
            double len_DE = Hip_Knee_Kin_Dyn::calKneeLength(alpha);
            outputFile << (i + 1) << "," << alpha << "," << len_DE << "\n";
        }

        std::ofstream outputFile2("IKinemics_Knee.csv");
        if (!outputFile2.is_open()) {
            std::cerr << "无法打开文件!" << std::endl;
            return 1;
        }
        outputFile2 << "DataSet,L_DE(mm),alpha\n";

        double DE;
        double resolotion2 = 0.01;
        double numDE = 100 / resolotion2;
        for (int i = 0; i <= numDE; ++i) {
            DE = 236 + resolotion2 * i;
            double Alpha = Hip_Knee_Kin_Dyn::calKneeAngle(DE);
            outputFile2 << (i + 1) << "," << DE << "," << Alpha << "\n";
        }

    } else if (choice == 4) {
        double l_de, l_de_old, dt;
        std::cout << "请输入当前DE长度 l_de（mm）: ";
        std::cin >> l_de;
        std::cout << "请输入上一时刻DE长度 l_de_old（mm）: ";
        std::cin >> l_de_old;
        std::cout << "请输入时间间隔 dt（秒）: ";
        std::cin >> dt;

        double alpha_now = Hip_Knee_Kin_Dyn::calKneeAngle(l_de);
        double alpha_old = Hip_Knee_Kin_Dyn::calKneeAngle(l_de_old);
        double alpha_dot = Hip_Knee_Kin_Dyn::degToRad(alpha_now - alpha_old) / dt;

        std::cout << "当前角度 alpha = " << alpha_now << " 度" << std::endl;
        std::cout << "估算的角速度 alpha_dot = " << alpha_dot << " rad/s" << std::endl;

    } else if (choice == 5) {
        double alpha, F_DE;
        std::cout << "请输入关节角度 alpha（rad）: ";
        std::cin >> alpha;
        std::cout << "请输入推力 F_DE（单位 N，方向为 D→E）: ";
        std::cin >> F_DE;

        double torque = Hip_Knee_Kin_Dyn::calKneeForce(alpha, F_DE);
        std::cout << "计算得到的关节力矩为: " << torque << " N·m" << std::endl;

    } else {
        std::cout << "无效的选择。" << std::endl;
    }

    return 0;
}
