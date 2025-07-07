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

using namespace Eigen;

// 将角度转换为弧度
double DegToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

// 将弧度转换为角度
double RadToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

double sign(double data_in){
    return data_in >= 0 ? 1.0 : -1.0;
}

const double len_AB = 50.0;
const double len_AC = 44.0;
const double len_BD = 84.4;
const double len_CD = 104.7;
const double len_CE = 242.87;
const double len_AE = 283.1;

double x_A = -sin(DegToRad(10)) * len_AB;
double y_A =  cos(DegToRad(10)) * len_AB;
double x_B = 0.0;
double y_B = 0.0;

double cal_len_DE(double alpha) {
    double delta_x_C = -sin(DegToRad(alpha)) * len_AC;
    double delta_y_C =  cos(DegToRad(alpha)) * len_AC;
    double x_C = x_A + delta_x_C;
    double y_C = y_A + delta_y_C;
    double len_BC = sqrt(pow(x_C, 2) + pow(y_C, 2));

    double RAD_ACE = acos((pow(len_AC,2)+pow(len_CE,2)-pow(len_AE,2))/(2*len_AC*len_CE));
    double RAD_ACB = acos((pow(len_AC,2)+pow(len_BC,2)-pow(len_AB,2))/(2*len_AC*len_BC));
    double RAD_BCD = acos((pow(len_BC,2)+pow(len_CD,2)-pow(len_BD,2))/(2*len_BC*len_CD));
    double RAD_DCE{0.0};

    if(alpha > 10)
        RAD_DCE = RAD_ACE - RAD_ACB - RAD_BCD;
    else
        RAD_DCE = RAD_ACE + RAD_ACB - RAD_BCD;

    double len_DE = sqrt(pow(len_CD, 2) + pow(len_CE, 2) - 2 * len_CD * len_CE * cos(RAD_DCE));
    return len_DE;
}

double calculateAlpha(double len_DE) {
    double rad_ECD = acos((pow(len_CE,2)+pow(len_CD,2)-pow(len_DE,2))/(2*len_CE*len_CD));
    double rad_ACE = acos((pow(len_AC,2)+pow(len_CE,2)-pow(len_AE,2))/(2*len_AC*len_CE));
    double rad_ACD = rad_ACE - rad_ECD;
    double len_AD =  sqrt(pow(len_AC,2)+pow(len_CD,2)-2*len_AC*len_CD*cos(rad_ACD));
    double rad_DAB = acos((pow(len_AB,2)+pow(len_AD,2)-pow(len_BD,2))/(2*len_AB*len_AD));
    double rad_CAD = acos((pow(len_AC,2)+pow(len_AD,2)-pow(len_CD,2))/(2*len_AC*len_AD));

    double alpha = 19*M_PI/18 - rad_CAD - rad_DAB;
    return RadToDeg(alpha);
}

int main() {
    int choice;
    std::cout << "选择操作:\n"
              << "1. 给定角度alpha，计算DE长度\n"
              << "2. 给定DE长度，计算角度alpha\n"
              << "3. 生成数据\n"
              << "4. 通过两次长度反求当前角度和角速度\n";
    std::cin >> choice;

    if (choice == 1) {
        double alpha;
        std::cout << "请输入角度alpha（度）: ";
        std::cin >> alpha;
        double len_DE = cal_len_DE(alpha);
        std::cout << "DE长度: " << len_DE << std::endl;

    } else if (choice == 2) {
        double len_DE;
        std::cout << "请输入DE长度: ";
        std::cin >> len_DE;
        double alpha = calculateAlpha(len_DE);
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
            double len_DE = cal_len_DE(alpha);
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
            double Alpha = calculateAlpha(DE);
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

        double alpha_now = calculateAlpha(l_de);
        double alpha_old = calculateAlpha(l_de_old);
        double alpha_dot = DegToRad(alpha_now - alpha_old) / dt;

        std::cout << "当前角度 alpha = " << alpha_now << " 度" << std::endl;
        std::cout << "估算的角速度 alpha_dot = " << alpha_dot << " rad/s" << std::endl;

    } else {
        std::cout << "无效的选择。" << std::endl;
    }

    return 0;
}
