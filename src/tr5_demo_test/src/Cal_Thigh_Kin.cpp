/*
Calculate Kinemic of thigh

Run step:
cd ./path_of_this_file
g++ Cal_Thigh_Kin.cpp -o Cal_Thigh_Kin
./Cal_Thigh_Kin
*/

#include <iostream>
#include <cmath>
#include <fstream>

// 将角度转换为弧度
double DegToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

// 将弧度转换为角度
double RadToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

const double len_AB = 40.0;
const double len_AC = 44.0;
const double len_BD = 84.4;
const double len_CD = 104.7;
const double len_CE = 269.95;
const double len_AE = 312.73;

double x_A = -sin(DegToRad(10)) * len_AB;
double y_A =  cos(DegToRad(10)) * len_AB;
double x_B = 0.0;
double y_B = 0.0;

double cal_len_DE(double alpha) {
    double x_C = x_A - len_AC * sin(DegToRad(alpha));
    double y_C = y_A + len_AC * cos(DegToRad(alpha));

    double D1 = -2 * x_C;
    double E1 = -2 * y_C;
    double F1 = pow(x_C, 2) + pow(y_C, 2) - pow(len_CD, 2);
    double D2 = -2 * x_B;
    double E2 = -2 * y_B;
    double F2 = pow(x_B, 2) + pow(y_B, 2) - pow(len_BD, 2);

    double a = (E2 - E1) / (D1 - D2);
    double b = (F2 - F1) / (D1 - D2);

    double A = pow(a, 2) + 1;
    double B = 2 * a * b + a * D1 + E1;
    double C = pow(b, 2) + D1 * b + F1;

    double y_D = (-B - sqrt(pow(B, 2) - 4 * A * C)) / (2 * A);
    double x_D = a * y_D + b;

    double beta = acos((pow(len_AC, 2) + pow(len_AE, 2) - pow(len_CE, 2)) / (2 * len_AC * len_AE));
    double x_E = x_A - len_AE * sin(DegToRad(alpha) + beta);
    double y_E = y_A + len_AE * cos(DegToRad(alpha) + beta);

    double len_DE = sqrt(pow(x_E - x_D, 2) + pow(y_E - y_D, 2));
    return len_DE;
}

double calculateAlpha(double len_DE) {
    double rad_ECD = acos((pow(len_CE, 2) + pow(len_CD, 2) - pow(len_DE, 2)) / (2 * len_CE * len_CD));
    double rad_ACE = acos((pow(len_AC, 2) + pow(len_CE, 2) - pow(len_AE, 2)) / (2 * len_AC * len_CE));
    double rad_ACD = rad_ACE - rad_ECD;
    double len_AD = sqrt(pow(len_AC, 2) + pow(len_CD, 2) - 2 * len_AC * len_CD * cos(rad_ACD));
    double rad_DAB = acos((pow(len_AB, 2) + pow(len_AD, 2) - pow(len_BD, 2)) / (2 * len_AB * len_AD));
    double rad_CAD = acos((pow(len_AC, 2) + pow(len_AD, 2) - pow(len_CD, 2)) / (2 * len_AC * len_AD));

    double alpha = 19 * M_PI / 18 - rad_CAD - rad_DAB;
    return RadToDeg(alpha);
}


double calculateAlphadot(double len_DE, double len_DE_old, double dt) {
    return DegToRad((calculateAlpha(len_DE) - calculateAlpha(len_DE_old)) / dt);
}


int main() {
    int choice;
    std::cout << "选择操作:\n"
              << "1. 给定角度alpha，计算DE长度\n"
              << "2. 给定DE长度，计算角度alpha\n"
              << "3. 生成数据\n"
              << "4. 通过两次DE长度反推角度和角速度\n";
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
        std::ofstream outputFile("Kinemics_Thigh.csv");
        if (!outputFile.is_open()) {
            std::cerr << "无法打开文件!" << std::endl;
            return 1;
        }
        outputFile << "DataSet,alpha(degrees),Distance_DE\n";

        double alpha;
        double resolution = 0.001;
        double numalpha = 130 / resolution;
        for (int i = 0; i <= numalpha; ++i) {
            alpha = 0 + resolution * i;
            double len_DE = cal_len_DE(alpha);
            outputFile << (i + 1) << "," << alpha << "," << len_DE << "\n";
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
        double alpha_dot = calculateAlphadot(l_de, l_de_old, dt);

        std::cout << "当前角度 alpha = " << alpha_now << " 度" << std::endl;
        std::cout << "估算的角速度 alpha_dot = " << alpha_dot << " rad/s" << std::endl;

    } else {
        std::cout << "无效的选择。" << std::endl;
    }

    return 0;
}
