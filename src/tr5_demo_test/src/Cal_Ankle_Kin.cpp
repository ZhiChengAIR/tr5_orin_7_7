
#include <fstream> 
#include "Transformation.h"

int main() {

    KinTransformation transform;     

    double len_A1_C1; //从机器人后面向前面看去，左腿右腿ankle关节处，左侧的直线电机为长度为A1C1，右侧的直线电机长度为A2C2
    double len_A2_C2;
    Eigen::Vector2d len_AC;

    int choice;
    std::cout << "选择操作:\n1. 给定角度pitch和roll，计算AC1和AC2长度\n2. 生成数据\n";
    std::cin >> choice;

    if (choice == 1) {

        std::cout << "请输入角度pitch, roll（度）: ";
        std::cin >> transform.pitch >> transform.roll;

        len_AC = transform.getDistanceAC();
        len_A1_C1 = len_AC[0];
        len_A2_C2 = len_AC[1];

        std::cout << "AC1长度: " << len_A1_C1 << std::endl;
        std::cout << "AC2长度: " << len_A2_C2 << std::endl;
    }
    else if(choice == 2) {

        std::ofstream outputFile("Kin_Ankel_data.csv");
        if (!outputFile.is_open()) {
            std::cerr << "无法打开文件!" << std::endl;
            return 1;
        }

        outputFile << "DataSet,Pitch(degrees),Roll(degrees),Distance_A1_C1,Distance_A2_C2\n";
        
        double resolotion = 0.01;
        double numPitch = 75.05/resolotion;
        double numRoll = 29.8/resolotion;

        for (int i = 0; i < numPitch; ++i) {
            transform.pitch = -44 + resolotion*i;

            for (int j = 0; j < numRoll; ++j){
                transform.roll = -20 + resolotion*j;

                len_AC = transform.getDistanceAC();
                len_A1_C1 = len_AC[0];
                len_A2_C2 = len_AC[1];

                outputFile << (i + 1) << "," << transform.pitch << "," << transform.roll << "," << len_A1_C1 << "," << len_A2_C2 << "\n";
            }
        }

        outputFile.close();

        std::cout << "数据已成功写入 data.csv 文件。" << std::endl;
    }
    else {
        std::cout << "无效的选择。" << std::endl;
    }
    
    return 0;
}

