#include <iostream>
#include <fstream> 
#include <cmath>
#include "Transformation.h"

int main() {
    double pitch0 = 0;
    double roll0 = 0;

    AnkleIKSolver solver(pitch0, roll0);

    double AC1, AC2;

    std::cout << "请输入AC1和AC2的长度: ";
    std::cin >> AC1 >> AC2;

    solver.setMeasurements(AC1, AC2);

    if (solver.solve()) {
        std::cout.precision(6);
        std::cout << std::fixed << "优化求解成功:\n";
        std::cout << "角度 p: " << solver.getAngleP() << " 度\n";
        std::cout << "角度 r: " << solver.getAngleR() << " 度\n";
        solver.AngleInit();
        std::cout << "角度 p: " << solver.getAngleP() << " 度\n";
        std::cout << "角度 r: " << solver.getAngleR() << " 度\n";
    } else {
        std::cout << "优化求解失败，请检查输入数据！\n";
    }

    return 0;
}

