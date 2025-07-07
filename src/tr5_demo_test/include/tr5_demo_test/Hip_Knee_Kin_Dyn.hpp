#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

class Hip_Knee_Kin_Dyn {
public:
    // Conversion functions.
    static inline double degToRad(double degrees) {
        return degrees * M_PI / 180.0;
    }
    static inline double radToDeg(double radians) {
        return radians * 180.0 / M_PI;
    }

    // ----- Hip constants and global points -----
    static constexpr double hip_len_AB = 40.0;
    static constexpr double hip_len_AC = 44.0;
    static constexpr double hip_len_BD = 84.4;
    static constexpr double hip_len_CD = 104.7;
    static constexpr double hip_len_CE = 269.95;
    static constexpr double hip_len_AE = 312.73;

    // ----- Knee constants and global points -----
    static constexpr double knee_len_AB = 50.0;
    static constexpr double knee_len_AC = 44.0;
    static constexpr double knee_len_BD = 84.4;
    static constexpr double knee_len_CD = 104.7;
    static constexpr double knee_len_CE = 242.87;
    static constexpr double knee_len_AE = 283.1;

private:
    static constexpr double fixed_angle = 10.0;

    static inline double hip_x_A = -std::sin(degToRad(fixed_angle)) * hip_len_AB;
    static inline double hip_y_A = std::cos(degToRad(fixed_angle)) * hip_len_AB;
    static inline double hip_x_B = 0.0;
    static inline double hip_y_B = 0.0;

    static inline double knee_x_A = -std::sin(degToRad(fixed_angle)) * knee_len_AB;
    static inline double knee_y_A = std::cos(degToRad(fixed_angle)) * knee_len_AB;
    static inline double knee_x_B = 0.0;
    static inline double knee_y_B = 0.0;

public:
    // --- Hip functions ---
    static double calHipLength(double alpha) {
        double initial_degree = degToRad(90.0);
        alpha += initial_degree;

        double x_C = hip_x_A - sin(alpha) * hip_len_AC;
        double y_C = hip_y_A + cos(alpha) * hip_len_AC;
        double len_BC = sqrt(x_C * x_C + y_C * y_C);

        double RAD_ACE = acos((pow(hip_len_AC,2)+pow(hip_len_CE,2)-pow(hip_len_AE,2)) / (2 * hip_len_AC * hip_len_CE));
        double RAD_ACB = acos((pow(hip_len_AC,2)+pow(len_BC,2)-pow(hip_len_AB,2)) / (2 * hip_len_AC * len_BC));
        double RAD_BCD = acos((pow(len_BC,2)+pow(hip_len_CD,2)-pow(hip_len_BD,2)) / (2 * len_BC * hip_len_CD));
        double RAD_DCE;

        if (alpha > degToRad(10.0))
            RAD_DCE = RAD_ACE - RAD_ACB - RAD_BCD;
        else
            RAD_DCE = RAD_ACE + RAD_ACB - RAD_BCD;

        return sqrt(pow(hip_len_CD, 2) + pow(hip_len_CE, 2) - 2.0 * hip_len_CD * hip_len_CE * cos(RAD_DCE));
    }

    static double calHipAngle(double len_DE) {
        double rad_ECD = acos((pow(hip_len_CE,2)+pow(hip_len_CD,2)-pow(len_DE,2))/(2*hip_len_CE*hip_len_CD));
        double rad_ACE = acos((pow(hip_len_AC,2)+pow(hip_len_CE,2)-pow(hip_len_AE,2))/(2*hip_len_AC*hip_len_CE));
        double rad_ACD = rad_ACE - rad_ECD;
        double len_AD = sqrt(pow(hip_len_AC,2)+pow(hip_len_CD,2)-2*hip_len_AC*hip_len_CD*cos(rad_ACD));
        double rad_DAB = acos((pow(hip_len_AB,2)+pow(len_AD,2)-pow(hip_len_BD,2))/(2*hip_len_AB*len_AD));
        double rad_CAD = acos((pow(hip_len_AC,2)+pow(len_AD,2)-pow(hip_len_CD,2))/(2*hip_len_AC*len_AD));
        double alpha = 19 * M_PI / 18 - rad_CAD - rad_DAB - degToRad(90.0);
        return alpha;
    }

    static double calHipTorque(double alpha_rad, double F_DE) {
        double x_A = -sin(degToRad(10)) * hip_len_AB;
        double y_A =  cos(degToRad(10)) * hip_len_AB;
        double x_C = x_A - sin(alpha_rad) * hip_len_AC;
        double y_C = y_A + cos(alpha_rad) * hip_len_AC;
        double len_BC = sqrt(x_C*x_C + y_C*y_C);

        double angle_ACB = acos((pow(hip_len_AC,2) + pow(len_BC,2) - pow(hip_len_AB,2)) / (2 * hip_len_AC * len_BC));
        double angle_BCD = acos((pow(len_BC,2) + pow(hip_len_CD,2) - pow(hip_len_BD,2)) / (2 * len_BC * hip_len_CD));

        double theta = (alpha_rad > degToRad(10)) ?
            alpha_rad - angle_ACB - angle_BCD :
            alpha_rad + angle_ACB - angle_BCD;

        double x_D = x_C - sin(theta) * hip_len_CD;
        double y_D = y_C + cos(theta) * hip_len_CD;

        double RAD_ACE = acos((pow(hip_len_AC,2)+pow(hip_len_CE,2)-pow(hip_len_AE,2)) / (2 * hip_len_AC * hip_len_CE));
        double RAD_DCE = (alpha_rad > degToRad(10)) ?
            RAD_ACE - angle_ACB - angle_BCD :
            RAD_ACE + angle_ACB - angle_BCD;

        double x_E = x_C + sin(RAD_DCE) * hip_len_CE;
        double y_E = y_C - cos(RAD_DCE) * hip_len_CE;

        Eigen::Vector2d r(x_E - x_D, y_E - y_D);
        Eigen::Vector2d dir = r.normalized();
        Eigen::Vector2d F = F_DE * dir;

        Eigen::Vector2d r_joint(x_D - 0.0, y_D - 0.0); // B is origin
        double torque_Nmm = r_joint.x() * F.y() - r_joint.y() * F.x();
        return torque_Nmm / 1000.0;
    } 

    static double calHipForce(double alpha_rad, double torque_Nm) {
        alpha_rad = alpha_rad;
	alpha_rad = 1.5;
        //std::cout << "alpha_rad " << alpha_rad << std::endl;
        double x_A = -sin(degToRad(10)) * hip_len_AB;
        double y_A =  cos(degToRad(10)) * hip_len_AB;
        double x_C = x_A - sin(alpha_rad) * hip_len_AC;
        double y_C = y_A + cos(alpha_rad) * hip_len_AC;
        double len_BC = sqrt(x_C*x_C + y_C*y_C);
        // std::cout << "x_A " << x_A << std::endl;
        // std::cout << "y_A " << y_A << std::endl;
        // std::cout << "x_C " << x_C << std::endl;
        // std::cout << "y_C " << y_C << std::endl;

        double angle_ACB = acos((pow(hip_len_AC,2) + pow(len_BC,2) - pow(hip_len_AB,2)) / (2 * hip_len_AC * len_BC));
        double angle_BCD = acos((pow(len_BC,2) + pow(hip_len_CD,2) - pow(hip_len_BD,2)) / (2 * len_BC * hip_len_CD));
        double theta = angle_ACB + angle_BCD - alpha_rad;
        // double theta = (alpha_rad > degToRad(10)) ?
        //     alpha_rad - angle_ACB - angle_BCD :
            // alpha_rad + angle_ACB - angle_BCD;
        // std::cout << "hip_len_CD " << hip_len_CD << std::endl;
        // std::cout << "theta " << theta << std::endl;
        double x_D = x_C - sin(theta) * hip_len_CD;
        double y_D = y_C - cos(theta) * hip_len_CD;

        double RAD_ACE = acos((pow(hip_len_AC,2)+pow(hip_len_CE,2)-pow(hip_len_AE,2)) / (2 * hip_len_AC * hip_len_CE));
        double RAD_DCE = (alpha_rad > degToRad(10)) ?
            RAD_ACE - angle_ACB - angle_BCD :
            RAD_ACE + angle_ACB - angle_BCD;

        double x_E = x_C - sin(RAD_DCE + theta) * hip_len_CE;
        double y_E = y_C - cos(RAD_DCE + theta) * hip_len_CE;
        // std::cout << "x_D " << x_D << std::endl;
        // std::cout << "y_D " << y_D << std::endl;
        // std::cout << "x_E " << x_E << std::endl;
        // std::cout << "y_E " << y_E << std::endl;

        //Eigen::Vector2d r_DE(x_E - x_D, y_E - y_D);
        //double dx = x_E - x_D;
        //double dy = y_E - y_D;
        //double numerator = std::abs(dx * (y_A - y_D) - dy * (x_A - x_D));
        //double denominator = std::sqrt(dx * dx + dy * dy);
        //double res = numerator / denominator;
        // std::cout << "res hip " << res << std::endl;
        //Eigen::Vector2d u_DE = r_DE.normalized();
        //Eigen::Vector2d r_joint(x_D+x_A, y_D+y_A); // from B (origin) to D

        //double moment_arm = r_joint.x() * u_DE.y() - r_joint.y() * u_DE.x();
        
	double DA_x = x_A - x_D;
        double DA_y = y_A - y_D;
        double DE_x = x_E - x_D;
        double DE_y = y_E - y_D;
        double cross_val = std::abs((DA_x * DE_y) - (DA_y * DE_x));
        double length_DE = std::sqrt(DE_x * DE_x + DE_y * DE_y);
        double moment_arm = cross_val / length_DE;
	//std::cout << "moment_arm" << std::endl;
        //std::cout << moment_arm << std::endl;
        // std::cout << "_joint.x()" << std::endl;
        // std::cout << r_joint.x() << std::endl;
        // std::cout << "u_DE.y()" << std::endl;
        // std::cout << u_DE.y() << std::endl;
        // std::cout << "r_joint.y()" << std::endl;
        // std::cout << r_joint.y() << std::endl;
        // std::cout << "u_DE.x()" << std::endl;
        // std::cout << u_DE.x() << std::endl;
	//moment_arm = 100;
        if (std::abs(moment_arm) < 1e-8) return 0.0;

        return torque_Nm / (moment_arm * 1e-3); // 单位：N // scalar, sign indicates direction (D->E positive)
    }

    // --- Knee functions ---
    static double calKneeLength(double alpha) {
        alpha = -alpha;
        double initial_degree = degToRad(43.68);
        alpha += initial_degree;

        double x_C = knee_x_A - sin(alpha) * knee_len_AC;
        double y_C = knee_y_A + cos(alpha) * knee_len_AC;
        double len_BC = sqrt(x_C * x_C + y_C * y_C);

        double RAD_ACE = acos((pow(knee_len_AC,2)+pow(knee_len_CE,2)-pow(knee_len_AE,2)) / (2 * knee_len_AC * knee_len_CE));
        double RAD_ACB = acos((pow(knee_len_AC,2)+pow(len_BC,2)-pow(knee_len_AB,2)) / (2 * knee_len_AC * len_BC));
        double RAD_BCD = acos((pow(len_BC,2)+pow(knee_len_CD,2)-pow(knee_len_BD,2)) / (2 * len_BC * knee_len_CD));
        double RAD_DCE;

        if (alpha > degToRad(10.0))
            RAD_DCE = RAD_ACE - RAD_ACB - RAD_BCD;
        else
            RAD_DCE = RAD_ACE + RAD_ACB - RAD_BCD;

        return sqrt(pow(knee_len_CD, 2) + pow(knee_len_CE, 2) - 2.0 * knee_len_CD * knee_len_CE * cos(RAD_DCE));
    }

    static double calKneeAngle(double len_DE) {
        double rad_ECD = acos((pow(knee_len_CE,2)+pow(knee_len_CD,2)-pow(len_DE,2))/(2*knee_len_CE*knee_len_CD));
        double rad_ACE = acos((pow(knee_len_AC,2)+pow(knee_len_CE,2)-pow(knee_len_AE,2))/(2*knee_len_AC*knee_len_CE));
        double rad_ACD = rad_ACE - rad_ECD;
        double len_AD = sqrt(pow(knee_len_AC,2)+pow(knee_len_CD,2)-2*knee_len_AC*knee_len_CD*cos(rad_ACD));
        double rad_DAB = acos((pow(knee_len_AB,2)+pow(len_AD,2)-pow(knee_len_BD,2))/(2*knee_len_AB*len_AD));
        double rad_CAD = acos((pow(knee_len_AC,2)+pow(len_AD,2)-pow(knee_len_CD,2))/(2*knee_len_AC*len_AD));
        double alpha = 19 * M_PI / 18 - rad_CAD - rad_DAB - degToRad(43.68);
        return -alpha;
    }

    static double calKneeTorque(double alpha_rad, double F_DE) {
        double x_A = -sin(degToRad(10)) * knee_len_AB;
        double y_A =  cos(degToRad(10)) * knee_len_AB;
        double x_C = x_A - sin(alpha_rad) * knee_len_AC;
        double y_C = y_A + cos(alpha_rad) * knee_len_AC;
        double len_BC = sqrt(x_C*x_C + y_C*y_C);
    
        double angle_ACB = acos((pow(knee_len_AC,2) + pow(len_BC,2) - pow(knee_len_AB,2)) / (2 * knee_len_AC * len_BC));
        double angle_BCD = acos((pow(len_BC,2) + pow(knee_len_CD,2) - pow(knee_len_BD,2)) / (2 * len_BC * knee_len_CD));
    
        double theta = (alpha_rad > degToRad(10)) ?
            alpha_rad - angle_ACB - angle_BCD :
            alpha_rad + angle_ACB - angle_BCD;
    
        double x_D = x_C - sin(theta) * knee_len_CD;
        double y_D = y_C + cos(theta) * knee_len_CD;
    
        double RAD_ACE = acos((pow(knee_len_AC,2)+pow(knee_len_CE,2)-pow(knee_len_AE,2)) / (2 * knee_len_AC * knee_len_CE));
        double RAD_DCE = (alpha_rad > degToRad(10)) ?
            RAD_ACE - angle_ACB - angle_BCD :
            RAD_ACE + angle_ACB - angle_BCD;
    
        double x_E = x_C + sin(RAD_DCE) * knee_len_CE;
        double y_E = y_C - cos(RAD_DCE) * knee_len_CE;
    
        Eigen::Vector2d r(x_E - x_D, y_E - y_D);
        Eigen::Vector2d dir = r.normalized();
        Eigen::Vector2d F = F_DE * dir;
    
        Eigen::Vector2d r_joint(x_D - 0.0, y_D - 0.0); // B is origin
        double torque_Nmm = r_joint.x() * F.y() - r_joint.y() * F.x();
        return torque_Nmm / 1000.0; // N·m
    } 

    static double calKneeForce(double alpha_rad, double torque_Nm) {
        alpha_rad = alpha_rad;
	//std::cout << "alpha_rad " << alpha_rad << std::endl; 
	double x_A = -sin(degToRad(10)) * knee_len_AB;
        double y_A =  cos(degToRad(10)) * knee_len_AB;
        double x_C = x_A - sin(alpha_rad) * knee_len_AC;
        double y_C = y_A + cos(alpha_rad) * knee_len_AC;
        double len_BC = sqrt(x_C*x_C + y_C*y_C);
        // std::cout << "x_A " << x_A << std::endl;
        // std::cout << "y_A " << y_A << std::endl;
        // std::cout << "x_C " << x_C << std::endl;
        // std::cout << "y_C " << y_C << std::endl;

        double angle_ACB = acos((pow(knee_len_AC,2) + pow(len_BC,2) - pow(knee_len_AB,2)) / (2 * knee_len_AC * len_BC));
        double angle_BCD = acos((pow(len_BC,2) + pow(knee_len_CD,2) - pow(knee_len_BD,2)) / (2 * len_BC * knee_len_CD));
        double theta = angle_ACB + angle_BCD - alpha_rad;
        // double theta = (alpha_rad > degToRad(10)) ?
        //     alpha_rad - angle_ACB - angle_BCD :
            // alpha_rad + angle_ACB - angle_BCD;
        // std::cout << "knee_len_CD " << knee_len_CD << std::endl;
        // std::cout << "theta " << theta << std::endl;
        double x_D = x_C - sin(theta) * knee_len_CD;
        double y_D = y_C - cos(theta) * knee_len_CD;

        double RAD_ACE = acos((pow(knee_len_AC,2)+pow(knee_len_CE,2)-pow(knee_len_AE,2)) / (2 * knee_len_AC * knee_len_CE));
        double RAD_DCE = (alpha_rad > degToRad(10)) ?
            RAD_ACE - angle_ACB - angle_BCD :
            RAD_ACE + angle_ACB - angle_BCD;

        double x_E = x_C - sin(RAD_DCE + theta) * knee_len_CE;
        double y_E = y_C - cos(RAD_DCE + theta) * knee_len_CE;
        // std::cout << "x_D " << x_D << std::endl;
        // std::cout << "y_D " << y_D << std::endl;
        // std::cout << "x_E " << x_E << std::endl;
        // std::cout << "y_E " << y_E << std::endl;

        //Eigen::Vector2d r_DE(x_E - x_D, y_E - y_D);
        //double dx = x_E - x_D;
        //double dy = y_E - y_D;
        //double numerator = std::abs(dx * (y_A - y_D) - dy * (x_A - x_D));
        //double denominator = std::sqrt(dx * dx + dy * dy);
        //double res = numerator / denominator;
        // std::cout << "res knee " << res << std::endl;
        //Eigen::Vector2d u_DE = r_DE.normalized();
        //Eigen::Vector2d r_joint(x_D+x_A, y_D+y_A); // from B (origin) to D

        //double moment_arm = r_joint.x() * u_DE.y() - r_joint.y() * u_DE.x();

	double DA_x = x_A - x_D;
        double DA_y = y_A - y_D;
        double DE_x = x_E - x_D;
        double DE_y = y_E - y_D;
        double cross_val = std::abs((DA_x * DE_y) - (DA_y * DE_x));
        double length_DE = std::sqrt(DE_x * DE_x + DE_y * DE_y);
        double moment_arm = cross_val / length_DE;

        //std::cout << "moment_arm" << std::endl;
        //std::cout << moment_arm << std::endl;
        // std::cout << "_joint.x()" << std::endl;
        // std::cout << r_joint.x() << std::endl;
        // std::cout << "u_DE.y()" << std::endl;
        // std::cout << u_DE.y() << std::endl;
        // std::cout << "r_joint.y()" << std::endl;
        // std::cout << r_joint.y() << std::endl;
        // std::cout << "u_DE.x()" << std::endl;
        // std::cout << u_DE.x() << std::endl;
        //moment_arm = 90;
	if (std::abs(moment_arm) < 1e-8) return 0.0;

        return torque_Nm / (moment_arm * 1e-3); // 单位：N // scalar, sign indicates direction (D->E positive)
    }    
};
