/*
 * File: utility.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-05-08
 * Last Modified: 2025-06-11
 */

#include "utility/utility.hpp"
#include <iostream>


double distance(double target, double angle)
{
    double diff = std::fabs(angle - target);
    return std::fmin(diff, M_PI * 2 - diff);
}

bool inRange(double angle, double left, double right) {
    double theta_add = normalizeAngle(angle + 0.001);
    double theta_minus = normalizeAngle(angle - 0.001);
    if(theta_add > left  && theta_add < right)
        return true;
    if(theta_minus > left  && theta_minus < right)
        return true;

    return false;
}

//double getAngleInRange(double angle1, double angle2) {
//    bool in1 = inRange(angle1);
//    bool in2 = inRange(angle2);
//
//    if (in1 && !in2) return angle1;
//    if (!in1 && in2) return angle2;
//
//    throw std::invalid_argument("Both angles are either in or out of (-pi, pi].");
//}


double normalizeAngle(double angle)
{
    angle = std::fmod(angle, 2 * M_PI); // 先模到 (-2pi, 2pi)
    if (angle <= 0)
        angle += 2 * M_PI;  // 转到 (0, 2pi)
    return angle;    // 映射到 (0, 2pi]
}


double selectBest(std::vector<double> &angle, std::vector<double> &range, double current)
{
    std::vector<bool> flag(angle.size(), 0);
    double closest = 2 * M_PI;
    int index = -1;
    for(size_t i = 0; i < angle.size(); ++i){
        if(inRange(angle[i], range[0] , range[1])){
            double diff = distance(current, angle[i]);
            if(diff < closest){
                closest = diff;
                index = i;
            }
        }
    }
    if(index == -1)
        throw std::runtime_error("Out of range");
    return angle[index]; 
}

bool isZero(double value)
{
    if(fabs(value) < 0.001)
        return true;
    return false;
}
