/*
 * File: utility.hpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-05-08
 * Last Modified: 2025-06-11
 */

#include <cmath>
#include <vector>
#include <stdexcept>

bool inRange(double angle, double left, double right);

bool isZero(double value);

//double getAngleInRange(double angle1, double angle2);

double distance(double target, double angle);

double normalizeAngle(double angle);

double selectBest(std::vector<double> &angle, std::vector<double> &range, double current);
