/*
 * File: DH_server.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-04-29
 * Last Modified: 2025-06-11
 */
#include "joint_kinematics_node/DH_server.hpp"
#include <iostream>

DHServer::DHServer(): theta_delta(6, 0.0)
{
}

void DHServer::set_DH_params(int x, const std::vector<double> &&params)
{
    this->joint[x] << params[0], params[1], params[2], params[3];
}
void DHServer::set_theta(int index, double theta)
{
    theta_delta[index] = theta;
}
void DHServer::get_transform(Eigen::Matrix4d &T){
    T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; i++) {
        double theta_total = get_theta(i); 
        double a = get_a(i);
        double alpha = get_alpha(i);
        double d = get_d(i);
        
        double ct = cos(theta_total);
        double st = sin(theta_total);
        double ca = cos(alpha);
        double sa = sin(alpha);
        
        Eigen::Matrix4d Ti;
        Ti << ct, -st*ca, st*sa, a*ct,
               st, ct*ca, -ct*sa, a*st,
               0, sa, ca, d,
               0, 0, 0, 1;
        
        T = T * Ti; 
    }
}

void DHServer::get_transform(Eigen::Matrix4d &T, std::vector<double> &t) {
    T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; i++) {
        double theta_total = get_d0(i) + t[i]; 
        double a = get_a(i);
        double alpha = get_alpha(i);
        double d = get_d(i);
        
        double ct = cos(theta_total);
        double st = sin(theta_total);
        double ca = cos(alpha);
        double sa = sin(alpha);
        
        Eigen::Matrix4d Ti;
        Ti << ct, -st*ca, st*sa, a*ct,
               st, ct*ca, -ct*sa, a*st,
               0, sa, ca, d,
               0, 0, 0, 1;
        
        T = T * Ti;
    }
}

void DHServer::get_A03(Eigen::Matrix4d &T, std::vector<double> &t) {
    T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; i++) { 
        double theta_total = get_d0(i) + t[i];
        double a = get_a(i);
        double alpha = get_alpha(i);
        double d = get_d(i);
        
        double ct = cos(theta_total);
        double st = sin(theta_total);
        double ca = cos(alpha);
        double sa = sin(alpha);
        
        Eigen::Matrix4d Ti;
        Ti << ct, -st*ca, st*sa, a*ct,
               st, ct*ca, -ct*sa, a*st,
               0, sa, ca, d,
               0, 0, 0, 1;
        
        T = T * Ti;
    }
}

void DHServer::get_A03(Eigen::Matrix4d &T) {
    T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; i++) { 
        double theta_total = get_theta(i); 
        double a = get_a(i);
        double alpha = get_alpha(i);
        double d = get_d(i);
        
        double ct = cos(theta_total);
        double st = sin(theta_total);
        double ca = cos(alpha);
        double sa = sin(alpha);
        
        Eigen::Matrix4d Ti;
        Ti << ct, -st*ca, st*sa, a*ct,
               st, ct*ca, -ct*sa, a*st,
               0, sa, ca, d,
               0, 0, 0, 1;
        
        T = T * Ti;
    }
}


double DHServer::get_a(int joint)
{
    return this->joint[joint][0];
}

double DHServer::get_alpha(int joint)
{
    return this->joint[joint][1];
}

double DHServer::get_d(int joint)
{
    return this->joint[joint][2];
}

double DHServer::get_theta(int joint)
{
    return this->joint[joint][3] + this->theta_delta[joint];
}
double DHServer::get_delta(int joint)
{
    return this->theta_delta[joint];
}
double DHServer::get_d0(int joint)
{
    return this->joint[joint][3];
}

