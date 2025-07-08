/*
 * File: joint_kinematics_node.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-04-29
 * Last Modified: 2025-06-12
 */

#include "joint_kinematics_node/joint_kinematics_node.hpp"
#include "utility/utility.hpp"
#include <cmath>


JointKinematicsNode::JointKinematicsNode()
: Node("kinematics_node"), range_(6)
{
    this->declare_parameter("joint_1", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_2", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_3", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_4", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_5", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_6", rclcpp::PARAMETER_DOUBLE_ARRAY);

    dh_server.set_DH_params(0, std::move(this->get_parameter("joint_1").as_double_array()));
    dh_server.set_DH_params(1, std::move(this->get_parameter("joint_2").as_double_array()));
    dh_server.set_DH_params(2, std::move(this->get_parameter("joint_3").as_double_array()));
    dh_server.set_DH_params(3, std::move(this->get_parameter("joint_4").as_double_array()));
    dh_server.set_DH_params(4, std::move(this->get_parameter("joint_5").as_double_array()));
    dh_server.set_DH_params(5, std::move(this->get_parameter("joint_6").as_double_array()));

    range_[0].set(M_PI / 2, 2 * M_PI);
    range_[1].set(0, M_PI);
    range_[2].set(M_PI, 2 * M_PI);
    range_[3].set(M_PI / 2, 2 * M_PI);
    range_[4].set(0,  M_PI);

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointKinematicsNode::jointStateCallback, this, std::placeholders::_1)
    );

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/gui_pose", 10,
      std::bind(&JointKinematicsNode::poseCallback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    //timer_ = this->create_wall_timer(
    //        std::chrono::milliseconds(10),  // 每 10ms触发一次
    //        std::bind(&JointKinematicsNode::timerCallback, this));
}

void JointKinematicsNode::timerCallback()
{
    sensor_msgs::msg::JointState msg;
    msg.position =  {0,0,0,0,0,0};
    msg.velocity =  {0,0,0,0,0,0};
    msg.effort   =  {0,0,0,0,0,0};
    msg.header.stamp = rclcpp::Clock().now();
    msg.name = {
    "base_to_link0",
    "link0_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5"};

    pub_->publish(msg);

}

void JointKinematicsNode::poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    Eigen::Vector3d t;
    t << msg->position.x , msg->position.y, msg->position.z;
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    inverseKinematics(t, q);
}

void JointKinematicsNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Received JointState:");
    for (size_t i = 0; i < msg->name.size(); ++i) {
        /*
      RCLCPP_INFO(this->get_logger(), "Joint %s: position=%.2f velocity=%.2f effort=%.2f",
                  msg->name[i].c_str(),
                  msg->position.size() > i ? msg->position[i] : 0.0,
                  msg->velocity.size() > i ? msg->velocity[i] : 0.0,
                  msg->effort.size() > i ? msg->effort[i] : 0.0);
                  */
      dh_server.set_theta(i, msg->position[i]);
    }
    forwardKinematics();
}
void JointKinematicsNode::forwardKinematics()
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector4d zero;
    zero << 0, 0, 0, 1;
    dh_server.get_transform(T);
    //Eigen::Vector4d end = T *zero;
    //RCLCPP_INFO(this->get_logger(), "FK x: %.3f, y: %.3f, z: %.3f", end(0), end(1), end(2));
    //Eigen::Matrix3d R = T.topLeftCorner(3, 3);
    //Eigen::Quaterniond q(R);
    //Eigen::Vector3d t = T.topRightCorner(3, 1);
    //RCLCPP_INFO(this->get_logger(), "q x: %.3f, y: %.3f, z: %.3f w: %.3f", q.x(), q.y(), q.z(), q.w());

    //inverseKinematics(t, q);
}

void JointKinematicsNode::inverseKinematics(Eigen::Vector3d &v, Eigen::Quaterniond &q)
{
    try{
        Eigen::Vector3d arm6;
        RCLCPP_INFO(this->get_logger(), "Solve IK");

        //Step 0
        arm6 << 0, 0, dh_server.get_d(5);
        Eigen::Vector3d j5 = v - q * arm6;
        //RCLCPP_INFO(this->get_logger(), "Joint5 x: %.3f, y: %.3f, z: %.3f", j5.x(), j5.y(), j5.z());

        //Step 1
        double a2 = dh_server.get_a(1);
        double a3 = dh_server.get_a(2);
        double d4 = dh_server.get_d(3);
        double c = (j5.squaredNorm() - a2 * a2 - a3 * a3 -d4 * d4) / (2 * a2);
        double R = std::sqrt(a3 * a3 + d4 * d4);

        double beta = std::atan2(d4, a3);
        double tmp = std::acos(c/R);
        std::vector<double> j3{normalizeAngle( tmp - beta - dh_server.get_d0(2)),
                               normalizeAngle(-tmp - beta - dh_server.get_d0(2))};
        std::vector<double> j3_candidate;

        for(auto t : j3){
            if(inRange(t, range_[2].left, range_[2].right))
                j3_candidate.push_back(t);
        }
        //RCLCPP_INFO(this->get_logger(), "theta3 %.3f, GT:%.3f", theta[2], theta[2] - dh_server.get_delta(2));

        //Step 2
        double theta1 = std::atan2(j5.y(), j5.x()) - dh_server.get_d0(0);
        std::vector<double> j1_candidate;
        if(inRange(theta1, range_[0].left, range_[0].right))
            j1_candidate.push_back(theta1);
        if(inRange(theta1 + M_PI, range_[0].left, range_[0].right))
            j1_candidate.push_back(theta1 + M_PI);

        //Step 3
        std::vector<std::vector<double>> j2_candidate;
        j2_candidate.reserve(8);
        for(auto theta : j3_candidate){
            for(auto j1_value : j1_candidate){
                double phi = theta + dh_server.get_d0(2) + beta;
                double R2 = std::sqrt(std::pow(R * std::cos(phi) + a2,2) + std::pow(R * std::sin(phi), 2));
                double beta2 = std::atan2(R * std::sin(phi), R * std::cos(phi) + a2);

                double v1 = j5.topRows(2).norm();
                std::vector<double> j2;
                if(isZero(std::cos(j1_value))){
                    j2.push_back(std::acos(v1 / R2) - beta2);
                    j2.push_back(-std::acos(v1 / R2) - beta2);
                    j2.push_back(std::acos(-v1 / R2) - beta2);
                    j2.push_back(-std::acos(-v1 / R2) - beta2);
                }else{
                    double dir_flag = j5(0) / std::cos(j1_value);
                    if(dir_flag < 0){
                        j2.push_back(std::acos(v1 / R2) - beta2);
                        j2.push_back(-std::acos(v1 / R2) - beta2);
                    }else{
                        j2.push_back(std::acos(-v1 / R2) - beta2);
                        j2.push_back(-std::acos(-v1 / R2) - beta2);
                    }
                }

                for(auto value : j2){
                    if(inRange(value, range_[1].left, range_[1].right)){
                        j2_candidate.push_back(std::vector<double>{j1_value, value, theta});
                    }
                }
            }
        }

        std::vector<std::vector<double>> candidate;
        //Get joint angles of the spherical wrist 
        for(auto theta : j2_candidate){
            Eigen::Matrix4d A03, A;
            A03 = Eigen::Matrix4d::Identity();
            A = Eigen::Matrix4d::Identity();
            dh_server.get_A03(A03, theta);
            A.topLeftCorner(3, 3) = q.toRotationMatrix();
            A.topRightCorner(3, 1) = v;
            Eigen::Matrix4d A36 = A03.inverse() * A;

            //Step 4 theta4
            std::vector<double> tmp0(theta);
            std::vector<double> tmp1(theta);
            bool vaild0 = true;
            bool vaild1 = true;
            tmp0.push_back(std::atan2(A36(1, 2), A36(0, 2)));
            if(!inRange(tmp0.back(), range_[3].left, range_[3].right)){
                vaild0 = false;
            }
            tmp1.push_back(std::atan2(-A36(1, 2), -A36(0, 2)));
            if(!inRange(tmp1.back(), range_[3].left, range_[3].right)){
                vaild1 = false;
            }

            //Step theta5
            double temp2 = std::sqrt(A36(1, 2) * A36(1, 2) + A36(0, 2) * A36(0, 2));
            tmp0.push_back(std::atan2(-temp2, A36(2, 2)) -dh_server.get_d0(4));
            if(!inRange(tmp0.back(), range_[4].left, range_[4].right)){
                vaild0 = false;
            }
            tmp1.push_back(std::atan2(temp2, A36(2, 2)) -dh_server.get_d0(4));
            if(!inRange(tmp1.back(), range_[4].left, range_[4].right)){
                vaild1 = false;
            }

            //Step theta6
            tmp0.push_back(std::atan2(A36(2, 1), -A36(2, 0)));
            tmp1.push_back(std::atan2(-A36(2, 1), A36(2, 0)));

            if(vaild0)
                candidate.push_back(tmp0);
            if(vaild1)
                candidate.push_back(tmp1);
        }
        RCLCPP_INFO(this->get_logger(), "full size %lu", candidate.size());
        std::vector<uint8_t> score(candidate.size(), 0);
        for(size_t i = 0; i < candidate.size(); ++i){
            RCLCPP_INFO(this->get_logger(), "candidate %lu %.4f %.4f %.4f %.4f %.4f %.4f",
                    i, candidate[i][0], candidate[i][1], candidate[i][2],
                    candidate[i][3], candidate[i][4], candidate[i][5]);
            Eigen::Matrix4d tempT = Eigen::Matrix4d::Identity(); 
            dh_server.get_transform(tempT, candidate[i]);
            Eigen::Vector3d solvedEnd = tempT.topRightCorner(3,1);
            double dis = (solvedEnd - v).norm();
            if(dis > 0.01){
                score[i] = 1 / dis;
            }
            else{
                double total_delta = 0.0;
                for(size_t j = 0; j < candidate[i].size();++j){
                    total_delta += fabs(normalizeAngle(candidate[i][j] - dh_server.get_delta(j)));
                }
                score[i] = 100 + 1 / total_delta;
            }
        }
        if(candidate.size() > 0){
            auto it = std::max_element(score.begin(), score.end());
            int index = std::distance(score.begin(), it);
            RCLCPP_INFO(this->get_logger(), "candidate %d is selected", index);

            sensor_msgs::msg::JointState msg;
            msg.position.assign(candidate[index].begin(), candidate[index].end());
            msg.velocity = {0,0,0,0,0,0};
            msg.effort   = {0,0,0,0,0,0};
            msg.header.stamp = rclcpp::Clock().now();
            msg.name = {
            "base_to_link0",
            "link0_to_link1",
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5"
            };
            pub_->publish(msg);
        }else{
            RCLCPP_ERROR(this->get_logger(), "Not Reachable!");
        }

    }
    catch(const std::runtime_error &e){
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
