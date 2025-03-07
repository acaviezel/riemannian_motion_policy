#include <chrono>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_pose.hpp"
#include "riemannian_motion_policy/user_input_server.hpp"
#include "riemannian_motion_policy/riemannian_motion_policy.hpp"

namespace riemannian_motion_policy{

void UserInputServer::setPose(const std::shared_ptr<messages_fr3::srv::SetPose::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetPose::Response> /*response*/)
{
    (*position_d_target_)[0] = request->x;
    (*position_d_target_)[1] = request->y;
    (*position_d_target_)[2] = request->z;
    (*rotation_d_target_)[0] = request->roll;
    (*rotation_d_target_)[1] = request->pitch;
    (*rotation_d_target_)[2] = request->yaw;
}

void UserInputServer::setParam(const std::shared_ptr<messages_fr3::srv::SetParam::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetParam::Response> /*response*/)
{   
    Eigen::VectorXd diag_values_inertia(6);
    diag_values_inertia << request->a, request->b, request->c, request->d, request->e, request->f;
    auto T_placeholder = diag_values_inertia.asDiagonal();
    (*T_) = T_placeholder;
    for (int i = 0; i < 6; ++i){
        (*D_)(i,i) = 2 * sqrt((*K_)(i,i)*(*T_)(i,i)); 
    }
    std::cout << "Went into setParam" << std::endl;
}


int UserInputServer::main(int /*argc*/, char** /***argv*/)
{    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("user_input_service");
    rclcpp::Service<messages_fr3::srv::SetPose>::SharedPtr pose_service =
        node->create_service<messages_fr3::srv::SetPose> 
        ("set_pose", std::bind(&UserInputServer::setPose, this, std::placeholders::_1, std::placeholders::_2));
    rclcpp::Service<messages_fr3::srv::SetParam>::SharedPtr param_service =
        node->create_service<messages_fr3::srv::SetParam> 
        ("set_param", std::bind(&UserInputServer::setParam, this, std::placeholders::_1, std::placeholders::_2));
    

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to be called.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
}

    
