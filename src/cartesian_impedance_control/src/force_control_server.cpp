#include <chrono>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <array>

//#include "messages_fr3/srv/set_force.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cartesian_impedance_control/force_control_server.hpp"
// #include "cartesian_impedance_control/cartesian_impedance_controller.hpp"
// the commented out <include> are includes copied from user_input_server.cpp which seem unnecessary

namespace cartesian_impedance_control{

void UserInputForceServer::setForce(const std::shared_ptr<messages_fr3::srv::SetForce::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetForce::Response> /*response*/)
{
    (*force_and_moment_target)[0] = request->x_force;
    (*force_and_moment_target)[1] = request->y_force;
    (*force_and_moment_target)[2] = request->z_force;
    (*force_and_moment_target)[3] = request->x_torque;
    (*force_and_moment_target)[4] = request->y_torque;
    (*force_and_moment_target)[5] = request->z_torque;

}

int UserInputForceServer::main(int /*argc*/, char** /***argv*/)
{    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("force_control_service");
    rclcpp::Service<messages_fr3::srv::SetForce>::SharedPtr pose_service =                                                               // creates ROS2 service (resp. server) with name
        node->create_service<messages_fr3::srv::SetForce>                                                                                // set_force
        ("set_force", std::bind(&UserInputForceServer::setForce, this, std::placeholders::_1, std::placeholders::_2));
    

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to exert forces. So call me baby. Force_server_node can't wait for action!");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
}