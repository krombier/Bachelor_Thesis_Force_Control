// so far this was just copy pasted and stupidly adapted from user_input_server.hpp
#pragma once                            //make sure this file is only included once by compiler, if done multiple times this may cause a compiler error

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <messages_fr3/srv/set_force.hpp>                     // I wrote this part of the code, just because I'm following along what others did. But this file doesn't exist. Maybe it is a reference to the messages_fr3::srv::SetForce.srv

namespace cartesian_impedance_control {

class UserInputForceServer {
public:
  UserInputForceServer (Eigen::Matrix<double, 6, 1>* force_and_moment) :    //constructor??
  force_and_moment_target(force_and_moment){}
  int main(int argc, char **argv);

private:
  Eigen::Matrix<double, 6, 1>* force_and_moment_target;
  void setForce(const std::shared_ptr<messages_fr3::srv::SetForce::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetForce::Response> response);
};
}

