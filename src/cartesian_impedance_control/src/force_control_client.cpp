#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_force.hpp"               // probably not needed because file doesn't exist, but included in anyways because it was done like this in user_input_client.cpp

#include <chrono>
#include <cstdlib>
#include <memory>
#include <array>                    //currently (06.05.2024) not needed
#include <cmath>                    //currently (06.05.2024) not needed

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("force_control_client");

    rclcpp::Client<messages_fr3::srv::SetForce>::SharedPtr force_client =
        node->create_client<messages_fr3::srv::SetForce>("set_force");
    auto force_request = std::make_shared<messages_fr3::srv::SetForce::Request>();

    int force_direction_selection, y_axis_check, code_saftey_feature;

    while (rclcpp::ok()){          //right now we still input a desired force
        std::cout << "Do we have the robots endeffector in position where by moving into y-direction we can't destroy much? [1] --> no, stop the program! [2] --> yes"<< std::endl;
        std::cin >> force_direction_selection;
        std::cout << "Is there a safety feature in the code that ensures that the robot doesn't exceed its limits? [1] --> no, stop the program! [2] --> yes"<< std::endl;
        std::cin >> code_saftey_feature;

        if (force_direction_selection==1 | code_saftey_feature==1){             // (| = or)
            break;
        }        
        
                                                                     // right now we only move in y direction to ensure we don't damage anything
        std::cout << "Choose an option \n [1] --> 0.1 N in positive y-direction \n [2] --> 0.1 N in negative y-direction \n [everything else] --> no force"<< std::endl;
        std::cout << "Be aware that 1 and 2 may be swaped in the program, aka I'm not sure wheter they are or not" << std::endl;
        std::cin >> force_direction_selection;
        switch (force_direction_selection){
            case 1:{
                force_request->x_force = 0.0;
                force_request->y_force = 0.1;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                break;
            }
            
            case 2:{
                force_request->x_force = 0.0;
                force_request->y_force = -0.1;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                break;
            }
            default:{
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                break;
            }
        }
        auto force_result = force_client->async_send_request(force_request);
        if(rclcpp::spin_until_future_complete(node, force_result) ==  rclcpp::FutureReturnCode::SUCCESS){
            std::cout << "Hat geklappt ;)";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Worked: %d", force_result.get()->success);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setForce :(");
        }
    }    
    rclcpp::shutdown();
    return 0;
}