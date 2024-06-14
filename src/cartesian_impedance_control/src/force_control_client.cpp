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
        std::cout << "Do we have the robots endeffector in a position where by moving it can't destroy much? [1] --> no, stop the program! [2] --> yes"<< std::endl;
        std::cin >> y_axis_check;
        std::cout << "Is checklimits not commented out in the update function and in the definition? [1] --> no, stop the program! [2] --> yes"<< std::endl;
        std::cin >> code_saftey_feature;

        if (y_axis_check==1 or code_saftey_feature==1){
            break;
        }        
        
                                                                     // right now we only move in y direction to ensure we don't damage anything
        std::cout << "Choose an option \n [1] -->  -1 N in z-direction \n [2] -->  -1.0 N in y-direction ONLY IF EMERGENCY STOP IS READY \n [182] -->  -6.0 N in z-direction\n [302] -->  -10.0 N in z-direction \n [492] increase force in negative z-direction by 1 N \n [everything else] --> no force \n"<< std::endl;
        // [2] -->  -0.1 N in y-direction \n
        std::cin >> force_direction_selection;
        switch (force_direction_selection){
            case 1:{
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -1.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                break;
            }
            
            case 2:{
                force_request->x_force = 0.0;
                force_request->y_force = -1.0;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                break;
            }
            
            case 492:{
                double store = force_request->z_force;
                force_request->z_force = store -1;
                break;
            }/*
            case 306:{
                force_request->x_force = 0.0;
                force_request->y_force = -0.1;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                break;
            }*/
           case 182:{
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -6.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                break;
            }
            case 302:{
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -10.0;
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
            std::cout << "Hat geklappt ;)"<< std::endl;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Worked: %d", force_result.get()->success);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setForce :(");
        }
    }    
    rclcpp::shutdown();
    return 0;
}