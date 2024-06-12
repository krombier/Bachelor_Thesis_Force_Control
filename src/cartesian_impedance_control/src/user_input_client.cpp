#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/srv/set_param.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <array>
#include <cmath>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("user_input_client");

    rclcpp::Client<messages_fr3::srv::SetPose>::SharedPtr pose_client =
        node->create_client<messages_fr3::srv::SetPose>("set_pose");
    auto pose_request = std::make_shared<messages_fr3::srv::SetPose::Request>();

    rclcpp::Client<messages_fr3::srv::SetParam>::SharedPtr param_client =
        node->create_client<messages_fr3::srv::SetParam>("set_param");
    auto param_request = std::make_shared<messages_fr3::srv::SetParam::Request>();

    int task_selection, param_selection;
    std::string pose_selection_str;
    char pose_selection_char;
    double store;

    while (rclcpp::ok()){
        /*                 // uncommented because I don't need the free float mode
        // IF THE PART THAT IS UNCOMMENTED IS NEEDED AGAIN MOVE ALL CODE THAT IS CURRENTLY ACTIVE TWO TABS TO THE RIGHT

        std::cout << "Enter the next task: \n [1] --> Change position \n [2] --> Change impedance parameters" << std::endl;
        std:: cin >> task_selection;
        switch (task_selection){
            case 1:{ */
        std::cout << "Enter new goal position: \n [1] --> 0.5, -0.4, 0.5 \n [2] --> DO NOT USE \n [3] --> 0.5, 0.4, 0.5\n [4] --> 0.5, 0.52, 0.5 \n [5] --> 0.5, -0.6, 0.5 \n ";
        std::cout << "[6] --> 0.5, 0.0, 0.04 \n [7] --> 0.5, 0.0, 0.4 \n [j] --> Reduce current z-Position by 1 cm \n [u] --> Increase current z-Position by 1 cm \n [w] --> Reduce current x-Position by 5 cm \n";
        std::cout << "[a] --> Reduce current y-Position by 5 cm \n [s] --> Increase current x-Position by 5 cm \n [d] --> Increase current y-Position by 5 cm \n";
        std::cout << "[4] and [5] currently reach the boundaries, so the arm will get stuck if you command this\n";
        std::cout << "FIRST SEND THE ROBOT TO A DEFINED POSITION, OTHERWISE THE PROGRAM FAILS\n";
        std::cin >> pose_selection_str;
        
        if (pose_selection_str.length() != 1) {
            std::cout << "Invalid input. Please enter a valid input." << std::endl;
            continue;
        }
        pose_selection_char = pose_selection_str[0];

        switch (pose_selection_char){
            case '1':{
                pose_request->x = 0.5;
                pose_request->y = -0.4;
                pose_request->z = 0.5;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                break;
            }
            case '2':{
                std::cout << "Enter your desired position and orientation" << std::endl;
                std::array<double, 6> pose;
                for (long unsigned int i = 0; i<pose.size(); ++i){
                    std::cin >> pose[i];
                }
                pose_request->x = pose[0];
                pose_request->y = pose[1];
                pose_request->z = pose[2];
                pose_request->roll = pose[3];
                pose_request->pitch = pose[4];
                pose_request->yaw = pose[5];
                break;
            }
            case '3':{
                pose_request->x = 0.5;
                pose_request->y = 0.4;
                pose_request->z = 0.5;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                break;
            }
            case '4':{
                pose_request->x = 0.5;
                pose_request->y = 0.52;
                pose_request->z = 0.5;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                break;
            }
            case '5':{
                pose_request->x = 0.5;
                pose_request->y = -0.6;
                pose_request->z = 0.5;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                break;
            }
            case '6':{
                pose_request->x = 0.5;
                pose_request->y = 0.0;
                pose_request->z = 0.04;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                break;
            }
            case '7':{
                pose_request->x = 0.5;
                pose_request->y = 0.0;
                pose_request->z = 0.4;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                break;
            }
            case 'j':{
                store = pose_request->z;
                pose_request->z = store - 0.01;
                break;
            }
            case 'u':{
                store = pose_request->z;
                pose_request->z = store + 0.01;
                break;
            }
            case 'w':{
                store = pose_request->x;
                pose_request->x = store - 0.05;
                break;
            }
            case 'a':{
                store = pose_request->y;
                pose_request->y = store - 0.05;
                break;
            }
            case 's':{
                store = pose_request->x;
                pose_request->x = store + 0.05;
                break;
            }
            case 'd':{
                store = pose_request->y;
                pose_request->y = store + 0.05;
                break;
            }
            default:{
                std::cout << "Invalid selection, please try again\n";
                break;
            }
        }
        auto pose_result = pose_client->async_send_request(pose_request);
        if(rclcpp::spin_until_future_complete(node, pose_result) ==  rclcpp::FutureReturnCode::SUCCESS){
            std::cout << "Hot geklappt\n";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Worked: %d", pose_result.get()->success);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setPose");
        }

        /*      //uncommented because I don't need this part
        break;
            
            }
            
            case 2:{
                std::cout << "Enter new inertia: \n [1] --> N/A \n [2] --> N/A \n [3] --> N/A\n";
                std::cin >> param_selection;
                switch(param_selection){
                    case 1:{
                        param_request->a = 2;
                        param_request->b = 0.5;
                        param_request->c = 0.5;
                        param_request->d = 2;
                        param_request->e = 0.5;
                        param_request->f = 0.5;
                        break;
                    }
                    default:{
                        param_request->a = 1;
                        param_request->b = 1;
                        param_request->c = 1;
                        param_request->d = 1;
                        param_request->e = 1;
                        param_request->f = 1;
                        break;
                    }
                }
                auto param_result = param_client->async_send_request(param_request);
                if(rclcpp::spin_until_future_complete(node, param_result) ==  rclcpp::FutureReturnCode::SUCCESS){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %d", param_result.get()->success);
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setParam");
                }
                break;
            }
            default:{
                std::cout << "Invalid selection, please try again\n";
                break;   
            }
            
        }*/
    }    
    rclcpp::shutdown();
    return 0;
}