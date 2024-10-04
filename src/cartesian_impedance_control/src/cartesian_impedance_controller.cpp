// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cartesian_impedance_control/cartesian_impedance_controller.hpp>                                     // Here we get all our variables from. Wish I'd known that earlier.

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>      // includes the whole Eigen library

// void CartesianImpedanceController::function_name(argument 1, ...) is a member function of the class CartesianImpedanceController with name function_name

namespace {

// the following template allows the following type of code to work:
// std::cout << name_of_array << std::endl; 

// implement a standart array by:
// std::array<int, 5> myArray = {1, 2, 3, 4, 5};

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}

namespace cartesian_impedance_control {

CartesianImpedanceController::CartesianImpedanceController() {}

void CartesianImpedanceController::change_wrench_to_base_frame(Eigen::Matrix<double, 6, 1>& F_cmd, const Eigen::Matrix<double,3,3>& rot_matrix,
                                                               const Eigen::Matrix<double,3,1>& position /*the position is the translation between the 2 frames*/){
  Eigen::Vector3d force_eff = F_cmd.head<3>();    // First 3 elements: [f_x, f_y, f_z]
  Eigen::Vector3d moment_eff = F_cmd.tail<3>();   // Last 3 elements: [m_x, m_y, m_z]
  Eigen::Vector3d force_base_frame = rot_matrix * force_eff;
  Eigen::Vector3d moment_base_frame = rot_matrix * moment_eff + position.cross(force_base_frame);
  F_cmd.head<3>() = force_base_frame;
  F_cmd.tail<3>() = moment_base_frame;
                                                               }

void CartesianImpedanceController::update_forces(Eigen::Matrix<double, 6, 1>& F_observed, 
                                                Eigen::Matrix<double, 7, 1>& beta_observer, 
                                                Eigen::Matrix<double, 7, 1>& gamma_observer,
                                                Eigen::Matrix<double, 7, 1>& r_observer,
                                                const Eigen::Matrix<double, 7, 7>& K_observer,
                                                const Eigen::Matrix<double, 7, 1>& coriolis,
                                                const Eigen::Matrix<double, 7, 7>& M_dot,
                                                const Eigen::Matrix<double, 7, 1>& dq_,
                                                const Eigen::Matrix<double, 7, 1>& tau_d,
                                                const double& dt,
                                                const Eigen::Matrix<double, 7, 7>& M,
                                                const Eigen::MatrixXd& jacobian_transpose_pinv,
                                                const Eigen::Matrix<double, 6, 1>& F_cmd) {
  //observed force
  beta_observer = coriolis - M_dot * dq_;
  gamma_observer += (tau_d - beta_observer + r_observer) * dt;
  r_observer = K_observer * (M * dq_ - gamma_observer);   // torque observed
  F_observed = jacobian_transpose_pinv * r_observer /*+ F_cmd*/;   // adds requested force, no clue why seems sus
}
/*
void cartesian_impedance_control::update_observer() {
  F_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state_.O_F_ext_hat_K.data()) * 0.01 + 0.99 * F_last;
  F_last = F_ext;
  delta_F =  (F_contact_des + F_ext); //F_ext shows in the negative direction of F_contact desired
  error_F += dt * delta_F;
  Eigen::Matrix<double, 6, 1> dF_error = (delta_F - delta_F_last) / dt * 0.1 + dF_last * 0.9; //filtered Force error derivative estimate
  dF_last = dF_error;
  delta_F_last = delta_F;
}
*/
// add Sm and Sf by pass by reference
//
void CartesianImpedanceController::adapt_Sm_and_Sf(const int& frame, const Eigen::Matrix<double, 6, 1>& F_contact_target, const Eigen::Matrix<double,3,3>& rot_matrix, 
                        Eigen::Matrix<double, 6, 6>& Sm, Eigen::Matrix<double, 6, 6>& Sf){    //update the Sm and Sf matrix
  Sm = Eigen::MatrixXd::Zero(6, 6);
  for (int i =0; i<6; ++i){
    if (F_contact_target(i)==0){
      //Eigen::Matrix<double, 6, 6> test;
      //test.topLeftCorner(3,3) = Eigen::Matrixd::Identity(3,3);
      Sm(i,i) = 1;
    }
  }
  
  Sf = IDENTITY - Sm;

  if (frame == 2){
    Eigen::Matrix<double, 3, 3> Sm_top_left = Sm.topLeftCorner(3,3);
    Eigen::Matrix<double, 3, 3> Sm_bottom_right = Sm.bottomRightCorner(3,3);
    Eigen::Matrix<double, 3, 3> Identity_3x3 = Eigen::MatrixXd::Identity(3, 3);
    
    // Calculations according to robot dynamics script chapter 3.9.4 Operational Space Control. The matrices had to to be inverted (= transposed to get it running).
    Sm.topLeftCorner(3,3) = rot_matrix.transpose() * Sm_top_left * rot_matrix;
    Sm.bottomRightCorner(3,3) = rot_matrix.transpose() * Sm_bottom_right * rot_matrix;
    Sf.topLeftCorner(3,3) = rot_matrix.transpose() * (Identity_3x3 - Sm_top_left) * rot_matrix;  
    Sf.bottomRightCorner(3,3) = rot_matrix.transpose() * (Identity_3x3 - Sm_bottom_right) * rot_matrix;
  }

  
}

//make current position desired position if the requested force is changed
void CartesianImpedanceController::change_desired_position_if_force_changes(const Eigen::Quaterniond orientation, const Eigen::Vector3d position, const Eigen::Matrix<double, 6, 1>& F_contact_target, 
                        Eigen::Matrix<double, 6, 1>& F_request_old, Eigen::Vector3d& position_d_, Eigen::Quaterniond& orientation_d_, 
                        Eigen::Vector3d& position_d_target_, Eigen::Vector3d& rotation_d_target_/*, bool& keepprinting*/){
  for (int i =0; i<6; ++i){
    if (F_contact_target(i)!= F_request_old(i)){
      position_d_ = position;
      position_d_target_ = position;
      orientation_d_ = orientation;
      Eigen::Matrix3d rot_Matrix_to_transform_to_euler_angles = orientation.toRotationMatrix();
      rotation_d_target_ = rot_Matrix_to_transform_to_euler_angles.eulerAngles(0,1,2);
      /*
      if(F_request_old.norm()!=0){    //was used for debugging
        keepprinting = false;
      }*/
      F_request_old = F_contact_target;
      break;
    }
  }
}


//not sure wheter it is worth to write an extra funciton for this, apparently this is needed because ROS2 makes mimimi
void CartesianImpedanceController::get_rid_of_friction_force(const Eigen::MatrixXd& jacobian_transpose_pinv, 
                                                            const Eigen::Matrix<double, 7, 1>& tau_friction, 
                                                            const Eigen::Matrix<double, 6, 1>& O_F_ext_hat_K_M,
                                                            Eigen::Matrix<double, 6, 1>& O_F_ext_hat_K_M_no_friction){
  O_F_ext_hat_K_M_no_friction = O_F_ext_hat_K_M - jacobian_transpose_pinv * tau_friction;           // friction compensation of measurement
 
}


// should stop the robot from moving when his EE y-position is >0.6 or <-0.6 or he goes higher in z direction than expected and stops robot from crashing into wall with EE
// Currently it does the crash the controller since the change in requested velocity is often to high and exceeds the limits set to the robot
void CartesianImpedanceController::checklimits(const Eigen::Vector3d& pos, Eigen::Matrix<double, 7, 1>& tau_d){
  /*if(std::abs(pos(1))>0.6 or pos(0)<0 or pos(2)>0.8){
    //std::cout << " std::abs(pos(1))> 0.45 yippie " << std::endl;
    tau_d.setZero();
    //D = lkjllka                                                         //////////add this to make controller more safe
    //std::cout << tau_d << std::endl;
    std::cout << "Controller reached predefined limits" << std::endl;
  }
  else{
    //std::cout << " std::abs(pos(1))< 0.45 " << std::endl;
  }*/
}



//Calculates the friction forces acting on the robot's joints depending on joint rotational speed.
//Exerts torque up to a certain empirically detected static friction threshold. 
//TODO: Afterwards, goes into the viscous domain and follows a linear raise depending on empiric parameters
void CartesianImpedanceController::calculate_tau_friction(const Eigen::Matrix<double, 7, 1>& dq_, const Eigen::Matrix<double, 7, 1>& tau_impedance,
                                                          const Eigen::Matrix<double, 6, 7>& jacobian, const Eigen::Matrix<double, 6, 6>& Sm,
                                                          const Eigen::Matrix<double, 6, 1>& error, const Eigen::MatrixXd& jacobian_pinv){
    double alpha = 0.01;//constant for exponential filter in relation to static friction moment        
    dq_filtered = alpha* dq_ + (1 - alpha) * dq_filtered; //Filtering dq_ of every joint
    tau_impedance_filtered = alpha*tau_impedance + (1 - alpha) * tau_impedance_filtered; //Filtering tau_impedance
    //Creating and filtering a "fake" tau_impedance with own weights, optimized for friction compensation (else friction compensation would get stronger with higher stiffnesses)
    tau_friction_impedance = jacobian.transpose() * Sm * (-alpha * (D_friction*(jacobian*dq_) + K_friction * error)) + (1 - alpha) * tau_friction_impedance;
    //Creating "fake" dq_, that acts only in the impedance-space, else dq_ in the nullspace also gets compensated, which we do not want due to null-space movement
    N = (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian); //Nullspace matrix 
    dq_imp = dq_filtered - N * dq_filtered;

    //Calculation of friction force according to Bachelor Thesis of Viktor Luba: https://polybox.ethz.ch/index.php/s/iYj8ALPijKTAC2z?path=%2FFriction%20compensation
    f = beta.cwiseProduct(dq_imp) + offset_friction;
    g(4) = (coulomb_friction(4) + (static_friction(4) - coulomb_friction(4)) * exp(-1 * std::abs(dq_imp(4)/dq_s(4))));                    // this lines was never tested before
    g(6) = (coulomb_friction(6) + (static_friction(6) - coulomb_friction(6)) * exp(-1 * std::abs(dq_imp(6)/dq_s(6))));                    // this lines was never tested before
    dz = dq_imp.array() - dq_imp.array().abs() / g.array() * sigma_0.array() * z.array() + 0.025* tau_friction_impedance.array()/*(jacobian.transpose() * K * error).array()*/;
    dz(6) -= 0.02*tau_friction_impedance(6);                                             // k is everywhere 0.025 excet for the last one that's why we do this wired move (last one is only 0.005 thats why we have wired differnce here)
    z = 0.001 * dz + z;
    tau_friction = sigma_0.array() * z.array() + 100 * sigma_1.array() * dz.array() + f.array();  //*100 because values in vector are only 1% of real values
}


/////////////////////////////////////////////////////////////////////////wieso hier kein pass-by-reference??????  Ist das der Grund wieso Curdins Controller im Mode 2 nicht geht?
void CartesianImpedanceController::update_stiffness_and_references(Eigen::Matrix<double, 6, 1>& F_contact_des){                                   // filtering to make step response not over agressive
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/            // selber entscheiden ob es rein soll// nicht so wichtig
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);                           // slerp calculates the shortest way to reach desired orient.
  // get Input from server
  F_contact_des = 0.05 * F_contact_target + 0.95 * F_contact_des;                                         // individual filtering is being done here, replace by a second filter parameter and add defintion in .hpp document
}


void CartesianImpedanceController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{          // das&  am Ende der Matrix ist für pass-by-reference anstatt pass-by-value
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

void CartesianImpedanceController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

// "Eigen::Matrix" is class that can be used to create Matrices                                 // tau_J_d_M is the calculated tau from 1 iteration before
Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(                   // This function ensures that the torques (tau) aren't changed to 
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,                                          // quickly
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d_M[i];
    tau_d_saturated[i] =
    tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);        // the max(min() ) makes sure that absolute value
  }                                                                                             // of difference is bellow delta_tau_max
  return tau_d_saturated;
}


inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {  //the inline before the void is only to improve the codes performance
  double lambda_ = damped ? 0.2 : 0.0;                                                                // damped == true -> lambda_ = 0.2    damped == false --> lambda_ = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);               // this function computes the pseudoInverse of M_   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);                // why not simply 1/(sing_vals(i)+lambda_*lambda_)?

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


controller_interface::InterfaceConfiguration                                                          // creates the Interface configuration aka names all 
CartesianImpedanceController::command_interface_configuration() const {                               // the different joints and stores them in a vector 
  controller_interface::InterfaceConfiguration config;                                                // and sets the interface type to individual
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");    // adds names for position and velocity for each joint
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);                                     // adds other interfaces to the config vector
    std::cout << franka_robot_model_name << std::endl;
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;                      // why is this string needed? --> probably somewhere else needed

  return state_interfaces_config;
}


CallbackReturn CartesianImpedanceController::on_init() {
   UserInputServer input_server_obj(&position_d_target_,& rotation_d_target_, /*&gripper_command,*/ & K,& D,& T);                // creates or starts the server aka calls constructor of Input server
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();                                                                                 // disconnect thread so other thread mustn't wait for 
   // the 3 lines below should start force_control_server.cpp
   UserInputForceServer input_server_obj2(&F_contact_target, &frame);                
   std::thread input_thread2(&UserInputForceServer::main, input_server_obj2, 0, nullptr);
   input_thread2.detach();

   return CallbackReturn::SUCCESS;                                                                        // this to be finished (it will be in spin so this is important)
}


CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(                   // std::make_unique creates idiotproof pointer
  franka_semantic_components::FrankaRobotModel(robot_name_ + "/" + k_robot_model_interface_name,
                                               robot_name_ + "/" + k_robot_state_interface_name));
                                               
  try {                                                           //if try has an error it does catch instead
    rclcpp::QoS qos_profile(1); // Depth of the message queue aka does only keep one message all older ones get deleted
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    franka_state_subscriber = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(        // creates publisher node
    "franka_robot_state_broadcaster/robot_state", qos_profile, 
    std::bind(&CartesianImpedanceController::topic_callback, this, std::placeholders::_1));
    std::cout << "Succesfully subscribed to robot_state_broadcaster" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }
  
  try {                                                           //if try has an error it does catch instead
    rclcpp::QoS qos_profile2(1); // Depth of the message queue aka does only keep one message all older ones get deleted
    qos_profile2.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    wrench_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Wrench>("wrench_topic", qos_profile2);
    std::cout << "Succesfully publishing F_ext" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher for F_ext creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }
  /*        //commented because I will try to remove that part from the controller
  try {                                                           //if try has an error it does catch instead
    rclcpp::QoS qos_profile3(1); // Depth of the message queue aka does only keep one message all older ones get deleted
    qos_profile3.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    gripper_command_publisher_ = get_node()->create_publisher<std_msgs::msg::String>("gripper_command", qos_profile3);
    std::cout << "Succesfully publishing gripper commands" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher for gripper command creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }
  */
   /* ////////////////////////////////////////////////////////////////////////////////////// 
  try {                                                           //set up for gripper must be executed when the gripper gets exchanged
    gripper_.homing();
    //gripper_.move(0.04, gripper_speed);
    
    std::cout<< "this has worked, we should see me once only"<< std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during homing of the gripper in on_configure with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    } */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////here listens to a topic (initialized publisher)
  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_activate(                                                 // skiped for now, because CallbackReturn seems to be not
  const rclcpp_lifecycle::State& /*previous_state*/) {                                                    // not important for me
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);  // seems to get the initial states when the controller gets started
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  std::cout << "Completed Activation process" << std::endl;
  /*      // basic version of this is done in .hpp file
  try {                                                                     // should initialize gripper
    gripper_ = std::make_unique<franka::Gripper>("192.168.1.200");
  } catch (const franka::Exception& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize gripper: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  } */
  
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(                       // skiped for now, because CallbackReturn seems to be not
  const rclcpp_lifecycle::State& /*previous_state*/) {                                                  // important for me
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

std::array<double, 6> CartesianImpedanceController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {  // assumption that this just changes the type
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}

void CartesianImpedanceController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {      // measures the external force 
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
}

void CartesianImpedanceController::updateJointStates() {                                                                // update states of position (q) and
  for (auto i = 0; i < num_joints; ++i) {                                                                               // velocity (dq) of each joint
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {          // 7 DOF robot  
    std::array<double, 49> mass = franka_robot_model_->getMassMatrix();                                                                      // 7x7 mass matrix
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);  // zero frame = base frame  // 6x7 Jacobian (6 coordinates, 7 DOFs)
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);                                           // T-Matrix (4x4)
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose.data()));
  Eigen::Vector3d position(transform.translation());                                                              // creates fancier T-Matrix
  Eigen::Quaterniond orientation(transform.rotation());
  M_dot = (M - M_old) * dt;
  M_old = M;

  geometry_msgs::msg::Wrench wrench_msg;
  Eigen::Matrix<double,3,3> rotation_matrix = transform.rotation();                                    
  
  updateJointStates();
  adapt_Sm_and_Sf(frame, F_contact_target,rotation_matrix, Sm, Sf);
  change_desired_position_if_force_changes(orientation, position, F_contact_target, F_request_old, position_d_, orientation_d_, position_d_target_, rotation_d_target_ /*, keepprinting*/);
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());                     // mimics a quaternion according to documentation in Eigen
  
  //////////////////////////////////////////// multiple /* */ present here
  /*
  if (outcounter % (1000/update_frequency) == 0 *//*and run*//*){
  //   std::cout << "Sm calculated"<< std::endl;
  //   std::cout << Sm << std::endl;
  //   std::cout << "--------" << std::endl;
  //   std::cout << "Sf calculated"<< std::endl;
  //   std::cout << Sf << std::endl;
  // std::cout << "--------" << std::endl;
  // std::cout << "I_error at start of loop" << std::endl;
  // std::cout << I_error << std::endl;
  std::cout << "--------" << std::endl;
  std::cout << "position_d_" << std::endl;
  std::cout << position_d_ << std::endl;
  std::cout << "--------" << std::endl;
  std::cout << "position_d_target_" << std::endl;
  std::cout << position_d_target_ << std::endl;
  std::cout << "--------" << std::endl;
  std::cout << "orientation_d_" << std::endl;
  std::cout << orientation_d_ << std::endl;
  std::cout << "--------" << std::endl;
  std::cout << "orientation_d_target_" << std::endl;
  std::cout << orientation_d_target_ << std::endl;
  std::cout << "--------" << std::endl;
  std::cout << "rotation_d_target_" << std::endl;
  std::cout << rotation_d_target_ << std::endl;
  std::cout << "--------" << std::endl;
  }
  */
  /*  //was only used for debugging
  if (!keepprinting and out_store){
    outcounter_store = outcounter;
    out_store = false;
  }
  if (!keepprinting and !out_store and (outcounter-outcounter_store)>1000){
    run = false;
  }*/

  

  
  error.head(3) << position - position_d_;

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {                                                  // think graphically about the dot product. Quaternions are just a scaled version of the rotation vector
    orientation.coeffs() << -orientation.coeffs();                                                                // fixes the issue that quaternion = - quaternion
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);                                    // calculates the needed rotation to get to desired orientation
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);     // transform to base frame                          // transform.rotation gets the rotation matrix
  I_error += Sm * dt * integrator_weights.cwiseProduct(error);                                                    // I = integrator von PID .cwiseProduct = coefficient-wise product
  for (int i = 0; i < 6; i++){
    I_error(i,0) = std::min(std::max(-max_I(i,0),  I_error(i,0)), max_I(i,0));                                     // saturate integrater error to no get overagressive
  }

  Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();                                             // Robot Dynamics Scipt S.79  inertia of robot
  // Theta = T*Lambda;                                                                                            // virtual scalling of the impedance (Trägheit)
  // F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
  //Inertia of the robot
  switch (mode_)                                                                                                  // mode is set to 1 so position control
  {
  case 1:
    Theta = Lambda;
    F_impedance = -1 * (D * (jacobian * dq_) + K * error /*+ I_error*/);
    break;

  case 2:
    Theta = T*Lambda;
    F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
    break;
  
  default:
    break;
  }

  // Definiton added in .hpp file
  //Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);                                                 // computes the pseudo inverse and stores it in second variable
  pseudoInverse(jacobian, jacobian_pinv);                                                           //needed for friction compensation
  calculate_tau_friction(dq_, tau_impedance, jacobian, Sm, error, jacobian_pinv); 

  get_rid_of_friction_force(jacobian_transpose_pinv, tau_friction, O_F_ext_hat_K_M, O_F_ext_hat_K_M_no_friction);
  F_ext = /*0.999 * F_ext + 0.001 */ /*O_F_ext_hat_K_M*/  O_F_ext_hat_K_M_no_friction; //Filtering       //the second part is the measured external force                                               // exponential filtering with smoothing factor = 0.1
  
  if(frame != frame_before){    //reset Integrator if frame gets changed while the controller is running
    I_F_error = 0;
    frame_before = frame;
  }
  
  switch (frame){
    case 1:  
      I_F_error += dt * Sf *(F_contact_des - F_ext);
      F_cmd = Sf *0.4 *(F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des;     //////////////////////////////                      // P + I + feedforward term for faster convergence
      break;
    case 2:
      F_ext_EE = rotation_matrix * F_ext;
      Sf_EE = IDENTITY;
      for (int j =0; j<6; ++j){
        if (F_contact_target(j)==0){
          Sf_EE(j,j) = 0;
        }
      }
      I_F_error += dt * Sf_EE *(F_contact_des - F_ext_EE);
      F_cmd = Sf_EE *0.4 *(F_contact_des - F_ext_EE) + 0.9 * I_F_error + 0.9 * F_contact_des;
      
      break;
    default:
      std::cout<< "Unknown frame \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n";
      break;
  }
  //Ist das in welchem frame oben dran.
  //change_wrench_to_base_frame(F_cmd, rotation_matrix, position);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *                                           /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////here nullspace matrix gets calculated
                    (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //if config_control = true we control the whole robot configuration
                    (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config_control ) false we don't care about the joint position, currently set to false.

  tau_impedance = jacobian.transpose() * Sm * (F_impedance /*+ F_repulsion + F_potential*/) + jacobian.transpose() * Sf * F_cmd;    // combine force control and impedance control (this is where the magic happens)
  auto tau_d_placeholder = tau_impedance + tau_nullspace + coriolis + tau_friction; //add nullspace and coriolis components to desired torque. Gets rid of part of tau_impedance that is only there because of friction
  tau_d << tau_d_placeholder;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;                                                                                            // store this value for saturateTorqueRate function in 
                                                                                                                // next iteration
  
  if (outcounter % (1000/update_frequency) == 0 /*and run*/){
    // std::cout << "F_contact_des [N]" << std::endl;
    // std::cout << F_contact_des << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_ext [N]" << std::endl;
    // std::cout << F_ext << std::endl;
    // std::cout << "--------" << std::endl;
    // F:cout << error << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_cmd [N]" << std::endl;
    // std::cout << F_cmd << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "O_F_ext_hat_K [N]" << std::endl;
    // std::cout << O_F_ext_hat_K << std::endl;
    // std::cout << "O_F_ext_hat_K_M [N]" << std::endl;
    // std::cout << O_F_ext_hat_K_M << std::endl;
    // std::cout << "norm = ";
    // std::cout << O_F_ext_hat_K_M.norm() << std::endl;
    // std::cout << "jacobian_transpose_pinv" << std::endl;
    // std::cout << jacobian_transpose_pinv << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "O_F_ext_hat_K_M_no_friction [N]" << std::endl;
    // std::cout << O_F_ext_hat_K_M_no_friction << std::endl;
    // std::cout << "Lambda  Thetha.inv(): " << std::endl;
    // std::cout << Lambda*Theta.inverse() << std::endl;
    // std::cout << "tau_d" << std::endl;
    // std::cout << tau_d << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_ext [N]" << std::endl;
    // std::cout << F_ext << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "norm = ";
    // std::cout << F_ext.norm() << std::endl;
    // std::cout << tau_nullspace << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_cmd"<< std::endl;
    // std::cout << F_cmd << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "Sm should be diagonal with Values 1"<< std::endl;
    // std::cout << Sm << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "Sf should be zero"<< std::endl;
    // std::cout << Sf << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_contact_target [N]"<< std::endl;
    // std::cout << F_contact_target << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_contact_des [N]"<< std::endl;
    // std::cout << F_contact_des << std::endl;
    // std::cout << "--------" << std::endl;
    std::cout << "--------" << std::endl;
    // std::cout << tau_impedance << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << coriolis << std::endl;
    // std::cout << "Inertia scaling [m]: " << std::endl;
    // std::cout << T << std::endl;
    std::cout << "Position [m]: " << std::endl;
    std::cout << position << std::endl;
    // std::cout << "Tau_d [m]: " << std::endl;
    // std::cout << tau_d << std::endl;
  }
  
  //never comment the line bellow, instead comment parts of the funtion definition
  checklimits(position, tau_d);
    
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));                                                                 // give computed values to the robot or the controller  
  }

  // publishes observed external force to topic
  wrench_msg.force.x = F_observed[0];
  wrench_msg.force.y = F_observed[1];
  wrench_msg.force.z = F_observed[2];
  wrench_msg.torque.x = F_observed[3];
  wrench_msg.torque.y = F_observed[4];
  wrench_msg.torque.z = F_observed[5];
  wrench_publisher_->publish(wrench_msg);
  

  outcounter++;
  update_stiffness_and_references(F_contact_des);                                                                            // update target position and resistances against movement
  update_forces(F_observed, beta_observer, gamma_observer, r_observer, K_observer, coriolis, M_dot, dq_,
                tau_d, dt, M, jacobian_transpose_pinv, F_cmd);
  //move_gripper(gripper_, gripper_command, gripper_speed);
  //gripper_.move(0.04, gripper_speed);          //just here for testing
  //gripper_.grasp(0.06, gripper_speed,50);       //distance for sanding block
  return controller_interface::return_type::OK;
}
}

// namespace cartesian_impedance_control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cartesian_impedance_control::CartesianImpedanceController,
                       controller_interface::ControllerInterface)