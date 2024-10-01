// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages_fr3:srv/SetForce.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_FORCE__BUILDER_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_FORCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "messages_fr3/srv/detail/set_force__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetForce_Request_frame
{
public:
  explicit Init_SetForce_Request_frame(::messages_fr3::srv::SetForce_Request & msg)
  : msg_(msg)
  {}
  ::messages_fr3::srv::SetForce_Request frame(::messages_fr3::srv::SetForce_Request::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Request msg_;
};

class Init_SetForce_Request_z_torque
{
public:
  explicit Init_SetForce_Request_z_torque(::messages_fr3::srv::SetForce_Request & msg)
  : msg_(msg)
  {}
  Init_SetForce_Request_frame z_torque(::messages_fr3::srv::SetForce_Request::_z_torque_type arg)
  {
    msg_.z_torque = std::move(arg);
    return Init_SetForce_Request_frame(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Request msg_;
};

class Init_SetForce_Request_y_torque
{
public:
  explicit Init_SetForce_Request_y_torque(::messages_fr3::srv::SetForce_Request & msg)
  : msg_(msg)
  {}
  Init_SetForce_Request_z_torque y_torque(::messages_fr3::srv::SetForce_Request::_y_torque_type arg)
  {
    msg_.y_torque = std::move(arg);
    return Init_SetForce_Request_z_torque(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Request msg_;
};

class Init_SetForce_Request_x_torque
{
public:
  explicit Init_SetForce_Request_x_torque(::messages_fr3::srv::SetForce_Request & msg)
  : msg_(msg)
  {}
  Init_SetForce_Request_y_torque x_torque(::messages_fr3::srv::SetForce_Request::_x_torque_type arg)
  {
    msg_.x_torque = std::move(arg);
    return Init_SetForce_Request_y_torque(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Request msg_;
};

class Init_SetForce_Request_z_force
{
public:
  explicit Init_SetForce_Request_z_force(::messages_fr3::srv::SetForce_Request & msg)
  : msg_(msg)
  {}
  Init_SetForce_Request_x_torque z_force(::messages_fr3::srv::SetForce_Request::_z_force_type arg)
  {
    msg_.z_force = std::move(arg);
    return Init_SetForce_Request_x_torque(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Request msg_;
};

class Init_SetForce_Request_y_force
{
public:
  explicit Init_SetForce_Request_y_force(::messages_fr3::srv::SetForce_Request & msg)
  : msg_(msg)
  {}
  Init_SetForce_Request_z_force y_force(::messages_fr3::srv::SetForce_Request::_y_force_type arg)
  {
    msg_.y_force = std::move(arg);
    return Init_SetForce_Request_z_force(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Request msg_;
};

class Init_SetForce_Request_x_force
{
public:
  Init_SetForce_Request_x_force()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetForce_Request_y_force x_force(::messages_fr3::srv::SetForce_Request::_x_force_type arg)
  {
    msg_.x_force = std::move(arg);
    return Init_SetForce_Request_y_force(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetForce_Request>()
{
  return messages_fr3::srv::builder::Init_SetForce_Request_x_force();
}

}  // namespace messages_fr3


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetForce_Response_success
{
public:
  Init_SetForce_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::messages_fr3::srv::SetForce_Response success(::messages_fr3::srv::SetForce_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetForce_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetForce_Response>()
{
  return messages_fr3::srv::builder::Init_SetForce_Response_success();
}

}  // namespace messages_fr3

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_FORCE__BUILDER_HPP_
