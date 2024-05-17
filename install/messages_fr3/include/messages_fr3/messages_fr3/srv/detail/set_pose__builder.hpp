// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages_fr3:srv/SetPose.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_POSE__BUILDER_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "messages_fr3/srv/detail/set_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetPose_Request_yaw
{
public:
  explicit Init_SetPose_Request_yaw(::messages_fr3::srv::SetPose_Request & msg)
  : msg_(msg)
  {}
  ::messages_fr3::srv::SetPose_Request yaw(::messages_fr3::srv::SetPose_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetPose_Request msg_;
};

class Init_SetPose_Request_pitch
{
public:
  explicit Init_SetPose_Request_pitch(::messages_fr3::srv::SetPose_Request & msg)
  : msg_(msg)
  {}
  Init_SetPose_Request_yaw pitch(::messages_fr3::srv::SetPose_Request::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_SetPose_Request_yaw(msg_);
  }

private:
  ::messages_fr3::srv::SetPose_Request msg_;
};

class Init_SetPose_Request_roll
{
public:
  explicit Init_SetPose_Request_roll(::messages_fr3::srv::SetPose_Request & msg)
  : msg_(msg)
  {}
  Init_SetPose_Request_pitch roll(::messages_fr3::srv::SetPose_Request::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_SetPose_Request_pitch(msg_);
  }

private:
  ::messages_fr3::srv::SetPose_Request msg_;
};

class Init_SetPose_Request_z
{
public:
  explicit Init_SetPose_Request_z(::messages_fr3::srv::SetPose_Request & msg)
  : msg_(msg)
  {}
  Init_SetPose_Request_roll z(::messages_fr3::srv::SetPose_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_SetPose_Request_roll(msg_);
  }

private:
  ::messages_fr3::srv::SetPose_Request msg_;
};

class Init_SetPose_Request_y
{
public:
  explicit Init_SetPose_Request_y(::messages_fr3::srv::SetPose_Request & msg)
  : msg_(msg)
  {}
  Init_SetPose_Request_z y(::messages_fr3::srv::SetPose_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_SetPose_Request_z(msg_);
  }

private:
  ::messages_fr3::srv::SetPose_Request msg_;
};

class Init_SetPose_Request_x
{
public:
  Init_SetPose_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPose_Request_y x(::messages_fr3::srv::SetPose_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetPose_Request_y(msg_);
  }

private:
  ::messages_fr3::srv::SetPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetPose_Request>()
{
  return messages_fr3::srv::builder::Init_SetPose_Request_x();
}

}  // namespace messages_fr3


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetPose_Response_success
{
public:
  Init_SetPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::messages_fr3::srv::SetPose_Response success(::messages_fr3::srv::SetPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetPose_Response>()
{
  return messages_fr3::srv::builder::Init_SetPose_Response_success();
}

}  // namespace messages_fr3

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_POSE__BUILDER_HPP_
