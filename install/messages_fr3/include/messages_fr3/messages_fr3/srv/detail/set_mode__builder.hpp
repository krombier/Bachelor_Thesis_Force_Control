// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages_fr3:srv/SetMode.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_MODE__BUILDER_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "messages_fr3/srv/detail/set_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetMode_Request_mode
{
public:
  Init_SetMode_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::messages_fr3::srv::SetMode_Request mode(::messages_fr3::srv::SetMode_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetMode_Request>()
{
  return messages_fr3::srv::builder::Init_SetMode_Request_mode();
}

}  // namespace messages_fr3


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetMode_Response_success
{
public:
  Init_SetMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::messages_fr3::srv::SetMode_Response success(::messages_fr3::srv::SetMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetMode_Response>()
{
  return messages_fr3::srv::builder::Init_SetMode_Response_success();
}

}  // namespace messages_fr3

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_MODE__BUILDER_HPP_
