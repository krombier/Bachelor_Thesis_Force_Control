// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages_fr3:srv/SetParam.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_PARAM__BUILDER_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_PARAM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "messages_fr3/srv/detail/set_param__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetParam_Request_f
{
public:
  explicit Init_SetParam_Request_f(::messages_fr3::srv::SetParam_Request & msg)
  : msg_(msg)
  {}
  ::messages_fr3::srv::SetParam_Request f(::messages_fr3::srv::SetParam_Request::_f_type arg)
  {
    msg_.f = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetParam_Request msg_;
};

class Init_SetParam_Request_e
{
public:
  explicit Init_SetParam_Request_e(::messages_fr3::srv::SetParam_Request & msg)
  : msg_(msg)
  {}
  Init_SetParam_Request_f e(::messages_fr3::srv::SetParam_Request::_e_type arg)
  {
    msg_.e = std::move(arg);
    return Init_SetParam_Request_f(msg_);
  }

private:
  ::messages_fr3::srv::SetParam_Request msg_;
};

class Init_SetParam_Request_d
{
public:
  explicit Init_SetParam_Request_d(::messages_fr3::srv::SetParam_Request & msg)
  : msg_(msg)
  {}
  Init_SetParam_Request_e d(::messages_fr3::srv::SetParam_Request::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_SetParam_Request_e(msg_);
  }

private:
  ::messages_fr3::srv::SetParam_Request msg_;
};

class Init_SetParam_Request_c
{
public:
  explicit Init_SetParam_Request_c(::messages_fr3::srv::SetParam_Request & msg)
  : msg_(msg)
  {}
  Init_SetParam_Request_d c(::messages_fr3::srv::SetParam_Request::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_SetParam_Request_d(msg_);
  }

private:
  ::messages_fr3::srv::SetParam_Request msg_;
};

class Init_SetParam_Request_b
{
public:
  explicit Init_SetParam_Request_b(::messages_fr3::srv::SetParam_Request & msg)
  : msg_(msg)
  {}
  Init_SetParam_Request_c b(::messages_fr3::srv::SetParam_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_SetParam_Request_c(msg_);
  }

private:
  ::messages_fr3::srv::SetParam_Request msg_;
};

class Init_SetParam_Request_a
{
public:
  Init_SetParam_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetParam_Request_b a(::messages_fr3::srv::SetParam_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_SetParam_Request_b(msg_);
  }

private:
  ::messages_fr3::srv::SetParam_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetParam_Request>()
{
  return messages_fr3::srv::builder::Init_SetParam_Request_a();
}

}  // namespace messages_fr3


namespace messages_fr3
{

namespace srv
{

namespace builder
{

class Init_SetParam_Response_success
{
public:
  Init_SetParam_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::messages_fr3::srv::SetParam_Response success(::messages_fr3::srv::SetParam_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages_fr3::srv::SetParam_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages_fr3::srv::SetParam_Response>()
{
  return messages_fr3::srv::builder::Init_SetParam_Response_success();
}

}  // namespace messages_fr3

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_PARAM__BUILDER_HPP_
