// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages_fr3:srv/SetParam.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_PARAM__STRUCT_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_PARAM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__messages_fr3__srv__SetParam_Request __attribute__((deprecated))
#else
# define DEPRECATED__messages_fr3__srv__SetParam_Request __declspec(deprecated)
#endif

namespace messages_fr3
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetParam_Request_
{
  using Type = SetParam_Request_<ContainerAllocator>;

  explicit SetParam_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0.0;
      this->b = 0.0;
      this->c = 0.0;
      this->d = 0.0;
      this->e = 0.0;
      this->f = 0.0;
    }
  }

  explicit SetParam_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0.0;
      this->b = 0.0;
      this->c = 0.0;
      this->d = 0.0;
      this->e = 0.0;
      this->f = 0.0;
    }
  }

  // field types and members
  using _a_type =
    double;
  _a_type a;
  using _b_type =
    double;
  _b_type b;
  using _c_type =
    double;
  _c_type c;
  using _d_type =
    double;
  _d_type d;
  using _e_type =
    double;
  _e_type e;
  using _f_type =
    double;
  _f_type f;

  // setters for named parameter idiom
  Type & set__a(
    const double & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const double & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__c(
    const double & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__d(
    const double & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__e(
    const double & _arg)
  {
    this->e = _arg;
    return *this;
  }
  Type & set__f(
    const double & _arg)
  {
    this->f = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages_fr3::srv::SetParam_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages_fr3::srv::SetParam_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetParam_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetParam_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages_fr3__srv__SetParam_Request
    std::shared_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages_fr3__srv__SetParam_Request
    std::shared_ptr<messages_fr3::srv::SetParam_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetParam_Request_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    if (this->e != other.e) {
      return false;
    }
    if (this->f != other.f) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetParam_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetParam_Request_

// alias to use template instance with default allocator
using SetParam_Request =
  messages_fr3::srv::SetParam_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace messages_fr3


#ifndef _WIN32
# define DEPRECATED__messages_fr3__srv__SetParam_Response __attribute__((deprecated))
#else
# define DEPRECATED__messages_fr3__srv__SetParam_Response __declspec(deprecated)
#endif

namespace messages_fr3
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetParam_Response_
{
  using Type = SetParam_Response_<ContainerAllocator>;

  explicit SetParam_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetParam_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages_fr3::srv::SetParam_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages_fr3::srv::SetParam_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetParam_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetParam_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages_fr3__srv__SetParam_Response
    std::shared_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages_fr3__srv__SetParam_Response
    std::shared_ptr<messages_fr3::srv::SetParam_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetParam_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetParam_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetParam_Response_

// alias to use template instance with default allocator
using SetParam_Response =
  messages_fr3::srv::SetParam_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace messages_fr3

namespace messages_fr3
{

namespace srv
{

struct SetParam
{
  using Request = messages_fr3::srv::SetParam_Request;
  using Response = messages_fr3::srv::SetParam_Response;
};

}  // namespace srv

}  // namespace messages_fr3

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_PARAM__STRUCT_HPP_
