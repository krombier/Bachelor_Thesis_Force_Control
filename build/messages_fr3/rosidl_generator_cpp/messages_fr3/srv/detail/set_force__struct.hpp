// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages_fr3:srv/SetForce.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_FORCE__STRUCT_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_FORCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__messages_fr3__srv__SetForce_Request __attribute__((deprecated))
#else
# define DEPRECATED__messages_fr3__srv__SetForce_Request __declspec(deprecated)
#endif

namespace messages_fr3
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetForce_Request_
{
  using Type = SetForce_Request_<ContainerAllocator>;

  explicit SetForce_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_force = 0.0;
      this->y_force = 0.0;
      this->z_force = 0.0;
      this->x_torque = 0.0;
      this->y_torque = 0.0;
      this->z_torque = 0.0;
    }
  }

  explicit SetForce_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_force = 0.0;
      this->y_force = 0.0;
      this->z_force = 0.0;
      this->x_torque = 0.0;
      this->y_torque = 0.0;
      this->z_torque = 0.0;
    }
  }

  // field types and members
  using _x_force_type =
    double;
  _x_force_type x_force;
  using _y_force_type =
    double;
  _y_force_type y_force;
  using _z_force_type =
    double;
  _z_force_type z_force;
  using _x_torque_type =
    double;
  _x_torque_type x_torque;
  using _y_torque_type =
    double;
  _y_torque_type y_torque;
  using _z_torque_type =
    double;
  _z_torque_type z_torque;

  // setters for named parameter idiom
  Type & set__x_force(
    const double & _arg)
  {
    this->x_force = _arg;
    return *this;
  }
  Type & set__y_force(
    const double & _arg)
  {
    this->y_force = _arg;
    return *this;
  }
  Type & set__z_force(
    const double & _arg)
  {
    this->z_force = _arg;
    return *this;
  }
  Type & set__x_torque(
    const double & _arg)
  {
    this->x_torque = _arg;
    return *this;
  }
  Type & set__y_torque(
    const double & _arg)
  {
    this->y_torque = _arg;
    return *this;
  }
  Type & set__z_torque(
    const double & _arg)
  {
    this->z_torque = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages_fr3::srv::SetForce_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages_fr3::srv::SetForce_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetForce_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetForce_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages_fr3__srv__SetForce_Request
    std::shared_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages_fr3__srv__SetForce_Request
    std::shared_ptr<messages_fr3::srv::SetForce_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetForce_Request_ & other) const
  {
    if (this->x_force != other.x_force) {
      return false;
    }
    if (this->y_force != other.y_force) {
      return false;
    }
    if (this->z_force != other.z_force) {
      return false;
    }
    if (this->x_torque != other.x_torque) {
      return false;
    }
    if (this->y_torque != other.y_torque) {
      return false;
    }
    if (this->z_torque != other.z_torque) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetForce_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetForce_Request_

// alias to use template instance with default allocator
using SetForce_Request =
  messages_fr3::srv::SetForce_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace messages_fr3


#ifndef _WIN32
# define DEPRECATED__messages_fr3__srv__SetForce_Response __attribute__((deprecated))
#else
# define DEPRECATED__messages_fr3__srv__SetForce_Response __declspec(deprecated)
#endif

namespace messages_fr3
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetForce_Response_
{
  using Type = SetForce_Response_<ContainerAllocator>;

  explicit SetForce_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetForce_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    messages_fr3::srv::SetForce_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages_fr3::srv::SetForce_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetForce_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages_fr3::srv::SetForce_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages_fr3__srv__SetForce_Response
    std::shared_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages_fr3__srv__SetForce_Response
    std::shared_ptr<messages_fr3::srv::SetForce_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetForce_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetForce_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetForce_Response_

// alias to use template instance with default allocator
using SetForce_Response =
  messages_fr3::srv::SetForce_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace messages_fr3

namespace messages_fr3
{

namespace srv
{

struct SetForce
{
  using Request = messages_fr3::srv::SetForce_Request;
  using Response = messages_fr3::srv::SetForce_Response;
};

}  // namespace srv

}  // namespace messages_fr3

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_FORCE__STRUCT_HPP_
