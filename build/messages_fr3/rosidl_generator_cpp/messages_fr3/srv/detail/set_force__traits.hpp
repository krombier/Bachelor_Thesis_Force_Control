// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages_fr3:srv/SetForce.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_FORCE__TRAITS_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_FORCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "messages_fr3/srv/detail/set_force__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace messages_fr3
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetForce_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_force
  {
    out << "x_force: ";
    rosidl_generator_traits::value_to_yaml(msg.x_force, out);
    out << ", ";
  }

  // member: y_force
  {
    out << "y_force: ";
    rosidl_generator_traits::value_to_yaml(msg.y_force, out);
    out << ", ";
  }

  // member: z_force
  {
    out << "z_force: ";
    rosidl_generator_traits::value_to_yaml(msg.z_force, out);
    out << ", ";
  }

  // member: x_torque
  {
    out << "x_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.x_torque, out);
    out << ", ";
  }

  // member: y_torque
  {
    out << "y_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.y_torque, out);
    out << ", ";
  }

  // member: z_torque
  {
    out << "z_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.z_torque, out);
    out << ", ";
  }

  // member: frame
  {
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetForce_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_force: ";
    rosidl_generator_traits::value_to_yaml(msg.x_force, out);
    out << "\n";
  }

  // member: y_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_force: ";
    rosidl_generator_traits::value_to_yaml(msg.y_force, out);
    out << "\n";
  }

  // member: z_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_force: ";
    rosidl_generator_traits::value_to_yaml(msg.z_force, out);
    out << "\n";
  }

  // member: x_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.x_torque, out);
    out << "\n";
  }

  // member: y_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.y_torque, out);
    out << "\n";
  }

  // member: z_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.z_torque, out);
    out << "\n";
  }

  // member: frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetForce_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace messages_fr3

namespace rosidl_generator_traits
{

[[deprecated("use messages_fr3::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const messages_fr3::srv::SetForce_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  messages_fr3::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use messages_fr3::srv::to_yaml() instead")]]
inline std::string to_yaml(const messages_fr3::srv::SetForce_Request & msg)
{
  return messages_fr3::srv::to_yaml(msg);
}

template<>
inline const char * data_type<messages_fr3::srv::SetForce_Request>()
{
  return "messages_fr3::srv::SetForce_Request";
}

template<>
inline const char * name<messages_fr3::srv::SetForce_Request>()
{
  return "messages_fr3/srv/SetForce_Request";
}

template<>
struct has_fixed_size<messages_fr3::srv::SetForce_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages_fr3::srv::SetForce_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages_fr3::srv::SetForce_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace messages_fr3
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetForce_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetForce_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetForce_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace messages_fr3

namespace rosidl_generator_traits
{

[[deprecated("use messages_fr3::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const messages_fr3::srv::SetForce_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  messages_fr3::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use messages_fr3::srv::to_yaml() instead")]]
inline std::string to_yaml(const messages_fr3::srv::SetForce_Response & msg)
{
  return messages_fr3::srv::to_yaml(msg);
}

template<>
inline const char * data_type<messages_fr3::srv::SetForce_Response>()
{
  return "messages_fr3::srv::SetForce_Response";
}

template<>
inline const char * name<messages_fr3::srv::SetForce_Response>()
{
  return "messages_fr3/srv/SetForce_Response";
}

template<>
struct has_fixed_size<messages_fr3::srv::SetForce_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages_fr3::srv::SetForce_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages_fr3::srv::SetForce_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages_fr3::srv::SetForce>()
{
  return "messages_fr3::srv::SetForce";
}

template<>
inline const char * name<messages_fr3::srv::SetForce>()
{
  return "messages_fr3/srv/SetForce";
}

template<>
struct has_fixed_size<messages_fr3::srv::SetForce>
  : std::integral_constant<
    bool,
    has_fixed_size<messages_fr3::srv::SetForce_Request>::value &&
    has_fixed_size<messages_fr3::srv::SetForce_Response>::value
  >
{
};

template<>
struct has_bounded_size<messages_fr3::srv::SetForce>
  : std::integral_constant<
    bool,
    has_bounded_size<messages_fr3::srv::SetForce_Request>::value &&
    has_bounded_size<messages_fr3::srv::SetForce_Response>::value
  >
{
};

template<>
struct is_service<messages_fr3::srv::SetForce>
  : std::true_type
{
};

template<>
struct is_service_request<messages_fr3::srv::SetForce_Request>
  : std::true_type
{
};

template<>
struct is_service_response<messages_fr3::srv::SetForce_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_FORCE__TRAITS_HPP_
