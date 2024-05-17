// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages_fr3:srv/SetParam.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_PARAM__TRAITS_HPP_
#define MESSAGES_FR3__SRV__DETAIL__SET_PARAM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "messages_fr3/srv/detail/set_param__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace messages_fr3
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetParam_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << ", ";
  }

  // member: c
  {
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << ", ";
  }

  // member: d
  {
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << ", ";
  }

  // member: e
  {
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << ", ";
  }

  // member: f
  {
    out << "f: ";
    rosidl_generator_traits::value_to_yaml(msg.f, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetParam_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << "\n";
  }

  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << "\n";
  }

  // member: e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << "\n";
  }

  // member: f
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f: ";
    rosidl_generator_traits::value_to_yaml(msg.f, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetParam_Request & msg, bool use_flow_style = false)
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
  const messages_fr3::srv::SetParam_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  messages_fr3::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use messages_fr3::srv::to_yaml() instead")]]
inline std::string to_yaml(const messages_fr3::srv::SetParam_Request & msg)
{
  return messages_fr3::srv::to_yaml(msg);
}

template<>
inline const char * data_type<messages_fr3::srv::SetParam_Request>()
{
  return "messages_fr3::srv::SetParam_Request";
}

template<>
inline const char * name<messages_fr3::srv::SetParam_Request>()
{
  return "messages_fr3/srv/SetParam_Request";
}

template<>
struct has_fixed_size<messages_fr3::srv::SetParam_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages_fr3::srv::SetParam_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages_fr3::srv::SetParam_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace messages_fr3
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetParam_Response & msg,
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
  const SetParam_Response & msg,
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

inline std::string to_yaml(const SetParam_Response & msg, bool use_flow_style = false)
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
  const messages_fr3::srv::SetParam_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  messages_fr3::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use messages_fr3::srv::to_yaml() instead")]]
inline std::string to_yaml(const messages_fr3::srv::SetParam_Response & msg)
{
  return messages_fr3::srv::to_yaml(msg);
}

template<>
inline const char * data_type<messages_fr3::srv::SetParam_Response>()
{
  return "messages_fr3::srv::SetParam_Response";
}

template<>
inline const char * name<messages_fr3::srv::SetParam_Response>()
{
  return "messages_fr3/srv/SetParam_Response";
}

template<>
struct has_fixed_size<messages_fr3::srv::SetParam_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages_fr3::srv::SetParam_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages_fr3::srv::SetParam_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages_fr3::srv::SetParam>()
{
  return "messages_fr3::srv::SetParam";
}

template<>
inline const char * name<messages_fr3::srv::SetParam>()
{
  return "messages_fr3/srv/SetParam";
}

template<>
struct has_fixed_size<messages_fr3::srv::SetParam>
  : std::integral_constant<
    bool,
    has_fixed_size<messages_fr3::srv::SetParam_Request>::value &&
    has_fixed_size<messages_fr3::srv::SetParam_Response>::value
  >
{
};

template<>
struct has_bounded_size<messages_fr3::srv::SetParam>
  : std::integral_constant<
    bool,
    has_bounded_size<messages_fr3::srv::SetParam_Request>::value &&
    has_bounded_size<messages_fr3::srv::SetParam_Response>::value
  >
{
};

template<>
struct is_service<messages_fr3::srv::SetParam>
  : std::true_type
{
};

template<>
struct is_service_request<messages_fr3::srv::SetParam_Request>
  : std::true_type
{
};

template<>
struct is_service_response<messages_fr3::srv::SetParam_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_PARAM__TRAITS_HPP_
