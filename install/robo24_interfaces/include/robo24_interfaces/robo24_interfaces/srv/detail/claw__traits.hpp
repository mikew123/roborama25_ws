// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robo24_interfaces:srv/Claw.idl
// generated code does not contain a copyright notice

#ifndef ROBO24_INTERFACES__SRV__DETAIL__CLAW__TRAITS_HPP_
#define ROBO24_INTERFACES__SRV__DETAIL__CLAW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robo24_interfaces/srv/detail/claw__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robo24_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Claw_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pct
  {
    out << "pct: ";
    rosidl_generator_traits::value_to_yaml(msg.pct, out);
    out << ", ";
  }

  // member: msec
  {
    out << "msec: ";
    rosidl_generator_traits::value_to_yaml(msg.msec, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Claw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pct
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pct: ";
    rosidl_generator_traits::value_to_yaml(msg.pct, out);
    out << "\n";
  }

  // member: msec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "msec: ";
    rosidl_generator_traits::value_to_yaml(msg.msec, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Claw_Request & msg, bool use_flow_style = false)
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

}  // namespace robo24_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robo24_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robo24_interfaces::srv::Claw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robo24_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robo24_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robo24_interfaces::srv::Claw_Request & msg)
{
  return robo24_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robo24_interfaces::srv::Claw_Request>()
{
  return "robo24_interfaces::srv::Claw_Request";
}

template<>
inline const char * name<robo24_interfaces::srv::Claw_Request>()
{
  return "robo24_interfaces/srv/Claw_Request";
}

template<>
struct has_fixed_size<robo24_interfaces::srv::Claw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robo24_interfaces::srv::Claw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robo24_interfaces::srv::Claw_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robo24_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Claw_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: resp
  {
    out << "resp: ";
    rosidl_generator_traits::value_to_yaml(msg.resp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Claw_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: resp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resp: ";
    rosidl_generator_traits::value_to_yaml(msg.resp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Claw_Response & msg, bool use_flow_style = false)
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

}  // namespace robo24_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robo24_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robo24_interfaces::srv::Claw_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robo24_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robo24_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robo24_interfaces::srv::Claw_Response & msg)
{
  return robo24_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robo24_interfaces::srv::Claw_Response>()
{
  return "robo24_interfaces::srv::Claw_Response";
}

template<>
inline const char * name<robo24_interfaces::srv::Claw_Response>()
{
  return "robo24_interfaces/srv/Claw_Response";
}

template<>
struct has_fixed_size<robo24_interfaces::srv::Claw_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robo24_interfaces::srv::Claw_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robo24_interfaces::srv::Claw_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robo24_interfaces::srv::Claw>()
{
  return "robo24_interfaces::srv::Claw";
}

template<>
inline const char * name<robo24_interfaces::srv::Claw>()
{
  return "robo24_interfaces/srv/Claw";
}

template<>
struct has_fixed_size<robo24_interfaces::srv::Claw>
  : std::integral_constant<
    bool,
    has_fixed_size<robo24_interfaces::srv::Claw_Request>::value &&
    has_fixed_size<robo24_interfaces::srv::Claw_Response>::value
  >
{
};

template<>
struct has_bounded_size<robo24_interfaces::srv::Claw>
  : std::integral_constant<
    bool,
    has_bounded_size<robo24_interfaces::srv::Claw_Request>::value &&
    has_bounded_size<robo24_interfaces::srv::Claw_Response>::value
  >
{
};

template<>
struct is_service<robo24_interfaces::srv::Claw>
  : std::true_type
{
};

template<>
struct is_service_request<robo24_interfaces::srv::Claw_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robo24_interfaces::srv::Claw_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBO24_INTERFACES__SRV__DETAIL__CLAW__TRAITS_HPP_
