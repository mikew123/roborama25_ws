// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robo24_interfaces:srv/Claw.idl
// generated code does not contain a copyright notice

#ifndef ROBO24_INTERFACES__SRV__DETAIL__CLAW__BUILDER_HPP_
#define ROBO24_INTERFACES__SRV__DETAIL__CLAW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robo24_interfaces/srv/detail/claw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robo24_interfaces
{

namespace srv
{

namespace builder
{

class Init_Claw_Request_msec
{
public:
  explicit Init_Claw_Request_msec(::robo24_interfaces::srv::Claw_Request & msg)
  : msg_(msg)
  {}
  ::robo24_interfaces::srv::Claw_Request msec(::robo24_interfaces::srv::Claw_Request::_msec_type arg)
  {
    msg_.msec = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robo24_interfaces::srv::Claw_Request msg_;
};

class Init_Claw_Request_pct
{
public:
  Init_Claw_Request_pct()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Claw_Request_msec pct(::robo24_interfaces::srv::Claw_Request::_pct_type arg)
  {
    msg_.pct = std::move(arg);
    return Init_Claw_Request_msec(msg_);
  }

private:
  ::robo24_interfaces::srv::Claw_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robo24_interfaces::srv::Claw_Request>()
{
  return robo24_interfaces::srv::builder::Init_Claw_Request_pct();
}

}  // namespace robo24_interfaces


namespace robo24_interfaces
{

namespace srv
{

namespace builder
{

class Init_Claw_Response_resp
{
public:
  Init_Claw_Response_resp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robo24_interfaces::srv::Claw_Response resp(::robo24_interfaces::srv::Claw_Response::_resp_type arg)
  {
    msg_.resp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robo24_interfaces::srv::Claw_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robo24_interfaces::srv::Claw_Response>()
{
  return robo24_interfaces::srv::builder::Init_Claw_Response_resp();
}

}  // namespace robo24_interfaces

#endif  // ROBO24_INTERFACES__SRV__DETAIL__CLAW__BUILDER_HPP_
