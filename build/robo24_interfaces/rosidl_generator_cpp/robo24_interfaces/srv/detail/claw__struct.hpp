// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robo24_interfaces:srv/Claw.idl
// generated code does not contain a copyright notice

#ifndef ROBO24_INTERFACES__SRV__DETAIL__CLAW__STRUCT_HPP_
#define ROBO24_INTERFACES__SRV__DETAIL__CLAW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robo24_interfaces__srv__Claw_Request __attribute__((deprecated))
#else
# define DEPRECATED__robo24_interfaces__srv__Claw_Request __declspec(deprecated)
#endif

namespace robo24_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Claw_Request_
{
  using Type = Claw_Request_<ContainerAllocator>;

  explicit Claw_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pct = 0;
      this->msec = 0;
    }
  }

  explicit Claw_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pct = 0;
      this->msec = 0;
    }
  }

  // field types and members
  using _pct_type =
    int16_t;
  _pct_type pct;
  using _msec_type =
    int16_t;
  _msec_type msec;

  // setters for named parameter idiom
  Type & set__pct(
    const int16_t & _arg)
  {
    this->pct = _arg;
    return *this;
  }
  Type & set__msec(
    const int16_t & _arg)
  {
    this->msec = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robo24_interfaces::srv::Claw_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robo24_interfaces::srv::Claw_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robo24_interfaces::srv::Claw_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robo24_interfaces::srv::Claw_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robo24_interfaces__srv__Claw_Request
    std::shared_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robo24_interfaces__srv__Claw_Request
    std::shared_ptr<robo24_interfaces::srv::Claw_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Claw_Request_ & other) const
  {
    if (this->pct != other.pct) {
      return false;
    }
    if (this->msec != other.msec) {
      return false;
    }
    return true;
  }
  bool operator!=(const Claw_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Claw_Request_

// alias to use template instance with default allocator
using Claw_Request =
  robo24_interfaces::srv::Claw_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robo24_interfaces


#ifndef _WIN32
# define DEPRECATED__robo24_interfaces__srv__Claw_Response __attribute__((deprecated))
#else
# define DEPRECATED__robo24_interfaces__srv__Claw_Response __declspec(deprecated)
#endif

namespace robo24_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Claw_Response_
{
  using Type = Claw_Response_<ContainerAllocator>;

  explicit Claw_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resp = "";
    }
  }

  explicit Claw_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : resp(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resp = "";
    }
  }

  // field types and members
  using _resp_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _resp_type resp;

  // setters for named parameter idiom
  Type & set__resp(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->resp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robo24_interfaces::srv::Claw_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robo24_interfaces::srv::Claw_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robo24_interfaces::srv::Claw_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robo24_interfaces::srv::Claw_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robo24_interfaces__srv__Claw_Response
    std::shared_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robo24_interfaces__srv__Claw_Response
    std::shared_ptr<robo24_interfaces::srv::Claw_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Claw_Response_ & other) const
  {
    if (this->resp != other.resp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Claw_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Claw_Response_

// alias to use template instance with default allocator
using Claw_Response =
  robo24_interfaces::srv::Claw_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robo24_interfaces

namespace robo24_interfaces
{

namespace srv
{

struct Claw
{
  using Request = robo24_interfaces::srv::Claw_Request;
  using Response = robo24_interfaces::srv::Claw_Response;
};

}  // namespace srv

}  // namespace robo24_interfaces

#endif  // ROBO24_INTERFACES__SRV__DETAIL__CLAW__STRUCT_HPP_
