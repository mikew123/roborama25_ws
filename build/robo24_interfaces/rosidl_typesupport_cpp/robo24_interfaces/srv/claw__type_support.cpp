// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from robo24_interfaces:srv/Claw.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robo24_interfaces/srv/detail/claw__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robo24_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Claw_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Claw_Request_type_support_ids_t;

static const _Claw_Request_type_support_ids_t _Claw_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Claw_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Claw_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Claw_Request_type_support_symbol_names_t _Claw_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robo24_interfaces, srv, Claw_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robo24_interfaces, srv, Claw_Request)),
  }
};

typedef struct _Claw_Request_type_support_data_t
{
  void * data[2];
} _Claw_Request_type_support_data_t;

static _Claw_Request_type_support_data_t _Claw_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Claw_Request_message_typesupport_map = {
  2,
  "robo24_interfaces",
  &_Claw_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Claw_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Claw_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Claw_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Claw_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robo24_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robo24_interfaces::srv::Claw_Request>()
{
  return &::robo24_interfaces::srv::rosidl_typesupport_cpp::Claw_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robo24_interfaces, srv, Claw_Request)() {
  return get_message_type_support_handle<robo24_interfaces::srv::Claw_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robo24_interfaces/srv/detail/claw__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robo24_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Claw_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Claw_Response_type_support_ids_t;

static const _Claw_Response_type_support_ids_t _Claw_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Claw_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Claw_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Claw_Response_type_support_symbol_names_t _Claw_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robo24_interfaces, srv, Claw_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robo24_interfaces, srv, Claw_Response)),
  }
};

typedef struct _Claw_Response_type_support_data_t
{
  void * data[2];
} _Claw_Response_type_support_data_t;

static _Claw_Response_type_support_data_t _Claw_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Claw_Response_message_typesupport_map = {
  2,
  "robo24_interfaces",
  &_Claw_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Claw_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Claw_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Claw_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Claw_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robo24_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robo24_interfaces::srv::Claw_Response>()
{
  return &::robo24_interfaces::srv::rosidl_typesupport_cpp::Claw_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robo24_interfaces, srv, Claw_Response)() {
  return get_message_type_support_handle<robo24_interfaces::srv::Claw_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robo24_interfaces/srv/detail/claw__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robo24_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Claw_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Claw_type_support_ids_t;

static const _Claw_type_support_ids_t _Claw_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Claw_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Claw_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Claw_type_support_symbol_names_t _Claw_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robo24_interfaces, srv, Claw)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robo24_interfaces, srv, Claw)),
  }
};

typedef struct _Claw_type_support_data_t
{
  void * data[2];
} _Claw_type_support_data_t;

static _Claw_type_support_data_t _Claw_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Claw_service_typesupport_map = {
  2,
  "robo24_interfaces",
  &_Claw_service_typesupport_ids.typesupport_identifier[0],
  &_Claw_service_typesupport_symbol_names.symbol_name[0],
  &_Claw_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Claw_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Claw_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robo24_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robo24_interfaces::srv::Claw>()
{
  return &::robo24_interfaces::srv::rosidl_typesupport_cpp::Claw_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, robo24_interfaces, srv, Claw)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<robo24_interfaces::srv::Claw>();
}

#ifdef __cplusplus
}
#endif
