// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robo24_interfaces:srv/Claw.idl
// generated code does not contain a copyright notice

#ifndef ROBO24_INTERFACES__SRV__DETAIL__CLAW__STRUCT_H_
#define ROBO24_INTERFACES__SRV__DETAIL__CLAW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Claw in the package robo24_interfaces.
typedef struct robo24_interfaces__srv__Claw_Request
{
  int16_t pct;
  int16_t msec;
} robo24_interfaces__srv__Claw_Request;

// Struct for a sequence of robo24_interfaces__srv__Claw_Request.
typedef struct robo24_interfaces__srv__Claw_Request__Sequence
{
  robo24_interfaces__srv__Claw_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robo24_interfaces__srv__Claw_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'resp'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Claw in the package robo24_interfaces.
typedef struct robo24_interfaces__srv__Claw_Response
{
  rosidl_runtime_c__String resp;
} robo24_interfaces__srv__Claw_Response;

// Struct for a sequence of robo24_interfaces__srv__Claw_Response.
typedef struct robo24_interfaces__srv__Claw_Response__Sequence
{
  robo24_interfaces__srv__Claw_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robo24_interfaces__srv__Claw_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBO24_INTERFACES__SRV__DETAIL__CLAW__STRUCT_H_
