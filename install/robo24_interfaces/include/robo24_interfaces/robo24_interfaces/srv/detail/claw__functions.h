// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robo24_interfaces:srv/Claw.idl
// generated code does not contain a copyright notice

#ifndef ROBO24_INTERFACES__SRV__DETAIL__CLAW__FUNCTIONS_H_
#define ROBO24_INTERFACES__SRV__DETAIL__CLAW__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robo24_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "robo24_interfaces/srv/detail/claw__struct.h"

/// Initialize srv/Claw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robo24_interfaces__srv__Claw_Request
 * )) before or use
 * robo24_interfaces__srv__Claw_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Request__init(robo24_interfaces__srv__Claw_Request * msg);

/// Finalize srv/Claw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Request__fini(robo24_interfaces__srv__Claw_Request * msg);

/// Create srv/Claw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robo24_interfaces__srv__Claw_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
robo24_interfaces__srv__Claw_Request *
robo24_interfaces__srv__Claw_Request__create();

/// Destroy srv/Claw message.
/**
 * It calls
 * robo24_interfaces__srv__Claw_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Request__destroy(robo24_interfaces__srv__Claw_Request * msg);

/// Check for srv/Claw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Request__are_equal(const robo24_interfaces__srv__Claw_Request * lhs, const robo24_interfaces__srv__Claw_Request * rhs);

/// Copy a srv/Claw message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Request__copy(
  const robo24_interfaces__srv__Claw_Request * input,
  robo24_interfaces__srv__Claw_Request * output);

/// Initialize array of srv/Claw messages.
/**
 * It allocates the memory for the number of elements and calls
 * robo24_interfaces__srv__Claw_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Request__Sequence__init(robo24_interfaces__srv__Claw_Request__Sequence * array, size_t size);

/// Finalize array of srv/Claw messages.
/**
 * It calls
 * robo24_interfaces__srv__Claw_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Request__Sequence__fini(robo24_interfaces__srv__Claw_Request__Sequence * array);

/// Create array of srv/Claw messages.
/**
 * It allocates the memory for the array and calls
 * robo24_interfaces__srv__Claw_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
robo24_interfaces__srv__Claw_Request__Sequence *
robo24_interfaces__srv__Claw_Request__Sequence__create(size_t size);

/// Destroy array of srv/Claw messages.
/**
 * It calls
 * robo24_interfaces__srv__Claw_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Request__Sequence__destroy(robo24_interfaces__srv__Claw_Request__Sequence * array);

/// Check for srv/Claw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Request__Sequence__are_equal(const robo24_interfaces__srv__Claw_Request__Sequence * lhs, const robo24_interfaces__srv__Claw_Request__Sequence * rhs);

/// Copy an array of srv/Claw messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Request__Sequence__copy(
  const robo24_interfaces__srv__Claw_Request__Sequence * input,
  robo24_interfaces__srv__Claw_Request__Sequence * output);

/// Initialize srv/Claw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robo24_interfaces__srv__Claw_Response
 * )) before or use
 * robo24_interfaces__srv__Claw_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Response__init(robo24_interfaces__srv__Claw_Response * msg);

/// Finalize srv/Claw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Response__fini(robo24_interfaces__srv__Claw_Response * msg);

/// Create srv/Claw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robo24_interfaces__srv__Claw_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
robo24_interfaces__srv__Claw_Response *
robo24_interfaces__srv__Claw_Response__create();

/// Destroy srv/Claw message.
/**
 * It calls
 * robo24_interfaces__srv__Claw_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Response__destroy(robo24_interfaces__srv__Claw_Response * msg);

/// Check for srv/Claw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Response__are_equal(const robo24_interfaces__srv__Claw_Response * lhs, const robo24_interfaces__srv__Claw_Response * rhs);

/// Copy a srv/Claw message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Response__copy(
  const robo24_interfaces__srv__Claw_Response * input,
  robo24_interfaces__srv__Claw_Response * output);

/// Initialize array of srv/Claw messages.
/**
 * It allocates the memory for the number of elements and calls
 * robo24_interfaces__srv__Claw_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Response__Sequence__init(robo24_interfaces__srv__Claw_Response__Sequence * array, size_t size);

/// Finalize array of srv/Claw messages.
/**
 * It calls
 * robo24_interfaces__srv__Claw_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Response__Sequence__fini(robo24_interfaces__srv__Claw_Response__Sequence * array);

/// Create array of srv/Claw messages.
/**
 * It allocates the memory for the array and calls
 * robo24_interfaces__srv__Claw_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
robo24_interfaces__srv__Claw_Response__Sequence *
robo24_interfaces__srv__Claw_Response__Sequence__create(size_t size);

/// Destroy array of srv/Claw messages.
/**
 * It calls
 * robo24_interfaces__srv__Claw_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
void
robo24_interfaces__srv__Claw_Response__Sequence__destroy(robo24_interfaces__srv__Claw_Response__Sequence * array);

/// Check for srv/Claw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Response__Sequence__are_equal(const robo24_interfaces__srv__Claw_Response__Sequence * lhs, const robo24_interfaces__srv__Claw_Response__Sequence * rhs);

/// Copy an array of srv/Claw messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robo24_interfaces
bool
robo24_interfaces__srv__Claw_Response__Sequence__copy(
  const robo24_interfaces__srv__Claw_Response__Sequence * input,
  robo24_interfaces__srv__Claw_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBO24_INTERFACES__SRV__DETAIL__CLAW__FUNCTIONS_H_
