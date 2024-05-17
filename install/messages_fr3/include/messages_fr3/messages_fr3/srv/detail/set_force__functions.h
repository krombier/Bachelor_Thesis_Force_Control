// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from messages_fr3:srv/SetForce.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_FORCE__FUNCTIONS_H_
#define MESSAGES_FR3__SRV__DETAIL__SET_FORCE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "messages_fr3/msg/rosidl_generator_c__visibility_control.h"

#include "messages_fr3/srv/detail/set_force__struct.h"

/// Initialize srv/SetForce message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * messages_fr3__srv__SetForce_Request
 * )) before or use
 * messages_fr3__srv__SetForce_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Request__init(messages_fr3__srv__SetForce_Request * msg);

/// Finalize srv/SetForce message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Request__fini(messages_fr3__srv__SetForce_Request * msg);

/// Create srv/SetForce message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * messages_fr3__srv__SetForce_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
messages_fr3__srv__SetForce_Request *
messages_fr3__srv__SetForce_Request__create();

/// Destroy srv/SetForce message.
/**
 * It calls
 * messages_fr3__srv__SetForce_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Request__destroy(messages_fr3__srv__SetForce_Request * msg);

/// Check for srv/SetForce message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Request__are_equal(const messages_fr3__srv__SetForce_Request * lhs, const messages_fr3__srv__SetForce_Request * rhs);

/// Copy a srv/SetForce message.
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
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Request__copy(
  const messages_fr3__srv__SetForce_Request * input,
  messages_fr3__srv__SetForce_Request * output);

/// Initialize array of srv/SetForce messages.
/**
 * It allocates the memory for the number of elements and calls
 * messages_fr3__srv__SetForce_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Request__Sequence__init(messages_fr3__srv__SetForce_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetForce messages.
/**
 * It calls
 * messages_fr3__srv__SetForce_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Request__Sequence__fini(messages_fr3__srv__SetForce_Request__Sequence * array);

/// Create array of srv/SetForce messages.
/**
 * It allocates the memory for the array and calls
 * messages_fr3__srv__SetForce_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
messages_fr3__srv__SetForce_Request__Sequence *
messages_fr3__srv__SetForce_Request__Sequence__create(size_t size);

/// Destroy array of srv/SetForce messages.
/**
 * It calls
 * messages_fr3__srv__SetForce_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Request__Sequence__destroy(messages_fr3__srv__SetForce_Request__Sequence * array);

/// Check for srv/SetForce message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Request__Sequence__are_equal(const messages_fr3__srv__SetForce_Request__Sequence * lhs, const messages_fr3__srv__SetForce_Request__Sequence * rhs);

/// Copy an array of srv/SetForce messages.
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
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Request__Sequence__copy(
  const messages_fr3__srv__SetForce_Request__Sequence * input,
  messages_fr3__srv__SetForce_Request__Sequence * output);

/// Initialize srv/SetForce message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * messages_fr3__srv__SetForce_Response
 * )) before or use
 * messages_fr3__srv__SetForce_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Response__init(messages_fr3__srv__SetForce_Response * msg);

/// Finalize srv/SetForce message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Response__fini(messages_fr3__srv__SetForce_Response * msg);

/// Create srv/SetForce message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * messages_fr3__srv__SetForce_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
messages_fr3__srv__SetForce_Response *
messages_fr3__srv__SetForce_Response__create();

/// Destroy srv/SetForce message.
/**
 * It calls
 * messages_fr3__srv__SetForce_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Response__destroy(messages_fr3__srv__SetForce_Response * msg);

/// Check for srv/SetForce message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Response__are_equal(const messages_fr3__srv__SetForce_Response * lhs, const messages_fr3__srv__SetForce_Response * rhs);

/// Copy a srv/SetForce message.
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
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Response__copy(
  const messages_fr3__srv__SetForce_Response * input,
  messages_fr3__srv__SetForce_Response * output);

/// Initialize array of srv/SetForce messages.
/**
 * It allocates the memory for the number of elements and calls
 * messages_fr3__srv__SetForce_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Response__Sequence__init(messages_fr3__srv__SetForce_Response__Sequence * array, size_t size);

/// Finalize array of srv/SetForce messages.
/**
 * It calls
 * messages_fr3__srv__SetForce_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Response__Sequence__fini(messages_fr3__srv__SetForce_Response__Sequence * array);

/// Create array of srv/SetForce messages.
/**
 * It allocates the memory for the array and calls
 * messages_fr3__srv__SetForce_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
messages_fr3__srv__SetForce_Response__Sequence *
messages_fr3__srv__SetForce_Response__Sequence__create(size_t size);

/// Destroy array of srv/SetForce messages.
/**
 * It calls
 * messages_fr3__srv__SetForce_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
void
messages_fr3__srv__SetForce_Response__Sequence__destroy(messages_fr3__srv__SetForce_Response__Sequence * array);

/// Check for srv/SetForce message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Response__Sequence__are_equal(const messages_fr3__srv__SetForce_Response__Sequence * lhs, const messages_fr3__srv__SetForce_Response__Sequence * rhs);

/// Copy an array of srv/SetForce messages.
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
ROSIDL_GENERATOR_C_PUBLIC_messages_fr3
bool
messages_fr3__srv__SetForce_Response__Sequence__copy(
  const messages_fr3__srv__SetForce_Response__Sequence * input,
  messages_fr3__srv__SetForce_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_FORCE__FUNCTIONS_H_
