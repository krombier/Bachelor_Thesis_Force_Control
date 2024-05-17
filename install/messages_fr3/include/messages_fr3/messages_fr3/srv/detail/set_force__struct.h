// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages_fr3:srv/SetForce.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_FORCE__STRUCT_H_
#define MESSAGES_FR3__SRV__DETAIL__SET_FORCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetForce in the package messages_fr3.
typedef struct messages_fr3__srv__SetForce_Request
{
  double x_force;
  double y_force;
  double z_force;
  double x_torque;
  double y_torque;
  double z_torque;
} messages_fr3__srv__SetForce_Request;

// Struct for a sequence of messages_fr3__srv__SetForce_Request.
typedef struct messages_fr3__srv__SetForce_Request__Sequence
{
  messages_fr3__srv__SetForce_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages_fr3__srv__SetForce_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetForce in the package messages_fr3.
typedef struct messages_fr3__srv__SetForce_Response
{
  bool success;
} messages_fr3__srv__SetForce_Response;

// Struct for a sequence of messages_fr3__srv__SetForce_Response.
typedef struct messages_fr3__srv__SetForce_Response__Sequence
{
  messages_fr3__srv__SetForce_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages_fr3__srv__SetForce_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_FORCE__STRUCT_H_
