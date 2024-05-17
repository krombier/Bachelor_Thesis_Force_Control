// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages_fr3:srv/SetPose.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_POSE__STRUCT_H_
#define MESSAGES_FR3__SRV__DETAIL__SET_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetPose in the package messages_fr3.
typedef struct messages_fr3__srv__SetPose_Request
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
} messages_fr3__srv__SetPose_Request;

// Struct for a sequence of messages_fr3__srv__SetPose_Request.
typedef struct messages_fr3__srv__SetPose_Request__Sequence
{
  messages_fr3__srv__SetPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages_fr3__srv__SetPose_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetPose in the package messages_fr3.
typedef struct messages_fr3__srv__SetPose_Response
{
  bool success;
} messages_fr3__srv__SetPose_Response;

// Struct for a sequence of messages_fr3__srv__SetPose_Response.
typedef struct messages_fr3__srv__SetPose_Response__Sequence
{
  messages_fr3__srv__SetPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages_fr3__srv__SetPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_POSE__STRUCT_H_
