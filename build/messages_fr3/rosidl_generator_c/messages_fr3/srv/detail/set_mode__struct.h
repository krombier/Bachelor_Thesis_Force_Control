// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages_fr3:srv/SetMode.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES_FR3__SRV__DETAIL__SET_MODE__STRUCT_H_
#define MESSAGES_FR3__SRV__DETAIL__SET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetMode in the package messages_fr3.
typedef struct messages_fr3__srv__SetMode_Request
{
  int32_t mode;
} messages_fr3__srv__SetMode_Request;

// Struct for a sequence of messages_fr3__srv__SetMode_Request.
typedef struct messages_fr3__srv__SetMode_Request__Sequence
{
  messages_fr3__srv__SetMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages_fr3__srv__SetMode_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetMode in the package messages_fr3.
typedef struct messages_fr3__srv__SetMode_Response
{
  bool success;
} messages_fr3__srv__SetMode_Response;

// Struct for a sequence of messages_fr3__srv__SetMode_Response.
typedef struct messages_fr3__srv__SetMode_Response__Sequence
{
  messages_fr3__srv__SetMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages_fr3__srv__SetMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES_FR3__SRV__DETAIL__SET_MODE__STRUCT_H_
