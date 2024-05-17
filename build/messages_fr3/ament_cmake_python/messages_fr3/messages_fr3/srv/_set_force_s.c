// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from messages_fr3:srv/SetForce.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "messages_fr3/srv/detail/set_force__struct.h"
#include "messages_fr3/srv/detail/set_force__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool messages_fr3__srv__set_force__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("messages_fr3.srv._set_force.SetForce_Request", full_classname_dest, 44) == 0);
  }
  messages_fr3__srv__SetForce_Request * ros_message = _ros_message;
  {  // x_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_torque");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_torque = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_torque");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_torque = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_torque");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_torque = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * messages_fr3__srv__set_force__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetForce_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("messages_fr3.srv._set_force");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetForce_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  messages_fr3__srv__SetForce_Request * ros_message = (messages_fr3__srv__SetForce_Request *)raw_ros_message;
  {  // x_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_torque
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_torque);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_torque", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_torque
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_torque);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_torque", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_torque
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_torque);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_torque", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "messages_fr3/srv/detail/set_force__struct.h"
// already included above
// #include "messages_fr3/srv/detail/set_force__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool messages_fr3__srv__set_force__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("messages_fr3.srv._set_force.SetForce_Response", full_classname_dest, 45) == 0);
  }
  messages_fr3__srv__SetForce_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * messages_fr3__srv__set_force__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetForce_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("messages_fr3.srv._set_force");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetForce_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  messages_fr3__srv__SetForce_Response * ros_message = (messages_fr3__srv__SetForce_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
