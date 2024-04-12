// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
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
#include "crtp_interface/srv/detail/crtp_packet_send__struct.h"
#include "crtp_interface/srv/detail/crtp_packet_send__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool crtp_interface__msg__crtp_packet__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * crtp_interface__msg__crtp_packet__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool crtp_interface__srv__crtp_packet_send__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("crtp_interface.srv._crtp_packet_send.CrtpPacketSend_Request", full_classname_dest, 59) == 0);
  }
  crtp_interface__srv__CrtpPacketSend_Request * ros_message = _ros_message;
  {  // channel
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->channel = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // address
    PyObject * field = PyObject_GetAttrString(_pymsg, "address");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_UINT8);
      Py_ssize_t size = 5;
      uint8_t * dest = ros_message->address;
      for (Py_ssize_t i = 0; i < size; ++i) {
        uint8_t tmp = *(npy_uint8 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(uint8_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // datarate
    PyObject * field = PyObject_GetAttrString(_pymsg, "datarate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->datarate = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // packet
    PyObject * field = PyObject_GetAttrString(_pymsg, "packet");
    if (!field) {
      return false;
    }
    if (!crtp_interface__msg__crtp_packet__convert_from_py(field, &ros_message->packet)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crtp_interface__srv__crtp_packet_send__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CrtpPacketSend_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crtp_interface.srv._crtp_packet_send");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CrtpPacketSend_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  crtp_interface__srv__CrtpPacketSend_Request * ros_message = (crtp_interface__srv__CrtpPacketSend_Request *)raw_ros_message;
  {  // channel
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->channel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // address
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "address");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT8);
    assert(sizeof(npy_uint8) == sizeof(uint8_t));
    npy_uint8 * dst = (npy_uint8 *)PyArray_GETPTR1(seq_field, 0);
    uint8_t * src = &(ros_message->address[0]);
    memcpy(dst, src, 5 * sizeof(uint8_t));
    Py_DECREF(field);
  }
  {  // datarate
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->datarate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "datarate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // packet
    PyObject * field = NULL;
    field = crtp_interface__msg__crtp_packet__convert_to_py(&ros_message->packet);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "packet", field);
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
// #include "crtp_interface/srv/detail/crtp_packet_send__struct.h"
// already included above
// #include "crtp_interface/srv/detail/crtp_packet_send__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool crtp_interface__srv__crtp_packet_send__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[61];
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
    assert(strncmp("crtp_interface.srv._crtp_packet_send.CrtpPacketSend_Response", full_classname_dest, 60) == 0);
  }
  crtp_interface__srv__CrtpPacketSend_Response * ros_message = _ros_message;
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
PyObject * crtp_interface__srv__crtp_packet_send__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CrtpPacketSend_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crtp_interface.srv._crtp_packet_send");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CrtpPacketSend_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  crtp_interface__srv__CrtpPacketSend_Response * ros_message = (crtp_interface__srv__CrtpPacketSend_Response *)raw_ros_message;
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
