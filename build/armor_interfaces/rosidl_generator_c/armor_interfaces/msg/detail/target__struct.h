// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from armor_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef ARMOR_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_
#define ARMOR_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Target in the package armor_interfaces.
typedef struct armor_interfaces__msg__Target
{
  std_msgs__msg__Header header;
  bool tracking;
  uint8_t id;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  uint8_t num;
  uint8_t delay;
  double yaw;
  double v_yaw;
  double r1;
  double r2;
  double dz;
  bool is_master;
} armor_interfaces__msg__Target;

// Struct for a sequence of armor_interfaces__msg__Target.
typedef struct armor_interfaces__msg__Target__Sequence
{
  armor_interfaces__msg__Target * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} armor_interfaces__msg__Target__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARMOR_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_
