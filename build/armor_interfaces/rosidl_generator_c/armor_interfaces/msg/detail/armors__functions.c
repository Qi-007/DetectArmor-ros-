// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from armor_interfaces:msg/Armors.idl
// generated code does not contain a copyright notice
#include "armor_interfaces/msg/detail/armors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `armors`
#include "armor_interfaces/msg/detail/armor__functions.h"

bool
armor_interfaces__msg__Armors__init(armor_interfaces__msg__Armors * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    armor_interfaces__msg__Armors__fini(msg);
    return false;
  }
  // armors
  if (!armor_interfaces__msg__Armor__Sequence__init(&msg->armors, 0)) {
    armor_interfaces__msg__Armors__fini(msg);
    return false;
  }
  return true;
}

void
armor_interfaces__msg__Armors__fini(armor_interfaces__msg__Armors * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // armors
  armor_interfaces__msg__Armor__Sequence__fini(&msg->armors);
}

bool
armor_interfaces__msg__Armors__are_equal(const armor_interfaces__msg__Armors * lhs, const armor_interfaces__msg__Armors * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // armors
  if (!armor_interfaces__msg__Armor__Sequence__are_equal(
      &(lhs->armors), &(rhs->armors)))
  {
    return false;
  }
  return true;
}

bool
armor_interfaces__msg__Armors__copy(
  const armor_interfaces__msg__Armors * input,
  armor_interfaces__msg__Armors * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // armors
  if (!armor_interfaces__msg__Armor__Sequence__copy(
      &(input->armors), &(output->armors)))
  {
    return false;
  }
  return true;
}

armor_interfaces__msg__Armors *
armor_interfaces__msg__Armors__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  armor_interfaces__msg__Armors * msg = (armor_interfaces__msg__Armors *)allocator.allocate(sizeof(armor_interfaces__msg__Armors), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(armor_interfaces__msg__Armors));
  bool success = armor_interfaces__msg__Armors__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
armor_interfaces__msg__Armors__destroy(armor_interfaces__msg__Armors * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    armor_interfaces__msg__Armors__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
armor_interfaces__msg__Armors__Sequence__init(armor_interfaces__msg__Armors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  armor_interfaces__msg__Armors * data = NULL;

  if (size) {
    data = (armor_interfaces__msg__Armors *)allocator.zero_allocate(size, sizeof(armor_interfaces__msg__Armors), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = armor_interfaces__msg__Armors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        armor_interfaces__msg__Armors__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
armor_interfaces__msg__Armors__Sequence__fini(armor_interfaces__msg__Armors__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      armor_interfaces__msg__Armors__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

armor_interfaces__msg__Armors__Sequence *
armor_interfaces__msg__Armors__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  armor_interfaces__msg__Armors__Sequence * array = (armor_interfaces__msg__Armors__Sequence *)allocator.allocate(sizeof(armor_interfaces__msg__Armors__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = armor_interfaces__msg__Armors__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
armor_interfaces__msg__Armors__Sequence__destroy(armor_interfaces__msg__Armors__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    armor_interfaces__msg__Armors__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
armor_interfaces__msg__Armors__Sequence__are_equal(const armor_interfaces__msg__Armors__Sequence * lhs, const armor_interfaces__msg__Armors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!armor_interfaces__msg__Armors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
armor_interfaces__msg__Armors__Sequence__copy(
  const armor_interfaces__msg__Armors__Sequence * input,
  armor_interfaces__msg__Armors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(armor_interfaces__msg__Armors);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    armor_interfaces__msg__Armors * data =
      (armor_interfaces__msg__Armors *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!armor_interfaces__msg__Armors__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          armor_interfaces__msg__Armors__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!armor_interfaces__msg__Armors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
