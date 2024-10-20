// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from parc_robot_interfaces:msg/CropYield.idl
// generated code does not contain a copyright notice
#include "parc_robot_interfaces/msg/detail/crop_yield__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
parc_robot_interfaces__msg__CropYield__init(parc_robot_interfaces__msg__CropYield * msg)
{
  if (!msg) {
    return false;
  }
  // data
  return true;
}

void
parc_robot_interfaces__msg__CropYield__fini(parc_robot_interfaces__msg__CropYield * msg)
{
  if (!msg) {
    return;
  }
  // data
}

bool
parc_robot_interfaces__msg__CropYield__are_equal(const parc_robot_interfaces__msg__CropYield * lhs, const parc_robot_interfaces__msg__CropYield * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (lhs->data != rhs->data) {
    return false;
  }
  return true;
}

bool
parc_robot_interfaces__msg__CropYield__copy(
  const parc_robot_interfaces__msg__CropYield * input,
  parc_robot_interfaces__msg__CropYield * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  output->data = input->data;
  return true;
}

parc_robot_interfaces__msg__CropYield *
parc_robot_interfaces__msg__CropYield__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parc_robot_interfaces__msg__CropYield * msg = (parc_robot_interfaces__msg__CropYield *)allocator.allocate(sizeof(parc_robot_interfaces__msg__CropYield), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parc_robot_interfaces__msg__CropYield));
  bool success = parc_robot_interfaces__msg__CropYield__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parc_robot_interfaces__msg__CropYield__destroy(parc_robot_interfaces__msg__CropYield * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    parc_robot_interfaces__msg__CropYield__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
parc_robot_interfaces__msg__CropYield__Sequence__init(parc_robot_interfaces__msg__CropYield__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parc_robot_interfaces__msg__CropYield * data = NULL;

  if (size) {
    data = (parc_robot_interfaces__msg__CropYield *)allocator.zero_allocate(size, sizeof(parc_robot_interfaces__msg__CropYield), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parc_robot_interfaces__msg__CropYield__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parc_robot_interfaces__msg__CropYield__fini(&data[i - 1]);
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
parc_robot_interfaces__msg__CropYield__Sequence__fini(parc_robot_interfaces__msg__CropYield__Sequence * array)
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
      parc_robot_interfaces__msg__CropYield__fini(&array->data[i]);
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

parc_robot_interfaces__msg__CropYield__Sequence *
parc_robot_interfaces__msg__CropYield__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parc_robot_interfaces__msg__CropYield__Sequence * array = (parc_robot_interfaces__msg__CropYield__Sequence *)allocator.allocate(sizeof(parc_robot_interfaces__msg__CropYield__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parc_robot_interfaces__msg__CropYield__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parc_robot_interfaces__msg__CropYield__Sequence__destroy(parc_robot_interfaces__msg__CropYield__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    parc_robot_interfaces__msg__CropYield__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
parc_robot_interfaces__msg__CropYield__Sequence__are_equal(const parc_robot_interfaces__msg__CropYield__Sequence * lhs, const parc_robot_interfaces__msg__CropYield__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!parc_robot_interfaces__msg__CropYield__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
parc_robot_interfaces__msg__CropYield__Sequence__copy(
  const parc_robot_interfaces__msg__CropYield__Sequence * input,
  parc_robot_interfaces__msg__CropYield__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(parc_robot_interfaces__msg__CropYield);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    parc_robot_interfaces__msg__CropYield * data =
      (parc_robot_interfaces__msg__CropYield *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!parc_robot_interfaces__msg__CropYield__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          parc_robot_interfaces__msg__CropYield__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!parc_robot_interfaces__msg__CropYield__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
