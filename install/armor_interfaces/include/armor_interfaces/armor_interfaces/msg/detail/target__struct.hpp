// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from armor_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef ARMOR_INTERFACES__MSG__DETAIL__TARGET__STRUCT_HPP_
#define ARMOR_INTERFACES__MSG__DETAIL__TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__armor_interfaces__msg__Target __attribute__((deprecated))
#else
# define DEPRECATED__armor_interfaces__msg__Target __declspec(deprecated)
#endif

namespace armor_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Target_
{
  using Type = Target_<ContainerAllocator>;

  explicit Target_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tracking = false;
      this->id = 0;
      this->num = 0;
      this->delay = 0;
      this->yaw = 0.0;
      this->v_yaw = 0.0;
      this->r1 = 0.0;
      this->r2 = 0.0;
      this->dz = 0.0;
      this->is_master = false;
    }
  }

  explicit Target_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tracking = false;
      this->id = 0;
      this->num = 0;
      this->delay = 0;
      this->yaw = 0.0;
      this->v_yaw = 0.0;
      this->r1 = 0.0;
      this->r2 = 0.0;
      this->dz = 0.0;
      this->is_master = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _tracking_type =
    bool;
  _tracking_type tracking;
  using _id_type =
    uint8_t;
  _id_type id;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _num_type =
    uint8_t;
  _num_type num;
  using _delay_type =
    uint8_t;
  _delay_type delay;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _v_yaw_type =
    double;
  _v_yaw_type v_yaw;
  using _r1_type =
    double;
  _r1_type r1;
  using _r2_type =
    double;
  _r2_type r2;
  using _dz_type =
    double;
  _dz_type dz;
  using _is_master_type =
    bool;
  _is_master_type is_master;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__tracking(
    const bool & _arg)
  {
    this->tracking = _arg;
    return *this;
  }
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__num(
    const uint8_t & _arg)
  {
    this->num = _arg;
    return *this;
  }
  Type & set__delay(
    const uint8_t & _arg)
  {
    this->delay = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__v_yaw(
    const double & _arg)
  {
    this->v_yaw = _arg;
    return *this;
  }
  Type & set__r1(
    const double & _arg)
  {
    this->r1 = _arg;
    return *this;
  }
  Type & set__r2(
    const double & _arg)
  {
    this->r2 = _arg;
    return *this;
  }
  Type & set__dz(
    const double & _arg)
  {
    this->dz = _arg;
    return *this;
  }
  Type & set__is_master(
    const bool & _arg)
  {
    this->is_master = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    armor_interfaces::msg::Target_<ContainerAllocator> *;
  using ConstRawPtr =
    const armor_interfaces::msg::Target_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<armor_interfaces::msg::Target_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<armor_interfaces::msg::Target_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      armor_interfaces::msg::Target_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<armor_interfaces::msg::Target_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      armor_interfaces::msg::Target_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<armor_interfaces::msg::Target_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<armor_interfaces::msg::Target_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<armor_interfaces::msg::Target_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__armor_interfaces__msg__Target
    std::shared_ptr<armor_interfaces::msg::Target_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__armor_interfaces__msg__Target
    std::shared_ptr<armor_interfaces::msg::Target_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->tracking != other.tracking) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->num != other.num) {
      return false;
    }
    if (this->delay != other.delay) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->v_yaw != other.v_yaw) {
      return false;
    }
    if (this->r1 != other.r1) {
      return false;
    }
    if (this->r2 != other.r2) {
      return false;
    }
    if (this->dz != other.dz) {
      return false;
    }
    if (this->is_master != other.is_master) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_

// alias to use template instance with default allocator
using Target =
  armor_interfaces::msg::Target_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace armor_interfaces

#endif  // ARMOR_INTERFACES__MSG__DETAIL__TARGET__STRUCT_HPP_
