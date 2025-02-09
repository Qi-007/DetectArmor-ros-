// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from armor_interfaces:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef ARMOR_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_
#define ARMOR_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "armor_interfaces/msg/detail/armor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace armor_interfaces
{

namespace msg
{

namespace builder
{

class Init_Armor_world_pose
{
public:
  explicit Init_Armor_world_pose(::armor_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  ::armor_interfaces::msg::Armor world_pose(::armor_interfaces::msg::Armor::_world_pose_type arg)
  {
    msg_.world_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::armor_interfaces::msg::Armor msg_;
};

class Init_Armor_pose
{
public:
  explicit Init_Armor_pose(::armor_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_world_pose pose(::armor_interfaces::msg::Armor::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Armor_world_pose(msg_);
  }

private:
  ::armor_interfaces::msg::Armor msg_;
};

class Init_Armor_apexs
{
public:
  explicit Init_Armor_apexs(::armor_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_pose apexs(::armor_interfaces::msg::Armor::_apexs_type arg)
  {
    msg_.apexs = std::move(arg);
    return Init_Armor_pose(msg_);
  }

private:
  ::armor_interfaces::msg::Armor msg_;
};

class Init_Armor_distance_to_center
{
public:
  explicit Init_Armor_distance_to_center(::armor_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_apexs distance_to_center(::armor_interfaces::msg::Armor::_distance_to_center_type arg)
  {
    msg_.distance_to_center = std::move(arg);
    return Init_Armor_apexs(msg_);
  }

private:
  ::armor_interfaces::msg::Armor msg_;
};

class Init_Armor_color
{
public:
  explicit Init_Armor_color(::armor_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_distance_to_center color(::armor_interfaces::msg::Armor::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_Armor_distance_to_center(msg_);
  }

private:
  ::armor_interfaces::msg::Armor msg_;
};

class Init_Armor_type
{
public:
  explicit Init_Armor_type(::armor_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_color type(::armor_interfaces::msg::Armor::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Armor_color(msg_);
  }

private:
  ::armor_interfaces::msg::Armor msg_;
};

class Init_Armor_number
{
public:
  Init_Armor_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Armor_type number(::armor_interfaces::msg::Armor::_number_type arg)
  {
    msg_.number = std::move(arg);
    return Init_Armor_type(msg_);
  }

private:
  ::armor_interfaces::msg::Armor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::armor_interfaces::msg::Armor>()
{
  return armor_interfaces::msg::builder::Init_Armor_number();
}

}  // namespace armor_interfaces

#endif  // ARMOR_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_
