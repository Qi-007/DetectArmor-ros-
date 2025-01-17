// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from armor_interfaces:msg/Armors.idl
// generated code does not contain a copyright notice

#ifndef ARMOR_INTERFACES__MSG__DETAIL__ARMORS__BUILDER_HPP_
#define ARMOR_INTERFACES__MSG__DETAIL__ARMORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "armor_interfaces/msg/detail/armors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace armor_interfaces
{

namespace msg
{

namespace builder
{

class Init_Armors_armors
{
public:
  explicit Init_Armors_armors(::armor_interfaces::msg::Armors & msg)
  : msg_(msg)
  {}
  ::armor_interfaces::msg::Armors armors(::armor_interfaces::msg::Armors::_armors_type arg)
  {
    msg_.armors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::armor_interfaces::msg::Armors msg_;
};

class Init_Armors_header
{
public:
  Init_Armors_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Armors_armors header(::armor_interfaces::msg::Armors::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Armors_armors(msg_);
  }

private:
  ::armor_interfaces::msg::Armors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::armor_interfaces::msg::Armors>()
{
  return armor_interfaces::msg::builder::Init_Armors_header();
}

}  // namespace armor_interfaces

#endif  // ARMOR_INTERFACES__MSG__DETAIL__ARMORS__BUILDER_HPP_
