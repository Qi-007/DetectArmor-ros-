// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from armor_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef ARMOR_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
#define ARMOR_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "armor_interfaces/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace armor_interfaces
{

namespace msg
{

namespace builder
{

class Init_Target_is_master
{
public:
  explicit Init_Target_is_master(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  ::armor_interfaces::msg::Target is_master(::armor_interfaces::msg::Target::_is_master_type arg)
  {
    msg_.is_master = std::move(arg);
    return std::move(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_dz
{
public:
  explicit Init_Target_dz(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_is_master dz(::armor_interfaces::msg::Target::_dz_type arg)
  {
    msg_.dz = std::move(arg);
    return Init_Target_is_master(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_r2
{
public:
  explicit Init_Target_r2(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_dz r2(::armor_interfaces::msg::Target::_r2_type arg)
  {
    msg_.r2 = std::move(arg);
    return Init_Target_dz(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_r1
{
public:
  explicit Init_Target_r1(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_r2 r1(::armor_interfaces::msg::Target::_r1_type arg)
  {
    msg_.r1 = std::move(arg);
    return Init_Target_r2(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_v_yaw
{
public:
  explicit Init_Target_v_yaw(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_r1 v_yaw(::armor_interfaces::msg::Target::_v_yaw_type arg)
  {
    msg_.v_yaw = std::move(arg);
    return Init_Target_r1(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_yaw
{
public:
  explicit Init_Target_yaw(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_v_yaw yaw(::armor_interfaces::msg::Target::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Target_v_yaw(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_delay
{
public:
  explicit Init_Target_delay(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_yaw delay(::armor_interfaces::msg::Target::_delay_type arg)
  {
    msg_.delay = std::move(arg);
    return Init_Target_yaw(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_num
{
public:
  explicit Init_Target_num(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_delay num(::armor_interfaces::msg::Target::_num_type arg)
  {
    msg_.num = std::move(arg);
    return Init_Target_delay(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_velocity
{
public:
  explicit Init_Target_velocity(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_num velocity(::armor_interfaces::msg::Target::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Target_num(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_position
{
public:
  explicit Init_Target_position(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_velocity position(::armor_interfaces::msg::Target::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Target_velocity(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_id
{
public:
  explicit Init_Target_id(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_position id(::armor_interfaces::msg::Target::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Target_position(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_tracking
{
public:
  explicit Init_Target_tracking(::armor_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_id tracking(::armor_interfaces::msg::Target::_tracking_type arg)
  {
    msg_.tracking = std::move(arg);
    return Init_Target_id(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

class Init_Target_header
{
public:
  Init_Target_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_tracking header(::armor_interfaces::msg::Target::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Target_tracking(msg_);
  }

private:
  ::armor_interfaces::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::armor_interfaces::msg::Target>()
{
  return armor_interfaces::msg::builder::Init_Target_header();
}

}  // namespace armor_interfaces

#endif  // ARMOR_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
