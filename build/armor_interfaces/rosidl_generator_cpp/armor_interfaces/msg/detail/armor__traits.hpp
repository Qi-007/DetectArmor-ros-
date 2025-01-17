// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from armor_interfaces:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef ARMOR_INTERFACES__MSG__DETAIL__ARMOR__TRAITS_HPP_
#define ARMOR_INTERFACES__MSG__DETAIL__ARMOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "armor_interfaces/msg/detail/armor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'apexs'
#include "geometry_msgs/msg/detail/point32__traits.hpp"
// Member 'pose'
// Member 'world_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace armor_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Armor & msg,
  std::ostream & out)
{
  out << "{";
  // member: number
  {
    out << "number: ";
    rosidl_generator_traits::value_to_yaml(msg.number, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << ", ";
  }

  // member: distance_to_center
  {
    out << "distance_to_center: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_center, out);
    out << ", ";
  }

  // member: apexs
  {
    if (msg.apexs.size() == 0) {
      out << "apexs: []";
    } else {
      out << "apexs: [";
      size_t pending_items = msg.apexs.size();
      for (auto item : msg.apexs) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: world_pose
  {
    out << "world_pose: ";
    to_flow_style_yaml(msg.world_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Armor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number: ";
    rosidl_generator_traits::value_to_yaml(msg.number, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }

  // member: distance_to_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_to_center: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_center, out);
    out << "\n";
  }

  // member: apexs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.apexs.size() == 0) {
      out << "apexs: []\n";
    } else {
      out << "apexs:\n";
      for (auto item : msg.apexs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: world_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "world_pose:\n";
    to_block_style_yaml(msg.world_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Armor & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace armor_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use armor_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const armor_interfaces::msg::Armor & msg,
  std::ostream & out, size_t indentation = 0)
{
  armor_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use armor_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const armor_interfaces::msg::Armor & msg)
{
  return armor_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<armor_interfaces::msg::Armor>()
{
  return "armor_interfaces::msg::Armor";
}

template<>
inline const char * name<armor_interfaces::msg::Armor>()
{
  return "armor_interfaces/msg/Armor";
}

template<>
struct has_fixed_size<armor_interfaces::msg::Armor>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<armor_interfaces::msg::Armor>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<armor_interfaces::msg::Armor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARMOR_INTERFACES__MSG__DETAIL__ARMOR__TRAITS_HPP_
