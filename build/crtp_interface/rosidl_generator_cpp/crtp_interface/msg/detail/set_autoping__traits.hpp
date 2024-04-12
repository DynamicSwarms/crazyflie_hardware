// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crtp_interface:msg/SetAutoping.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__TRAITS_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crtp_interface/msg/detail/set_autoping__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crtp_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetAutoping & msg,
  std::ostream & out)
{
  out << "{";
  // member: channel
  {
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << ", ";
  }

  // member: address
  {
    if (msg.address.size() == 0) {
      out << "address: []";
    } else {
      out << "address: [";
      size_t pending_items = msg.address.size();
      for (auto item : msg.address) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: datarate
  {
    out << "datarate: ";
    rosidl_generator_traits::value_to_yaml(msg.datarate, out);
    out << ", ";
  }

  // member: rate
  {
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetAutoping & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: channel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << "\n";
  }

  // member: address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.address.size() == 0) {
      out << "address: []\n";
    } else {
      out << "address:\n";
      for (auto item : msg.address) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: datarate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "datarate: ";
    rosidl_generator_traits::value_to_yaml(msg.datarate, out);
    out << "\n";
  }

  // member: rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetAutoping & msg, bool use_flow_style = false)
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

}  // namespace crtp_interface

namespace rosidl_generator_traits
{

[[deprecated("use crtp_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crtp_interface::msg::SetAutoping & msg,
  std::ostream & out, size_t indentation = 0)
{
  crtp_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crtp_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const crtp_interface::msg::SetAutoping & msg)
{
  return crtp_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crtp_interface::msg::SetAutoping>()
{
  return "crtp_interface::msg::SetAutoping";
}

template<>
inline const char * name<crtp_interface::msg::SetAutoping>()
{
  return "crtp_interface/msg/SetAutoping";
}

template<>
struct has_fixed_size<crtp_interface::msg::SetAutoping>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crtp_interface::msg::SetAutoping>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crtp_interface::msg::SetAutoping>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRTP_INTERFACE__MSG__DETAIL__SET_AUTOPING__TRAITS_HPP_
