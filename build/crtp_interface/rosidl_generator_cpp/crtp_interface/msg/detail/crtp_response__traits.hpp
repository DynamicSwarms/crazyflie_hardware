// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crtp_interface:msg/CrtpResponse.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__TRAITS_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crtp_interface/msg/detail/crtp_response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'packet'
#include "crtp_interface/msg/detail/crtp_packet__traits.hpp"

namespace crtp_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const CrtpResponse & msg,
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

  // member: packet
  {
    out << "packet: ";
    to_flow_style_yaml(msg.packet, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CrtpResponse & msg,
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

  // member: packet
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "packet:\n";
    to_block_style_yaml(msg.packet, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CrtpResponse & msg, bool use_flow_style = false)
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
  const crtp_interface::msg::CrtpResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  crtp_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crtp_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const crtp_interface::msg::CrtpResponse & msg)
{
  return crtp_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crtp_interface::msg::CrtpResponse>()
{
  return "crtp_interface::msg::CrtpResponse";
}

template<>
inline const char * name<crtp_interface::msg::CrtpResponse>()
{
  return "crtp_interface/msg/CrtpResponse";
}

template<>
struct has_fixed_size<crtp_interface::msg::CrtpResponse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crtp_interface::msg::CrtpResponse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crtp_interface::msg::CrtpResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_RESPONSE__TRAITS_HPP_
