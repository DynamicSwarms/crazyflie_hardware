// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crtp_interface:msg/CrtpPacket.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__TRAITS_HPP_
#define CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crtp_interface/msg/detail/crtp_packet__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crtp_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const CrtpPacket & msg,
  std::ostream & out)
{
  out << "{";
  // member: port
  {
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << ", ";
  }

  // member: channel
  {
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: data_length
  {
    out << "data_length: ";
    rosidl_generator_traits::value_to_yaml(msg.data_length, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CrtpPacket & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << "\n";
  }

  // member: channel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: data_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_length: ";
    rosidl_generator_traits::value_to_yaml(msg.data_length, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CrtpPacket & msg, bool use_flow_style = false)
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
  const crtp_interface::msg::CrtpPacket & msg,
  std::ostream & out, size_t indentation = 0)
{
  crtp_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crtp_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const crtp_interface::msg::CrtpPacket & msg)
{
  return crtp_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crtp_interface::msg::CrtpPacket>()
{
  return "crtp_interface::msg::CrtpPacket";
}

template<>
inline const char * name<crtp_interface::msg::CrtpPacket>()
{
  return "crtp_interface/msg/CrtpPacket";
}

template<>
struct has_fixed_size<crtp_interface::msg::CrtpPacket>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crtp_interface::msg::CrtpPacket>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crtp_interface::msg::CrtpPacket>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRTP_INTERFACE__MSG__DETAIL__CRTP_PACKET__TRAITS_HPP_
