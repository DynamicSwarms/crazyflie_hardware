// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crtp_interface:srv/CrtpPacketSend.idl
// generated code does not contain a copyright notice

#ifndef CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__TRAITS_HPP_
#define CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crtp_interface/srv/detail/crtp_packet_send__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'packet'
#include "crtp_interface/msg/detail/crtp_packet__traits.hpp"

namespace crtp_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const CrtpPacketSend_Request & msg,
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

  // member: packet
  {
    out << "packet: ";
    to_flow_style_yaml(msg.packet, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CrtpPacketSend_Request & msg,
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

  // member: packet
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "packet:\n";
    to_block_style_yaml(msg.packet, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CrtpPacketSend_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crtp_interface

namespace rosidl_generator_traits
{

[[deprecated("use crtp_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crtp_interface::srv::CrtpPacketSend_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crtp_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crtp_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const crtp_interface::srv::CrtpPacketSend_Request & msg)
{
  return crtp_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crtp_interface::srv::CrtpPacketSend_Request>()
{
  return "crtp_interface::srv::CrtpPacketSend_Request";
}

template<>
inline const char * name<crtp_interface::srv::CrtpPacketSend_Request>()
{
  return "crtp_interface/srv/CrtpPacketSend_Request";
}

template<>
struct has_fixed_size<crtp_interface::srv::CrtpPacketSend_Request>
  : std::integral_constant<bool, has_fixed_size<crtp_interface::msg::CrtpPacket>::value> {};

template<>
struct has_bounded_size<crtp_interface::srv::CrtpPacketSend_Request>
  : std::integral_constant<bool, has_bounded_size<crtp_interface::msg::CrtpPacket>::value> {};

template<>
struct is_message<crtp_interface::srv::CrtpPacketSend_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crtp_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const CrtpPacketSend_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CrtpPacketSend_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CrtpPacketSend_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crtp_interface

namespace rosidl_generator_traits
{

[[deprecated("use crtp_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crtp_interface::srv::CrtpPacketSend_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crtp_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crtp_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const crtp_interface::srv::CrtpPacketSend_Response & msg)
{
  return crtp_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crtp_interface::srv::CrtpPacketSend_Response>()
{
  return "crtp_interface::srv::CrtpPacketSend_Response";
}

template<>
inline const char * name<crtp_interface::srv::CrtpPacketSend_Response>()
{
  return "crtp_interface/srv/CrtpPacketSend_Response";
}

template<>
struct has_fixed_size<crtp_interface::srv::CrtpPacketSend_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crtp_interface::srv::CrtpPacketSend_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crtp_interface::srv::CrtpPacketSend_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crtp_interface::srv::CrtpPacketSend>()
{
  return "crtp_interface::srv::CrtpPacketSend";
}

template<>
inline const char * name<crtp_interface::srv::CrtpPacketSend>()
{
  return "crtp_interface/srv/CrtpPacketSend";
}

template<>
struct has_fixed_size<crtp_interface::srv::CrtpPacketSend>
  : std::integral_constant<
    bool,
    has_fixed_size<crtp_interface::srv::CrtpPacketSend_Request>::value &&
    has_fixed_size<crtp_interface::srv::CrtpPacketSend_Response>::value
  >
{
};

template<>
struct has_bounded_size<crtp_interface::srv::CrtpPacketSend>
  : std::integral_constant<
    bool,
    has_bounded_size<crtp_interface::srv::CrtpPacketSend_Request>::value &&
    has_bounded_size<crtp_interface::srv::CrtpPacketSend_Response>::value
  >
{
};

template<>
struct is_service<crtp_interface::srv::CrtpPacketSend>
  : std::true_type
{
};

template<>
struct is_service_request<crtp_interface::srv::CrtpPacketSend_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crtp_interface::srv::CrtpPacketSend_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRTP_INTERFACE__SRV__DETAIL__CRTP_PACKET_SEND__TRAITS_HPP_
