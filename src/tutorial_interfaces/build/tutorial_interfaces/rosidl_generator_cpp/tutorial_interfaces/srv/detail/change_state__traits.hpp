// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tutorial_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__SRV__DETAIL__CHANGE_STATE__TRAITS_HPP_
#define TUTORIAL_INTERFACES__SRV__DETAIL__CHANGE_STATE__TRAITS_HPP_

#include "tutorial_interfaces/srv/detail/change_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tutorial_interfaces::srv::ChangeState_Request>()
{
  return "tutorial_interfaces::srv::ChangeState_Request";
}

template<>
inline const char * name<tutorial_interfaces::srv::ChangeState_Request>()
{
  return "tutorial_interfaces/srv/ChangeState_Request";
}

template<>
struct has_fixed_size<tutorial_interfaces::srv::ChangeState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tutorial_interfaces::srv::ChangeState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tutorial_interfaces::srv::ChangeState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tutorial_interfaces::srv::ChangeState_Response>()
{
  return "tutorial_interfaces::srv::ChangeState_Response";
}

template<>
inline const char * name<tutorial_interfaces::srv::ChangeState_Response>()
{
  return "tutorial_interfaces/srv/ChangeState_Response";
}

template<>
struct has_fixed_size<tutorial_interfaces::srv::ChangeState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tutorial_interfaces::srv::ChangeState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tutorial_interfaces::srv::ChangeState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tutorial_interfaces::srv::ChangeState>()
{
  return "tutorial_interfaces::srv::ChangeState";
}

template<>
inline const char * name<tutorial_interfaces::srv::ChangeState>()
{
  return "tutorial_interfaces/srv/ChangeState";
}

template<>
struct has_fixed_size<tutorial_interfaces::srv::ChangeState>
  : std::integral_constant<
    bool,
    has_fixed_size<tutorial_interfaces::srv::ChangeState_Request>::value &&
    has_fixed_size<tutorial_interfaces::srv::ChangeState_Response>::value
  >
{
};

template<>
struct has_bounded_size<tutorial_interfaces::srv::ChangeState>
  : std::integral_constant<
    bool,
    has_bounded_size<tutorial_interfaces::srv::ChangeState_Request>::value &&
    has_bounded_size<tutorial_interfaces::srv::ChangeState_Response>::value
  >
{
};

template<>
struct is_service<tutorial_interfaces::srv::ChangeState>
  : std::true_type
{
};

template<>
struct is_service_request<tutorial_interfaces::srv::ChangeState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tutorial_interfaces::srv::ChangeState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TUTORIAL_INTERFACES__SRV__DETAIL__CHANGE_STATE__TRAITS_HPP_
