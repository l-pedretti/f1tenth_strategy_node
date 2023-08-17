// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:srv/ChangeState.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__SRV__DETAIL__CHANGE_STATE__BUILDER_HPP_
#define TUTORIAL_INTERFACES__SRV__DETAIL__CHANGE_STATE__BUILDER_HPP_

#include "tutorial_interfaces/srv/detail/change_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace tutorial_interfaces
{

namespace srv
{

namespace builder
{

class Init_ChangeState_Request_state
{
public:
  Init_ChangeState_Request_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tutorial_interfaces::srv::ChangeState_Request state(::tutorial_interfaces::srv::ChangeState_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::srv::ChangeState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::srv::ChangeState_Request>()
{
  return tutorial_interfaces::srv::builder::Init_ChangeState_Request_state();
}

}  // namespace tutorial_interfaces


namespace tutorial_interfaces
{

namespace srv
{

namespace builder
{

class Init_ChangeState_Response_response
{
public:
  Init_ChangeState_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tutorial_interfaces::srv::ChangeState_Response response(::tutorial_interfaces::srv::ChangeState_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::srv::ChangeState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::srv::ChangeState_Response>()
{
  return tutorial_interfaces::srv::builder::Init_ChangeState_Response_response();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__SRV__DETAIL__CHANGE_STATE__BUILDER_HPP_
