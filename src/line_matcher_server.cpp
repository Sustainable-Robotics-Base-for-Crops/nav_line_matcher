// Copyright 2026 SABI AGRI

#include "nav_line_matcher/line_matcher_server.hpp"

namespace nav_line_matcher
{
LineMatcherServer::LineMatcherServer(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("line_matcher_server", options)
{
}

LNI::CallbackReturn LineMatcherServer::on_configure(const rclcpp_lifecycle::State&)
{
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_activate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_activate(state);
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_deactivate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_deactivate(state);
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_cleanup(const rclcpp_lifecycle::State&)
{
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_shutdown(const rclcpp_lifecycle::State&)
{
  return LNI::CallbackReturn::SUCCESS;
}
}  // namespace nav_line_matcher

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav_line_matcher::LineMatcherServer)
