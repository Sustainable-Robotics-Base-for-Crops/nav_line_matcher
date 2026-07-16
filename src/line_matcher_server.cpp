// Copyright 2026 SABI AGRI

#include "nav_line_matcher/line_matcher_server.hpp"
#include "nav_util/geometry_computing.hpp"
#include "nav_util/geometry_conversion.hpp"
#include "nav_interfaces/action_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "angles/angles.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace nav_line_matcher
{
LineMatcherServer::LineMatcherServer(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("line_matcher_server", options)
{
  this->declare_parameter("control_looprate", control_looprate_);
  this->declare_parameter("zone_precision_multiplier", zone_precision_multiplier_);
}

LNI::CallbackReturn LineMatcherServer::on_configure(const rclcpp_lifecycle::State&)
{
  this->get_parameter("control_looprate", control_looprate_);
  this->get_parameter("zone_precision_multiplier", zone_precision_multiplier_);

  std::string parameters_server = "/auto/arbitration";

  parameters_client_ = std::make_shared<nav_util::ParametersClient>(shared_from_this(), parameters_server);

  if (!parameters_client_->wait_for_service(1s))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Parameters server " << parameters_server << " not available");
    return LNI::CallbackReturn::FAILURE;
  }

  std::vector<std::string> params_names{ "lateral_deviation_max", "lateral_deviation_max.uturn",
                                         "course_deviation_max" };

  auto params = parameters_client_->get_parameters(params_names);

  if (params.size() == params_names.size())
  {
    for (auto p : params)
    {
      parameters_handle(p);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Param from " << parameters_server << " cannot been get");
    return LNI::CallbackReturn::FAILURE;
  }

  parameters_client_->set_parameter_event_callback(std::bind(&LineMatcherServer::parameters_callback, this, _1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  action_server_ = std::make_unique<nav2_util::SimpleActionServer<LineMatcherAction>>(
      shared_from_this(), server_name_, std::bind(&LineMatcherServer::execute_callback, this), nullptr,
      std::chrono::milliseconds(500), true);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/loc/odom", 10, std::bind(&LineMatcherServer::odom_callback, this, _1));

  action_server_->activate();

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
  action_server_->deactivate();
  action_server_.reset();
  parameters_client_.reset();
  odom_pub_.reset();
  odom_sub_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_shutdown(const rclcpp_lifecycle::State&)
{
  action_server_.reset();
  parameters_client_.reset();
  odom_pub_.reset();
  odom_sub_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

void LineMatcherServer::parameters_handle(const rclcpp::Parameter& p)
{
  if (p.get_name() == "lateral_deviation_max")
  {
    lateral_deviation_max_ = p.as_double();
    zone_precision_ = lateral_deviation_max_ * zone_precision_multiplier_;
  }
  else if (p.get_name() == "lateral_deviation_max.uturn")
  {
    lateral_deviation_max_uturn_ = p.as_double();
  }
  else if (p.get_name() == "course_deviation_max")
  {
    course_deviation_max_ = p.as_double();
  }
}

void LineMatcherServer::parameters_callback(rcl_interfaces::msg::ParameterEvent::UniquePtr event)
{
  for (const auto& changed_parameter : event->changed_parameters)
  {
    parameters_handle(rclcpp::Parameter::from_parameter_msg(changed_parameter));
  }
}

bool LineMatcherServer::compute_command(const std::shared_ptr<const LineMatcherAction::Goal>& goal)
{
  nav_msgs::msg::Odometry odom_msg;
  std::shared_ptr<LineMatcherAction::Feedback> feedback = std::make_shared<LineMatcherAction::Feedback>();

  odom_msg.twist.twist.angular.x = 0.0;  // Curvature of the trajectory at the robot position, is null
  odom_msg.twist.twist.angular.y =
      0.0;  // Curvature of the trajectory at the robot position in x seconds only used in path matcher, is still null
  odom_msg.twist.twist.angular.z =
      current_odom_.twist.twist.angular.z;  // Angular speed of the robot, used only in predictive control

  update_distance_to_finish(goal->point_begin, goal->point_end);
  update_distance_to_begin(goal->point_begin, goal->point_end);
  compute_error_on_line(odom_msg, goal->point_begin, goal->point_end);

  if (!goal->is_working_zone.empty())
  {
    feedback->is_in_working_zone = goal->is_working_zone[0];
    odom_msg.twist.twist.linear.z = goal->is_working_zone[0];
  }
  else
  {
    feedback->is_in_working_zone = false;
    odom_msg.twist.twist.linear.z = 0.0;
  }

  const double active_lateral_deviation_max = goal->is_uturn ? lateral_deviation_max_uturn_ : lateral_deviation_max_;
  if (abs(lateral_deviation_) > active_lateral_deviation_max)
  {
    feedback->status |= uint64_t(nav_interfaces::AutoStatus::error_loc_path_lateral_deviation);

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, 5000,
                                "lateral deviation > lateral deviation max : abs("
                                    << lateral_deviation_ << ") > " << active_lateral_deviation_max
                                    << (goal->is_uturn ? " (uturn threshold)" : ""));

    if (is_terminate_goal(feedback->status))
    {
      return false;
    }
  }
  else if (abs(course_deviation_) > course_deviation_max_)
  {
    feedback->status |= uint64_t(nav_interfaces::AutoStatus::error_loc_path_course_deviation);

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, 5000,
                                "course deviation > course deviation max : abs(" << course_deviation_ << ") > "
                                                                                 << course_deviation_max_);

    if (is_terminate_goal(feedback->status))
    {
      return false;
    }
  }
  else
  {
    odom_pub_->publish(odom_msg);
  }

  feedback->distance_to_end = distance_to_end_;
  feedback->distance_to_begin = distance_to_begin_;
  feedback->course_deviation = course_deviation_;
  feedback->lateral_deviation = lateral_deviation_;
  action_server_->publish_feedback(feedback);

  return true;
}

void LineMatcherServer::compute_error_on_line(nav_msgs::msg::Odometry& msg,
                                              const geometry_msgs::msg::Point& point_begin,
                                              const geometry_msgs::msg::Point& point_end)
{
  lateral_deviation_ = nav_util::get_lateral_deviation_to_line(actual_position_, point_begin, point_end);

  double points_angle = atan2(point_end.y - point_begin.y, point_end.x - point_begin.x);
  course_deviation_ = angles::shortest_angular_distance(points_angle, actual_course_);

  msg.pose.pose.position.y = lateral_deviation_;
  nav_util::yaw_to_quaternion(course_deviation_, msg.pose.pose.orientation);
}

void LineMatcherServer::update_distance_to_begin(const geometry_msgs::msg::Point& point_begin,
                                                 const geometry_msgs::msg::Point& point_end)
{
  geometry_msgs::msg::Point projected_point =
      nav_util::get_point_projected_on_line(actual_position_, point_begin, point_end);
  distance_to_begin_ = nav_util::get_distance_to_point(projected_point, point_begin);
}

void LineMatcherServer::update_distance_to_finish(const geometry_msgs::msg::Point& point_begin,
                                                  const geometry_msgs::msg::Point& point_end)
{
  geometry_msgs::msg::Point projected_point =
      nav_util::get_point_projected_on_line(actual_position_, point_begin, point_end);
  distance_to_end_ = nav_util::get_distance_to_point(projected_point, point_end);
}

bool LineMatcherServer::is_terminate_goal(const uint64_t feedback_status)
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    std::shared_ptr<LineMatcherAction::Result> result = std::make_shared<LineMatcherAction::Result>();
    result->status = feedback_status;
    result->end_point_reached = false;
    action_server_->terminate_all(result);
    return true;
  }

  return false;
}

bool LineMatcherServer::current_goal_reached(const std::shared_ptr<const LineMatcherAction::Goal>& goal)
{
  update_distance_to_finish(goal->point_begin, goal->point_end);

  // Check if close to end or loc goes beyond the end
  if (distance_to_end_ < zone_precision_ ||
      nav_util::is_end_segment_exceeded(actual_position_, goal->point_begin, goal->point_end))
  {
    return true;
  }

  return false;
}

void LineMatcherServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  current_odom_ = *msg;
  actual_position_.x = msg->pose.pose.position.x;
  actual_position_.y = msg->pose.pose.position.y;
  nav_util::quaternion_to_yaw(msg->pose.pose.orientation, actual_course_);
}

void LineMatcherServer::execute_callback()
{
  RCLCPP_INFO(this->get_logger(), "Execute goal...");
  std::shared_ptr<const LineMatcherAction::Goal> goal = action_server_->get_current_goal();
  std::shared_ptr<LineMatcherAction::Result> result = std::make_shared<LineMatcherAction::Result>();

  try
  {
    rclcpp::WallRate loop_rate(control_looprate_);
    while (rclcpp::ok())
    {
      if (action_server_ == nullptr || !action_server_->is_server_active())
      {
        RCLCPP_DEBUG(this->get_logger(), "Action server unvailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested())
      {
        RCLCPP_INFO(this->get_logger(), "Goal was canceled.");
        action_server_->terminate_all();
        return;
      }

      if (action_server_->is_preempt_requested())
      {
        RCLCPP_INFO(this->get_logger(), "using new points");
        // Accept pending handle and use new goal
        goal = action_server_->accept_pending_goal();
      }

      {
        std::scoped_lock<std::mutex> lock(mutex_);

        if (current_goal_reached(goal))
        {
          RCLCPP_INFO(this->get_logger(), "Point reached");
          result->end_point_reached = true;
          break;
        }

        if (!compute_command(goal))
        {
          return;
        }
      }

      if (!loop_rate.sleep())
      {
        RCLCPP_WARN(this->get_logger(), "Control loop missed its desired rate of %.4fHz", control_looprate_);
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    std::shared_ptr<LineMatcherAction::Result> result = std::make_shared<LineMatcherAction::Result>();
    action_server_->terminate_current(result);
    return;
  }

  action_server_->succeeded_current(result);
}
}  // namespace nav_line_matcher

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav_line_matcher::LineMatcherServer)
