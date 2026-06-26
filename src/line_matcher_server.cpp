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

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/auto/arbitration");
  parameter_event_sub_ =
      parameters_client_->on_parameter_event(std::bind(&LineMatcherServer::parameters_callback, this, _1));

  if (!parameters_client_->wait_for_service(1s))
  {
    RCLCPP_ERROR(this->get_logger(), "Parameters server /auto/arbitration not available");
    return LNI::CallbackReturn::FAILURE;
  }

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_activate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_activate(state);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/loc/odom", 10, std::bind(&LineMatcherServer::odom_callback, this, _1));
  odom_update_goal_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(server_name_ + "/_action/update_goal", rclcpp::QoS(1),
                                                         std::bind(&LineMatcherServer::update_goal_callback, this, _1));
  reset_dynamic_point_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      server_name_ + "/_action/reset_dynamic_goal", rclcpp::QoS(1),
      std::bind(&LineMatcherServer::reset_dynamic_point_callback, this, _1));

  action_server_ = std::make_unique<nav2_util::SimpleActionServer<LineMatcherAction>>(
      this, server_name_, std::bind(&LineMatcherServer::execute_callback, this), nullptr,
      std::chrono::milliseconds(500), true);

  action_server_->activate();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_deactivate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_deactivate(state);
  action_server_->deactivate();
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_cleanup(const rclcpp_lifecycle::State&)
{
  parameters_client_.reset();
  parameter_event_sub_.reset();
  action_server_.reset();

  odom_pub_.reset();
  odom_sub_.reset();
  odom_update_goal_sub_.reset();
  reset_dynamic_point_sub_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LineMatcherServer::on_shutdown(const rclcpp_lifecycle::State&)
{
  parameters_client_.reset();
  parameter_event_sub_.reset();
  action_server_.reset();

  odom_pub_.reset();
  odom_sub_.reset();
  odom_update_goal_sub_.reset();
  reset_dynamic_point_sub_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

bool LineMatcherServer::compute_command(const std::shared_ptr<const LineMatcherAction::Goal>& goal,
                                        const geometry_msgs::msg::Point& point_end)
{
  nav_msgs::msg::Odometry odom_msg;
  std::shared_ptr<LineMatcherAction::Feedback> feedback = std::make_shared<LineMatcherAction::Feedback>();

  odom_msg.twist.twist.angular.x = 0.0;  // Curvature of the trajectory at the robot position, is null
  odom_msg.twist.twist.angular.y =
      0.0;  // Curvature of the trajectory at the robot position in x seconds only used in path matcher, is still null
  odom_msg.twist.twist.angular.z =
      current_odom_.twist.twist.angular.z;  // Angular speed of the robot, used only in predictive control

  update_distance_to_finish(goal->point_begin, point_end);
  update_distance_to_begin(goal->point_begin, point_end);
  compute_error_on_line(odom_msg, goal->point_begin, point_end);

  if (abs(lateral_deviation_) > lateral_deviation_max_)
  {
    feedback->status |= uint64_t(nav_interfaces::AutoStatus::error_loc_path_lateral_deviation);
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, 5000,
                                "lateral deviation > lateral deviation max : abs(" << lateral_deviation_ << ") > "
                                                                                   << lateral_deviation_max_);
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

bool LineMatcherServer::current_goal_reached(const std::shared_ptr<const LineMatcherAction::Goal>& goal,
                                             const geometry_msgs::msg::Point& point_end)
{
  update_distance_to_finish(goal->point_begin, point_end);
  // Check if close to end or loc goes beyond the end
  if (distance_to_end_ < zone_precision_ ||
      nav_util::is_end_segment_exceeded(actual_position_, goal->point_begin, point_end))
  {
    return true;
  }
  return false;
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
        if ((goal->is_dynamic && current_goal_reached(goal, dynamic_point_end_)) ||
            (!goal->is_dynamic && current_goal_reached(goal, goal->point_end)))
        {
          RCLCPP_INFO(this->get_logger(), "Point reached");
          result->end_point_reached = true;
          break;
        }

        bool result{ true };
        if (goal->is_dynamic)
        {
          result = compute_command(goal, dynamic_point_end_);
        }
        else
        {
          result = compute_command(goal, goal->point_end);
        }
        if (result == false)
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

void LineMatcherServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  current_odom_ = *msg;
  actual_position_.x = msg->pose.pose.position.x;
  actual_position_.y = msg->pose.pose.position.y;
  nav_util::quaternion_to_yaw(msg->pose.pose.orientation, actual_course_);
}

void LineMatcherServer::update_goal_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  dynamic_point_end_ = msg->pose.pose.position;
}

void LineMatcherServer::reset_dynamic_point_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == true)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    // TODO : cleared dynamic point should have nan points or zero points ? Should take care the point is not empty if
    // using it
    dynamic_point_end_ = geometry_msgs::msg::Point();
  }
}

void LineMatcherServer::parameters_handle(const rcl_interfaces::msg::Parameter& parameter)
{
  if (parameter.name == "lateral_deviation_max")
  {
    lateral_deviation_max_ = parameter.value.double_value;
    zone_precision_ = lateral_deviation_max_ * zone_precision_multiplier_;
  }
  else if (parameter.name == "course_deviation_max")
  {
    course_deviation_max_ = parameter.value.double_value;
  }
}

void LineMatcherServer::parameters_callback(rcl_interfaces::msg::ParameterEvent::UniquePtr event)
{
  for (auto& new_parameter : event->new_parameters)
  {
    parameters_handle(new_parameter);
  }

  for (auto& changed_parameter : event->changed_parameters)
  {
    parameters_handle(changed_parameter);
  }
}
}  // namespace nav_line_matcher

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav_line_matcher::LineMatcherServer)
