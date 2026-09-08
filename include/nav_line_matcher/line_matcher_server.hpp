// Copyright 2026 SABI AGRI

#ifndef NAV_LINE_MATCHER__LINE_MATCHER_SERVER_HPP_
#define NAV_LINE_MATCHER__LINE_MATCHER_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_util/parameters_client.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_interfaces/action/line_matcher.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using LineMatcherAction = nav_interfaces::action::LineMatcher;

namespace nav_line_matcher
{
class LineMatcherServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LineMatcherServer(const rclcpp::NodeOptions& options);

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

protected:
  void parameters_handle(const rclcpp::Parameter& p);
  void parameters_callback(rcl_interfaces::msg::ParameterEvent::UniquePtr event);
  void update_distance_to_begin(const geometry_msgs::msg::Point& point_begin,
                                const geometry_msgs::msg::Point& point_end);
  void update_distance_to_finish(const geometry_msgs::msg::Point& point_begin,
                                 const geometry_msgs::msg::Point& point_end);
  bool is_terminate_goal(const uint64_t feedback_status);
  bool current_goal_reached(const std::shared_ptr<const LineMatcherAction::Goal>& goal);
  void compute_error_on_line(nav_msgs::msg::Odometry& msg, const geometry_msgs::msg::Point& point_a,
                             const geometry_msgs::msg::Point& point_b);
  void execute_callback();
  bool compute_command(const std::shared_ptr<const LineMatcherAction::Goal>& goal);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  std::string server_name_{ "line_matcher" };

  double control_looprate_{ 10. };
  double zone_precision_multiplier_{ 0.7 };
  double lateral_deviation_max_{ 0.4 };
  double lateral_deviation_max_uturn_{ 1.5 };
  double course_deviation_max_{ M_PI / 8 };
  double course_deviation_max_uturn_{ M_PI / 3 };
  double zone_precision_{ 0.3 };
  double distance_to_end_{ 0. };
  double distance_to_begin_{ 0. };
  double lateral_deviation_{ 0. };
  double course_deviation_{ 0. };
  double actual_course_{ 0. };

  std::mutex mutex_;
  rclcpp::Clock clock_;

  nav_msgs::msg::Odometry current_odom_;
  geometry_msgs::msg::Point actual_position_;

  std::shared_ptr<nav_util::ParametersClient> parameters_client_;
  std::unique_ptr<nav2_util::SimpleActionServer<LineMatcherAction>> action_server_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
}  // namespace nav_line_matcher

#endif  // NAV_LINE_MATCHER__LINE_MATCHER_SERVER_HPP_
