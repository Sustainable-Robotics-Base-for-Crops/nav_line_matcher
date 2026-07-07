# nav_line_matcher

Lifecycle node that drives a robot along a straight line segment. It exposes the `line_matcher` action server, tracks lateral and course error against the segment, and publishes a repurposed `odom` message for [`nav_path_follow`](../nav_path_follow/README.md). [`nav_replay`](../nav_replay/README.md) sends `LineMatcher` goals for straight mission segments.

## Overview

On each control cycle (rate `control_looprate`):

1. Read robot pose from `/loc/odom`.
2. Project the robot onto the line `[point_begin, point_end]` (or a dynamic end point).
3. Compute lateral deviation, course deviation, and distances to segment ends.
4. Publish action feedback (status, deviations, distances).
5. If errors are within limits, publish `odom` for path following.
6. Succeed when the segment end is reached (distance, segment overrun, or cut-line crossing).

Goals can be **static** (`point_end` fixed in the goal) or **dynamic** (`is_dynamic`: end point updated live via `line_matcher/_action/update_goal`).

When `end_on_cut_line_cross` is set and `cut_line_a` ≠ `cut_line_b`, the goal also succeeds once the robot crosses the cut line (signed lateral deviation flips past `cut_line_overshoot`).

A goal can be preempted. A new goal sent while one is running is accepted live (`accept_pending_goal`) and replaces the current target.

On excessive lateral or course error, the server sets the corresponding `error_loc_path_*` status bit, stops publishing `odom`, and terminates the goal if still active.

## Action interface

Defined in `nav_interfaces/action/LineMatcher.action`.

**Goal**

| Field                   | Description                                                                                      |
| ----------------------- | ------------------------------------------------------------------------------------------------ |
| `point_begin`           | Start of the line segment                                                                        |
| `point_end`             | End of the segment (static goals)                                                                |
| `is_dynamic`            | If true, use `dynamic_point_end` updated by topic instead of `point_end`                         |
| `is_working_zone`       | Per-segment working-zone flags. Index `0` is forwarded in feedback and `odom.twist.linear.z`     |
| `is_uturn`              | If true, use `lateral_deviation_max.uturn` instead of `lateral_deviation_max` for error checking |
| `cut_line_a`            | First point of the optional cut line (used when `end_on_cut_line_cross` is true)                 |
| `cut_line_b`            | Second point of the optional cut line                                                            |
| `end_on_cut_line_cross` | If true, succeed when the robot crosses `[cut_line_a, cut_line_b]`                               |

**Result**

| Field               | Description                                                    |
| ------------------- | -------------------------------------------------------------- |
| `status`            | Bitmask (`nav_interfaces::AutoStatus`) on abnormal termination |
| `end_point_reached` | `true` when the segment end is reached                         |

**Feedback** (published every control cycle)

| Field                | Description                                              |
| -------------------- | -------------------------------------------------------- |
| `status`             | Error flags (lateral/course deviation, etc.)             |
| `distance_to_end`    | Distance along the line to the end                       |
| `distance_to_begin`  | Distance along the line to the start                     |
| `lateral_deviation`  | Signed lateral error (m)                                 |
| `course_deviation`   | Signed heading error (rad)                               |
| `is_in_working_zone` | `is_working_zone[0]` from the goal (or `false` if empty) |

## Parameters

### Node parameters

| Parameter                   | Default | Description                  |
| --------------------------- | ------- | ---------------------------- |
| `control_looprate`          | `10.0`  | Action control loop (Hz)     |
| `zone_precision_multiplier` | `0.7`   | End-of-line tolerance factor |

End-of-segment distance threshold: `zone_precision = lateral_deviation_max × zone_precision_multiplier`.

Cut-line crossing overshoot: `cut_line_overshoot = 0.05` m (hardcoded in the server).

### Remote parameters (`/auto/arbitration`)

Loaded at configure time from `/auto/arbitration`. Updates are applied on parameter events.

| Parameter                     | Default (header) | Description                                |
| ----------------------------- | ---------------- | ------------------------------------------ |
| `lateral_deviation_max`       | `0.4` (m)        | Max lateral deviation before error         |
| `lateral_deviation_max.uturn` | `1.5` (m)        | Max lateral deviation when `goal.is_uturn` |
| `course_deviation_max`        | `π/8` (rad)      | Max course deviation before error          |

Configure fails if `/auto/arbitration` is not available.

## Topics

| Topic                                     | Type                    | Direction | Description                                          |
| ----------------------------------------- | ----------------------- | --------- | ---------------------------------------------------- |
| `/loc/odom`                               | `nav_msgs/msg/Odometry` | In        | Robot pose and velocity                              |
| `line_matcher/_action/update_goal`        | `nav_msgs/msg/Odometry` | In        | Dynamic segment end (`pose.pose.position`)           |
| `line_matcher/_action/reset_dynamic_goal` | `std_msgs/msg/Bool`     | In        | Reset dynamic end to `(0, 0, 0)` when `data == true` |
| `odom`                                    | `nav_msgs/msg/Odometry` | Out       | Path-tracking payload for `nav_path_follow`          |

Non-standard `odom` payload:

| Field                   | Value in this node                                              |
| ----------------------- | --------------------------------------------------------------- |
| `pose.pose.position.y`  | Lateral deviation (m)                                           |
| `pose.pose.orientation` | Course deviation as yaw quaternion                              |
| `twist.twist.angular.x` | `0` (line curvature)                                            |
| `twist.twist.angular.y` | `0` (future curvature, unused for lines)                        |
| `twist.twist.angular.z` | Robot angular velocity from `/loc/odom`                         |
| `twist.twist.linear.z`  | `is_working_zone[0]` from the goal (`true` → `1.0`, else `0.0`) |
