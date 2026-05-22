# nav_line_matcher

Lifecycle node that drives a robot along a straight line segment. It exposes the `line_matcher` action server, tracks lateral and course error against the segment, and publishes a repurposed `odom` message for [`nav_path_follow`](../nav_path_follow/README.md).

**Node:** `line_matcher_server` · **Executable:** `nav_line_matcher_node` · **Action:** `nav_interfaces/action/LineMatcher`

## Overview

On each control cycle (rate `control_looprate`):

1. Read robot pose from `/loc/odom`.
2. Project the robot onto the line `[point_begin, point_end]` (or a dynamic end point).
3. Compute lateral deviation, course deviation, and distances to segment ends.
4. Publish action feedback; if errors are within limits, publish `odom` for path following.
5. Succeed when the end of the segment is reached (distance or segment overrun).

Goals can be **static** (`point_end` fixed in the goal) or **dynamic** (`is_dynamic`: end point updated live via `line_matcher/_action/update_goal`).

## Action interface

Defined in `nav_interfaces/action/LineMatcher.action`.

**Goal**

| Field             | Description                                                              |
| ----------------- | ------------------------------------------------------------------------ |
| `point_begin`     | Start of the line segment                                                |
| `point_end`       | End of the segment (static goals)                                        |
| `is_dynamic`      | If true, use `dynamic_point_end` updated by topic instead of `point_end` |
| `is_working_zone` | Reserved in the message definition; **not used** by the server yet       |

**Result**

| Field               | Description                                                    |
| ------------------- | -------------------------------------------------------------- |
| `status`            | Bitmask (`nav_interfaces::AutoStatus`) on abnormal termination |
| `end_point_reached` | `true` when the segment end is reached                         |

**Feedback** (published every control cycle)

| Field                | Description                                                |
| -------------------- | ---------------------------------------------------------- |
| `status`             | Error flags (lateral/course deviation, etc.)               |
| `distance_to_end`    | Distance along the line to the end                         |
| `distance_to_begin`  | Distance along the line to the start                       |
| `lateral_deviation`  | Signed lateral error (m)                                   |
| `course_deviation`   | Signed heading error (rad)                                 |
| `is_in_working_zone` | Defined in the action; **not populated** by the server yet |

On excessive lateral or course error, the server sets the corresponding `error_loc_path_*` status bit, **stops publishing `odom`**, and may terminate the goal if still active.

## Parameters

### Node parameters

| Parameter                   | Default | Description                  |
| --------------------------- | ------- | ---------------------------- |
| `control_looprate`          | `10.0`  | Action control loop (Hz)     |
| `zone_precision_multiplier` | `0.7`   | End-of-line tolerance factor |

### Remote parameters (`/auto/arbitration`)

Loaded at configure time via `AsyncParametersClient`; updates are applied on parameter events.

| Parameter               | Default (header) | Description                            |
| ----------------------- | ---------------- | -------------------------------------- |
| `lateral_deviation_max` | `0.4` (m)        | Max \|lateral deviation\| before error |
| `course_deviation_max`  | `π/8` (rad)      | Max \|course deviation\| before error  |

End-of-segment distance threshold: `zone_precision = lateral_deviation_max × zone_precision_multiplier`.

Configure fails if `/auto/arbitration` is not available.

## Topics

| Topic                                     | Type                    | Direction | Description                                          |
| ----------------------------------------- | ----------------------- | --------- | ---------------------------------------------------- |
| `/loc/odom`                               | `nav_msgs/msg/Odometry` | In        | Robot pose (standard semantics)                      |
| `line_matcher/_action/update_goal`        | `nav_msgs/msg/Odometry` | In        | Dynamic segment end (`pose.pose.position`)           |
| `line_matcher/_action/reset_dynamic_goal` | `std_msgs/msg/Bool`     | In        | Reset dynamic end to `(0, 0, 0)` when `data == true` |
| `odom`                                    | `nav_msgs/msg/Odometry` | Out       | Path-tracking payload for `nav_path_follow`          |

### `odom` payload (non-standard)

Fields reused for line-tracking errors (see also `nav_path_follow` README):

| Field                   | Value in this node                                                                           |
| ----------------------- | -------------------------------------------------------------------------------------------- |
| `pose.pose.position.y`  | Lateral deviation (m)                                                                        |
| `pose.pose.orientation` | Course deviation as yaw quaternion                                                           |
| `twist.twist.angular.x` | `0` (line curvature)                                                                         |
| `twist.twist.angular.y` | `0` (future curvature, unused for lines)                                                     |
| `twist.twist.angular.z` | Robot angular velocity from `/loc/odom`                                                      |
| `twist.twist.linear.z`  | **Not set** (defaults to `0`); `nav_path_follow` uses this as a working-zone flag when `> 0` |
