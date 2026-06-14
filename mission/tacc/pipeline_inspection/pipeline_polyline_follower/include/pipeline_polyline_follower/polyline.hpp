#pragma once

#include <Eigen/Core>
#include <vector>

/**
 * State of the directed polyline maintained in the odom XY plane.
 *
 * Joints grow monotonically as the vehicle travels forward.  The arc-length
 * cursor s_current is gated so it can never decrease — waypoints are always
 * ahead of where the vehicle has already been.
 */
struct PolylineState {
    std::vector<Eigen::Vector2d> joints;  ///< ordered odom-frame XY vertices
    double s_current{0.0};                ///< monotone arc-length cursor (m)
    bool initialized{false};  ///< true after first successful update
};

/**
 * Total arc-length of the polyline (sum of segment lengths).
 */
double arc_length(const std::vector<Eigen::Vector2d>& joints);

/**
 * Project @p pos onto the polyline.
 * Returns the arc-length of the closest point on the polyline.
 */
double project_onto_polyline(const std::vector<Eigen::Vector2d>& joints,
                             const Eigen::Vector2d& pos);

/**
 * Integrate newly detected odom-frame polyline vertices into @p state.
 *
 * On first call (state.initialized == false) the direction of @p detected is
 * chosen so that its forward endpoint has positive dot-product with
 * @p vehicle_vel_xy.
 *
 * On subsequent calls the detected polyline is oriented to continue from the
 * current forward end of state.joints, and any portion that lies ahead of the
 * known map is appended.
 *
 * @param detected        ordered odom-frame vertices from current frame
 * @param vehicle_pos_xy  current vehicle XY position in odom frame
 * @param vehicle_vel_xy  current vehicle XY velocity in odom frame (for
 *                        direction bootstrap only)
 * @param extend_margin   minimum distance past the current forward end before
 *                        a detected point is considered "new" (m)
 */
void update_polyline(PolylineState& state,
                     const std::vector<Eigen::Vector2d>& detected,
                     const Eigen::Vector2d& vehicle_pos_xy,
                     const Eigen::Vector2d& vehicle_vel_xy,
                     double extend_margin = 0.15);

/**
 * Update state.s_current to reflect vehicle progress along the polyline.
 * Enforces monotonicity: s_current can only grow.
 */
void advance_cursor(PolylineState& state,
                    const Eigen::Vector2d& vehicle_pos_xy);

/**
 * Compute a look-ahead waypoint at arc-length (s_current + lookahead_dist).
 *
 * @param state         current polyline state
 * @param lookahead_dist distance to look ahead from s_current (m)
 * @param[out] wp_xy    look-ahead XY position in odom frame
 * @param[out] yaw      heading angle (rad) at the look-ahead point
 * @return false if the polyline has no joints yet
 */
bool get_lookahead(const PolylineState& state,
                   double lookahead_dist,
                   Eigen::Vector2d& wp_xy,
                   double& yaw);
