#include "pipeline_polyline_follower/polyline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

double arc_length(const std::vector<Eigen::Vector2d>& joints) {
    double s = 0.0;
    for (size_t i = 0; i + 1 < joints.size(); ++i)
        s += (joints[i + 1] - joints[i]).norm();
    return s;
}

double project_onto_polyline(const std::vector<Eigen::Vector2d>& joints,
                             const Eigen::Vector2d& pos) {
    double best_s = 0.0;
    double best_dist = std::numeric_limits<double>::max();
    double accum = 0.0;
    for (size_t i = 0; i + 1 < joints.size(); ++i) {
        const Eigen::Vector2d ab = joints[i + 1] - joints[i];
        const double seg_len = ab.norm();
        if (seg_len < 1e-9) {
            accum += seg_len;
            continue;
        }
        const Eigen::Vector2d ap = pos - joints[i];
        double t = ap.dot(ab) / (seg_len * seg_len);
        t = std::clamp(t, 0.0, 1.0);
        const Eigen::Vector2d closest = joints[i] + t * ab;
        const double dist = (pos - closest).norm();
        if (dist < best_dist) {
            best_dist = dist;
            best_s = accum + t * seg_len;
        }
        accum += seg_len;
    }
    return best_s;
}

void update_polyline(PolylineState& state,
                     const std::vector<Eigen::Vector2d>& detected,
                     const Eigen::Vector2d& vehicle_pos_xy,
                     const Eigen::Vector2d& vehicle_vel_xy,
                     double extend_margin) {
    if (detected.size() < 2)
        return;

    if (!state.initialized) {
        // Bootstrap: orient detected so that forward end aligns with velocity.
        // Use the velocity direction if it has meaningful magnitude, otherwise
        // fall back to the detected-endpoint that is farther from the origin.
        auto oriented = detected;
        const Eigen::Vector2d chord = detected.back() - detected.front();

        Eigen::Vector2d ref = vehicle_vel_xy;
        if (ref.norm() < 0.05) {
            // No reliable velocity yet: pick direction away from vehicle start
            ref = detected.back() - vehicle_pos_xy;
        }

        if (chord.dot(ref) < 0.0)
            std::reverse(oriented.begin(), oriented.end());

        state.joints = oriented;
        state.s_current = 0.0;
        state.initialized = true;
        return;
    }

    // Orient detected so its entry end is closest to the current forward end.
    const Eigen::Vector2d fwd_end = state.joints.back();
    const double d_front = (detected.front() - fwd_end).norm();
    const double d_back = (detected.back() - fwd_end).norm();

    auto oriented = detected;
    if (d_back < d_front)
        std::reverse(oriented.begin(), oriented.end());

    // Determine the "forward" direction at the tip of the known polyline.
    Eigen::Vector2d fwd_dir{1.0, 0.0};
    if (state.joints.size() >= 2) {
        fwd_dir = (state.joints.back() - state.joints[state.joints.size() - 2])
                      .normalized();
    }

    // Append any point in `oriented` that is clearly ahead of fwd_end.
    // fwd_end and fwd_dir are updated after each append so that successive
    // points in the detected polyline are compared against the latest tip,
    // not the original one.
    Eigen::Vector2d tip = fwd_end;
    for (const auto& pt : oriented) {
        const double ahead = (pt - tip).dot(fwd_dir);
        if (ahead > extend_margin) {
            state.joints.push_back(pt);
            tip = pt;
            if (state.joints.size() >= 2) {
                fwd_dir = (state.joints.back() -
                           state.joints[state.joints.size() - 2])
                              .normalized();
            }
        }
    }
}

void advance_cursor(PolylineState& state,
                    const Eigen::Vector2d& vehicle_pos_xy) {
    if (!state.initialized || state.joints.size() < 2)
        return;
    const double s_vehicle =
        project_onto_polyline(state.joints, vehicle_pos_xy);
    // Monotone: only move forward
    if (s_vehicle > state.s_current)
        state.s_current = s_vehicle;
}

bool get_lookahead(const PolylineState& state,
                   double lookahead_dist,
                   Eigen::Vector2d& wp_xy,
                   double& yaw) {
    if (!state.initialized || state.joints.size() < 2)
        return false;

    const double target_s = state.s_current + lookahead_dist;
    double accum = 0.0;

    for (size_t i = 0; i + 1 < state.joints.size(); ++i) {
        const Eigen::Vector2d seg = state.joints[i + 1] - state.joints[i];
        const double seg_len = seg.norm();
        if (seg_len < 1e-9)
            continue;

        if (accum + seg_len >= target_s) {
            const double t = (target_s - accum) / seg_len;
            wp_xy = state.joints[i] + t * seg;
            yaw = std::atan2(seg.y(), seg.x());
            return true;
        }
        accum += seg_len;
    }

    // Beyond the known polyline: extrapolate along the final segment direction
    const Eigen::Vector2d last_seg =
        (state.joints.back() - state.joints[state.joints.size() - 2])
            .normalized();
    const double overshoot = target_s - accum;
    wp_xy = state.joints.back() + overshoot * last_seg;
    yaw = std::atan2(last_seg.y(), last_seg.x());
    return true;
}
