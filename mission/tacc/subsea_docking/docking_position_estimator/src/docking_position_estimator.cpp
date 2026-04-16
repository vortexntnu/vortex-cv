#include <spdlog/spdlog.h>  // til testing
// #include <cstdlib>
#include <docking_position_estimator/docking_position_estimator.hpp>

#include <vector>
#include <vortex/utils/types.hpp>

namespace vortex::docking_position_estimator {

DockingPositionEstimator::DockingPositionEstimator(
    const DockingPositionEstimatorConfig& config)
    : config_(config) {}

std::vector<CornerEstimate> DockingPositionEstimator::find_corner_estimates(
    const std::vector<vortex::utils::types::LineSegment2D>& lines,
    const Eigen::Vector2f& drone_pos,
    float drone_heading) const {
    std::vector<vortex::utils::types::LineSegment2D> right_wall_candidates;
    std::vector<vortex::utils::types::LineSegment2D> far_wall_candidates;
    std::vector<vortex::utils::types::LineSegment2D> left_wall_candidates;
    std::vector<CornerEstimate> corner_estimates;

    for (const auto& line : lines) {
        const WallClassification classification =
            classify_wall(line, drone_pos, drone_heading);

        if (classification == WallClassification::RightWall) {
            right_wall_candidates.push_back(line);
        } 
        else if (classification == WallClassification::FarWall) {
            far_wall_candidates.push_back(line);
        }
        else if (classification == WallClassification::LeftWall) {
            left_wall_candidates.push_back(line);
        }
    }

    spdlog::info("==== SUMMARY ====");
    spdlog::info("Right wall candidates: {}", right_wall_candidates.size());
    spdlog::info("Far wall candidates: {}", far_wall_candidates.size());
    spdlog::info("Left wall candidates: {}", left_wall_candidates.size());

    auto process_corner = [&](const auto& side_wall,
                              const auto& far_wall) {

        Eigen::Vector2f corner_intersection;

        if (!compute_line_intersection(side_wall, far_wall, corner_intersection)) {
            return;
        }

        float wall_angle = angle_between_lines(side_wall, far_wall);

        if (wall_angle < config_.min_corner_angle_rad ||
            wall_angle > config_.max_corner_angle_rad) {
            return;
        }

        corner_estimates.push_back({
            side_wall,
            far_wall,
            corner_intersection
        });
    };

    for (const auto& far_wall : far_wall_candidates) {

        if (!config_.use_left_wall) {
            for (const auto& right_wall : right_wall_candidates) {
                process_corner(right_wall, far_wall);
            }
        }

        if (config_.use_left_wall) {
            for (const auto& left_wall : left_wall_candidates) {
                process_corner(left_wall, far_wall);
            }
        }
    }
    return corner_estimates;
}

CornerEstimate DockingPositionEstimator::select_best_corner(
    const std::vector<CornerEstimate>& possible_corners,
    const Eigen::Vector2f& drone_pos) const {

    float min_distance = std::numeric_limits<float>::max();
    CornerEstimate best_corner = possible_corners.front();

    for (const auto& corner : possible_corners) {
        float distance = (corner.corner_point - drone_pos).squaredNorm();

        if (distance < min_distance) {
            min_distance = distance;
            best_corner = corner;
        }
    }
    return best_corner;
}

Eigen::Vector2f DockingPositionEstimator::estimate_docking_position(
    const CornerEstimate& estimated_corner,
    const Eigen::Vector2f& drone_pos) const {
    Eigen::Vector2f side_normal =
        compute_normal_towards_point(estimated_corner.side_wall, drone_pos);
    Eigen::Vector2f far_normal =
        compute_normal_towards_point(estimated_corner.far_wall, drone_pos);

    Eigen::Vector2f docking_estimate =
        estimated_corner.corner_point +
        side_normal * config_.side_wall_offset_m +
        far_normal * config_.far_wall_offset_m;

    return docking_estimate;
}

WallClassification DockingPositionEstimator::classify_wall(
    const vortex::utils::types::LineSegment2D& line,
    const Eigen::Vector2f& drone_pos,
    float drone_heading) const {
    const Eigen::Vector2f p0 = to_eigen(line.p0);
    const Eigen::Vector2f p1 = to_eigen(line.p1);

    const Eigen::Vector2f dir = p1 - p0;
    if (dir.squaredNorm() < 1e-6f) {  // ENDRE KRAV TIL LENGDE PÅ LINJE?
        spdlog::info("  -> REJECTED: degenerate line");
        return WallClassification::Rejected;
    }
    // TIL LOGGING
    spdlog::info("Line: ({:.2f}, {:.2f}) -> ({:.2f}, {:.2f})", p0.x(), p0.y(),
                 p1.x(), p1.y());
    //
    const Eigen::Vector2f projection = project_drone_onto_line(drone_pos, line);
    const Eigen::Vector2f rel = projection - drone_pos;

    const float distance = rel.norm();
    // TIL LOGGING
    spdlog::info("  Projection: ({:.2f}, {:.2f})", projection.x(),
                 projection.y());
    spdlog::info("  Rel: ({:.2f}, {:.2f}), dist: {:.2f}", rel.x(), rel.y(),
                 distance);
    //
    if (distance < config_.min_wall_distance_m ||
        distance > config_.max_wall_distance_m) {
        spdlog::info("  -> REJECTED by distance");
        return WallClassification::Rejected;
    }

    const float heading_wall_angle =
        angle_between_line_and_heading(line, drone_heading);
    spdlog::info("  Angle: {:.2f} rad ({:.1f} deg)", heading_wall_angle,
                 heading_wall_angle * 180.0 / M_PI);

    Eigen::Vector2f forward(std::cos(drone_heading), std::sin(drone_heading));
    Eigen::Vector2f right(-std::sin(drone_heading), std::cos(drone_heading));
    Eigen::Vector2f left(std::sin(drone_heading), -std::cos(drone_heading));

    float forward_dist = rel.dot(forward);
    float right_dist = rel.dot(right);
    float left_dist = rel.dot(left);

    // RIGHT WALL: projection has negative y-value in NED, wall is approximately
    // parallel to heading
    if (                   // projection.y() < config_.right_wall_max_y_m &&
        !config_.use_left_wall &&
        right_dist > 0 && 
        heading_wall_angle < config_.parallel_heading_angle_threshold_rad) {
        spdlog::info("  -> Classified as RIGHT candidate");
        return WallClassification::RightWall;
    }

    // FAR WALL: projection has positive x-value, wall is approximately
    // perpendicular to heading
    if (  // projection.x() > config_.far_wall_min_x_m &&
        forward_dist > 0 &&
        heading_wall_angle > config_.perpendicular_heading_angle_threshold_rad) {
        spdlog::info("  -> Classified as FAR candidate");
        return WallClassification::FarWall;
    }

    // LEFT WALL
    if (config_.use_left_wall &&
        left_dist > 0 &&
        heading_wall_angle < config_.parallel_heading_angle_threshold_rad) {
        spdlog::info("  -> Classified as LEFT candidate");
        return WallClassification::LeftWall;
    }

    spdlog::info("  -> REJECTED by geometry");
    return WallClassification::Rejected;
}

Eigen::Vector2f DockingPositionEstimator::project_drone_onto_line(
    const Eigen::Vector2f& drone_pos,
    const vortex::utils::types::LineSegment2D& line) const {
    const Eigen::Vector2f p0 = to_eigen(line.p0);
    const Eigen::Vector2f p1 = to_eigen(line.p1);

    const Eigen::Vector2f line_direction = p1 - p0;
    const float line_length_squared = line_direction.squaredNorm();

    const float projection_parameter =
        (drone_pos - p0).dot(line_direction) / line_length_squared;
    const Eigen::Vector2f projection_point =
        p0 + projection_parameter * line_direction;

    return projection_point;
}

float DockingPositionEstimator::angle_between_line_and_heading(
    const vortex::utils::types::LineSegment2D& line,
    float drone_heading) const {
    Eigen::Vector2f wall_direction = to_eigen(line.p1) - to_eigen(line.p0);
    const float wall_length = wall_direction.norm();

    if (wall_length < 1e-6f) {
        return std::numeric_limits<float>::infinity();
    }

    wall_direction /= wall_length;
    Eigen::Vector2f heading_direction(std::cos(drone_heading),
                                      std::sin(drone_heading));

    float cos_heading_wall_angle = std::abs(wall_direction.dot(
        heading_direction));  // rekkefølge p0 og p1 urelevant
    cos_heading_wall_angle =
        std::clamp(cos_heading_wall_angle, 0.0f, 1.0f);  // unngå små avvik

    return std::acos(cos_heading_wall_angle);
}

bool DockingPositionEstimator::compute_line_intersection(
    const vortex::utils::types::LineSegment2D& line0,
    const vortex::utils::types::LineSegment2D& line1,
    Eigen::Vector2f& intersection) const {
    const Eigen::Vector2f p00 = to_eigen(line0.p0);
    const Eigen::Vector2f p01 = to_eigen(line0.p1);
    const Eigen::Vector2f p10 = to_eigen(line1.p0);
    const Eigen::Vector2f p11 = to_eigen(line1.p1);

    float determinant = (p00.x() - p01.x()) * (p10.y() - p11.y()) -
                        (p00.y() - p01.y()) * (p10.x() - p11.x());

    if (std::abs(determinant) < 1e-6f)
        return false;  // parallelle linjer

    intersection.x() =
        ((p00.x() * p01.y() - p00.y() * p01.x()) * (p10.x() - p11.x()) -
         (p00.x() - p01.x()) * (p10.x() * p11.y() - p10.y() * p11.x())) /
        determinant;
    intersection.y() =
        ((p00.x() * p01.y() - p00.y() * p01.x()) * (p10.y() - p11.y()) -
         (p00.y() - p01.y()) * (p10.x() * p11.y() - p10.y() * p11.x())) /
        determinant;
    return true;
}

float DockingPositionEstimator::angle_between_lines(
    const vortex::utils::types::LineSegment2D& line0,
    const vortex::utils::types::LineSegment2D& line1) const {
    Eigen::Vector2f line0_direction = to_eigen(line0.p1) - to_eigen(line0.p0);

    Eigen::Vector2f line1_direction = to_eigen(line1.p1) - to_eigen(line1.p0);

    const float line0_length = line0_direction.norm();
    const float line1_length = line1_direction.norm();

    if (line0_length < 1e-6f || line1_length < 1e-6f) {
        return std::numeric_limits<float>::infinity();
    }

    line0_direction /= line0_length;
    line1_direction /= line1_length;

    float cos_angle_between_lines =
        std::abs(line0_direction.dot(line1_direction));

    cos_angle_between_lines = std::clamp(cos_angle_between_lines, 0.0f, 1.0f);

    return std::acos(cos_angle_between_lines);
}

Eigen::Vector2f DockingPositionEstimator::compute_normal_towards_point(
    const vortex::utils::types::LineSegment2D& line,
    const Eigen::Vector2f& drone_pos) const {
    Eigen::Vector2f p0 = to_eigen(line.p0);
    Eigen::Vector2f p1 = to_eigen(line.p1);

    Eigen::Vector2f line_direction = p1 - p0;

    if (line_direction.x() < 0.0f ||
        (line_direction.x() == 0.0f && line_direction.y() < 0.0f)) {
        std::swap(p0, p1);
        line_direction = p1 - p0;
    }

    const float line_length = line_direction.norm();
    if (line_length < 1e-6f) {
        return Eigen::Vector2f::Zero();
    }

    line_direction /= line_length;

    Eigen::Vector2f normal(-line_direction.y(), line_direction.x());

    const Eigen::Vector2f line_midpoint = 0.5f * (p0 + p1);
    const Eigen::Vector2f midpoint_to_drone = drone_pos - line_midpoint;

    if (midpoint_to_drone.dot(normal) < 0.0f) {
        normal = -normal;
    }

    return normal;
}

}  // namespace vortex::docking_position_estimator
