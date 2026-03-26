#pragma once

#include "bearing_localization/ray_measurement.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace bearing_localization {

/// Stores a sliding window of RayMeasurements.
class MeasurementBuffer {
   public:
    MeasurementBuffer(size_t window_size, double max_age_sec);

    void add(const RayMeasurement& ray);

    /// Remove entries older than max_age_sec relative to now.
    void prune(const rclcpp::Time& now);

    void remove_indices(const std::vector<size_t>& indices);

    void clear();

    const std::vector<RayMeasurement>& rays() const;
    size_t size() const;

   private:
    size_t window_size_;
    double max_age_sec_;
    std::vector<RayMeasurement> buffer_;
};

}  // namespace bearing_localization
