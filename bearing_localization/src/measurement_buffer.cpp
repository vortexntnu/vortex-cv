#include "bearing_localization/measurement_buffer.hpp"

#include <utility>

namespace bearing_localization {

MeasurementBuffer::MeasurementBuffer(size_t window_size, double max_age_sec)
    : window_size_(window_size), max_age_sec_(max_age_sec) {}

void MeasurementBuffer::add(const RayMeasurement& ray) {
    buffer_.push_back(ray);
    while (buffer_.size() > window_size_) {
        buffer_.erase(buffer_.begin());
    }
}

void MeasurementBuffer::prune(const rclcpp::Time& now) {
    const rclcpp::Time cutoff =
        now - rclcpp::Duration::from_seconds(max_age_sec_);

    std::vector<RayMeasurement> kept;
    kept.reserve(buffer_.size());

    for (const auto& ray : buffer_) {
        if (ray.stamp >= cutoff) {
            kept.push_back(ray);
        }
    }

    buffer_ = std::move(kept);
}

void MeasurementBuffer::remove_indices(const std::vector<size_t>& indices) {
    std::vector<bool> to_remove(buffer_.size(), false);
    for (size_t idx : indices) {
        if (idx < buffer_.size()) {
            to_remove[idx] = true;
        }
    }

    std::vector<RayMeasurement> kept;
    kept.reserve(buffer_.size() - indices.size());
    for (size_t i = 0; i < buffer_.size(); ++i) {
        if (!to_remove[i]) {
            kept.push_back(buffer_[i]);
        }
    }
    buffer_ = std::move(kept);
}

void MeasurementBuffer::clear() {
    buffer_.clear();
}

const std::vector<RayMeasurement>& MeasurementBuffer::rays() const {
    return buffer_;
}

size_t MeasurementBuffer::size() const {
    return buffer_.size();
}

}  // namespace bearing_localization
