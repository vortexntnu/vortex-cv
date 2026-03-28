#ifndef BEARING_LOCALIZATION__LIB__MEASUREMENT_BUFFER_HPP_
#define BEARING_LOCALIZATION__LIB__MEASUREMENT_BUFFER_HPP_

#include "bearing_localization/lib/ray_measurement.hpp"

#include <vector>

namespace bearing_localization {

/**
 * @brief Fixed-size sliding-window buffer for bearing ray measurements.
 *
 * Maintains a bounded collection of RayMeasurement entries, automatically
 * evicting the oldest entries when the window is full and pruning
 * measurements that exceed the configured maximum age.
 */
class MeasurementBuffer {
   public:
    /**
     * @brief Construct a MeasurementBuffer.
     * @param window_size Maximum number of measurements to retain.
     * @param max_measurement_age_sec Measurements older than this
     *        relative to the current time are pruned (s).
     */
    MeasurementBuffer(size_t window_size, double max_measurement_age_sec);

    /**
     * @brief Add a new ray measurement to the buffer.
     *
     * If the buffer is full the oldest entry is removed first.
     * @param ray The measurement to insert.
     */
    void add(const RayMeasurement& ray);

    /**
     * @brief Remove ray measurements whose stamp_sec is older than
     *        max_measurement_age_sec before the current time.
     *
     * Any measurement with stamp_sec < (now_sec - max_measurement_age_sec)
     * is discarded.
     *
     * @param now_sec Current time in seconds.
     */
    void prune(double now_sec);

    /**
     * @brief Remove measurements at the given indices (e.g. detected outliers).
     * @param indices Sorted indices to remove.
     */
    void remove_indices(const std::vector<size_t>& indices);

    /// @brief Clear all stored measurements.
    void clear();

    /// @brief Read-only access to the stored rays.
    const std::vector<RayMeasurement>& rays() const;

    /// @brief Number of measurements currently in the buffer.
    size_t size() const;

   private:
    size_t window_size_;
    double max_measurement_age_sec_;
    std::vector<RayMeasurement> buffer_;
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__MEASUREMENT_BUFFER_HPP_
