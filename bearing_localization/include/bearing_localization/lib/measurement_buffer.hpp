#ifndef BEARING_LOCALIZATION__LIB__MEASUREMENT_BUFFER_HPP_
#define BEARING_LOCALIZATION__LIB__MEASUREMENT_BUFFER_HPP_

#include "bearing_localization/lib/ray_measurement.hpp"

#include <vector>

namespace bearing_localization {

class MeasurementBuffer {
   public:
    MeasurementBuffer(size_t window_size, double max_age_sec);

    void add(const RayMeasurement& ray);

    void prune(double now_sec);

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

#endif  // BEARING_LOCALIZATION__LIB__MEASUREMENT_BUFFER_HPP_
