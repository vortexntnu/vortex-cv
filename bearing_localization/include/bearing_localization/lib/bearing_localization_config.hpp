#ifndef BEARING_LOCALIZATION__LIB__BEARING_LOCALIZATION_CONFIG_HPP_
#define BEARING_LOCALIZATION__LIB__BEARING_LOCALIZATION_CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace bearing_localization {

/**
 * @brief Configuration parameters for the bearing localization triangulator.
 *
 * Holds algorithm settings that control the measurement buffer,
 * geometry checks, outlier rejection, and output landmark.
 * Loaded from a YAML profile file via from_yaml().
 */
struct BearingLocalizationConfig {
    int window_size = 30;  // Max measurements kept in the buffer.
    double max_measurement_age_sec =
        5.0;                   // Discard measurements older than this (s).
    int min_measurements = 5;  // Min measurements required to triangulate.
    double min_baseline_m =
        0.5;  // Min distance between first and last origin (m).
    double min_ray_angle_deg =
        5.0;  // Min angle between rays to accept a solution (deg).
    double outlier_residual_threshold_m =
        1.0;  // Residual above this marks a measurement as outlier (m).
    int max_outlier_iterations =
        2;  // Max rounds of iterative outlier rejection.
    /**
     * @brief Load configuration from a YAML file.
     * @param path Filesystem path to the YAML profile.
     * @return Populated BearingLocalizationConfig.
     * @throws std::runtime_error if the file cannot be loaded.
     */
    static BearingLocalizationConfig from_yaml(const std::string& path) {
        YAML::Node yaml;
        try {
            yaml = YAML::LoadFile(path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to load config from '" + path +
                                     "': " + e.what());
        }

        BearingLocalizationConfig cfg;
        if (yaml["window_size"])
            cfg.window_size = yaml["window_size"].as<int>();
        if (yaml["max_measurement_age_sec"])
            cfg.max_measurement_age_sec =
                yaml["max_measurement_age_sec"].as<double>();
        if (yaml["min_measurements"])
            cfg.min_measurements = yaml["min_measurements"].as<int>();
        if (yaml["min_baseline_m"])
            cfg.min_baseline_m = yaml["min_baseline_m"].as<double>();
        if (yaml["min_ray_angle_deg"])
            cfg.min_ray_angle_deg = yaml["min_ray_angle_deg"].as<double>();
        if (yaml["outlier_residual_threshold_m"])
            cfg.outlier_residual_threshold_m =
                yaml["outlier_residual_threshold_m"].as<double>();
        if (yaml["max_outlier_iterations"])
            cfg.max_outlier_iterations =
                yaml["max_outlier_iterations"].as<int>();
        return cfg;
    }
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__BEARING_LOCALIZATION_CONFIG_HPP_
