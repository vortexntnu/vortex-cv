#ifndef BEARING_LOCALIZATION__LIB__BEARING_LOCALIZATION_CONFIG_HPP_
#define BEARING_LOCALIZATION__LIB__BEARING_LOCALIZATION_CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace bearing_localization {

struct BearingLocalizationConfig {
    std::string target_frame = "orca/odom";
    int window_size = 30;
    double max_measurement_age_sec = 5.0;
    int min_measurements = 5;
    double min_baseline_m = 0.5;
    double min_ray_angle_deg = 5.0;
    double outlier_residual_threshold_m = 1.0;
    int max_outlier_iterations = 2;
    bool publish_debug_markers = true;
    int landmark_type = 0;
    int landmark_subtype = 0;

    static BearingLocalizationConfig from_yaml(const std::string& path) {
        YAML::Node yaml;
        try {
            yaml = YAML::LoadFile(path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to load config from '" + path +
                                     "': " + e.what());
        }

        BearingLocalizationConfig cfg;
        if (yaml["target_frame"])
            cfg.target_frame = yaml["target_frame"].as<std::string>();
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
        if (yaml["publish_debug_markers"])
            cfg.publish_debug_markers =
                yaml["publish_debug_markers"].as<bool>();
        if (yaml["landmark_type"])
            cfg.landmark_type = yaml["landmark_type"].as<int>();
        if (yaml["landmark_subtype"])
            cfg.landmark_subtype = yaml["landmark_subtype"].as<int>();
        return cfg;
    }
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__BEARING_LOCALIZATION_CONFIG_HPP_
