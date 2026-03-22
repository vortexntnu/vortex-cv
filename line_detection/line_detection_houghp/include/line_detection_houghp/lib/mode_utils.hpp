#ifndef LINE_DETECTION_HOUGHP__LIB__MODE_UTILS_HPP_
#define LINE_DETECTION_HOUGHP__LIB__MODE_UTILS_HPP_

#include <array>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include "line_detection_houghp/lib/typedefs.hpp"

namespace vortex::line_detection {

inline constexpr std::array<std::pair<std::string_view, DetectorMode>, 3>
    kDetectorModeMap{{
        {"standard", DetectorMode::standard},
        {"visualize", DetectorMode::visualize},
        {"debug", DetectorMode::debug},
    }};

inline DetectorMode parse_mode(const std::string& s) {
    for (const auto& [name, mode] : kDetectorModeMap) {
        if (s == name)
            return mode;
    }
    throw std::runtime_error("Invalid DetectorMode: " + s);
}

inline std::string_view to_string(DetectorMode mode) {
    for (const auto& [name, m] : kDetectorModeMap) {
        if (m == mode)
            return name;
    }
    throw std::runtime_error("Unknown DetectorMode");
}

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_HOUGHP__LIB__MODE_UTILS_HPP_
