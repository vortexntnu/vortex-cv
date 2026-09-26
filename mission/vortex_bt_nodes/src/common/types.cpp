#include "vortex_bt_nodes/common/types.hpp"

#include <utility>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex_bt_nodes {

namespace {
using vortex_msgs::msg::LandmarkSubtype;
using vortex_msgs::msg::LandmarkType;

// Keep in step with LandmarkType.msg and LandmarkSubtype.msg.
using NameValue = std::pair<std::string_view, std::uint16_t>;

constexpr NameValue kTypes[] = {
    {"ARUCO_MARKER", LandmarkType::ARUCO_MARKER},
    {"ARUCO_BOARD", LandmarkType::ARUCO_BOARD},
    {"PIPELINE_START", LandmarkType::PIPELINE_START},
    {"PIPELINE_END", LandmarkType::PIPELINE_END},
    {"VALVE", LandmarkType::VALVE},
    {"GATE", LandmarkType::GATE},
    {"SLALOM_PIPE", LandmarkType::SLALOM_PIPE},
    {"TORPEDO_BOARD", LandmarkType::TORPEDO_BOARD},
    {"BIN", LandmarkType::BIN},
    {"PATH_MARKER", LandmarkType::PATH_MARKER},
    {"TABLE", LandmarkType::TABLE},
    {"OCTAGON", LandmarkType::OCTAGON},
    {"PINGER", LandmarkType::PINGER},
};

constexpr NameValue kSubtypes[] = {
    {"ARUCO_BOARD_CAMERA", LandmarkSubtype::ARUCO_BOARD_CAMERA},
    {"ARUCO_BOARD_SONAR", LandmarkSubtype::ARUCO_BOARD_SONAR},
    {"ARUCO_BOARD_DETECTION", LandmarkSubtype::ARUCO_BOARD_DETECTION},
    {"VALVE_VERTICAL", LandmarkSubtype::VALVE_VERTICAL},
    {"VALVE_HORIZONTAL", LandmarkSubtype::VALVE_HORIZONTAL},
    {"PIPELINE_START_CAMERA", LandmarkSubtype::PIPELINE_START_CAMERA},
    {"PIPELINE_START_SONAR", LandmarkSubtype::PIPELINE_START_SONAR},
    {"GATE_SEARCH_RESCUE", LandmarkSubtype::GATE_SEARCH_RESCUE},
    {"GATE_SURVEY_REPAIR", LandmarkSubtype::GATE_SURVEY_REPAIR},
    {"SLALOM_PIPE_WHITE", LandmarkSubtype::SLALOM_PIPE_WHITE},
    {"SLALOM_PIPE_RED", LandmarkSubtype::SLALOM_PIPE_RED},
    {"TORPEDO_BOARD_WHOLE", LandmarkSubtype::TORPEDO_BOARD_WHOLE},
    {"TORPEDO_TARGET_LARGE_SEARCH_RESCUE",
     LandmarkSubtype::TORPEDO_TARGET_LARGE_SEARCH_RESCUE},
    {"TORPEDO_TARGET_LARGE_SURVEY_REPAIR",
     LandmarkSubtype::TORPEDO_TARGET_LARGE_SURVEY_REPAIR},
    {"TORPEDO_TARGET_SMALL_SEARCH_RESCUE",
     LandmarkSubtype::TORPEDO_TARGET_SMALL_SEARCH_RESCUE},
    {"TORPEDO_TARGET_SMALL_SURVEY_REPAIR",
     LandmarkSubtype::TORPEDO_TARGET_SMALL_SURVEY_REPAIR},
    {"BIN_SEARCH_RESCUE", LandmarkSubtype::BIN_SEARCH_RESCUE},
    {"BIN_SURVEY_REPAIR", LandmarkSubtype::BIN_SURVEY_REPAIR},
    {"GATE_WHOLE", LandmarkSubtype::GATE_WHOLE},
    {"GATE_POLE_EDGE", LandmarkSubtype::GATE_POLE_EDGE},
    {"GATE_POLE_MIDDLE", LandmarkSubtype::GATE_POLE_MIDDLE},
    {"TORPEDO_ICON_FIRE", LandmarkSubtype::TORPEDO_ICON_FIRE},
    {"TORPEDO_ICON_BLOOD", LandmarkSubtype::TORPEDO_ICON_BLOOD},
    {"TORPEDO_ICON_FIRETRUCK", LandmarkSubtype::TORPEDO_ICON_FIRETRUCK},
    {"TORPEDO_ICON_AMBULANCE", LandmarkSubtype::TORPEDO_ICON_AMBULANCE},
    {"BIN_UNCLASSIFIED", LandmarkSubtype::BIN_UNCLASSIFIED},
    {"BIN_STRUCTURE", LandmarkSubtype::BIN_STRUCTURE},
    {"PATH_MARKER_WHOLE", LandmarkSubtype::PATH_MARKER_WHOLE},
    {"TABLE_WHOLE", LandmarkSubtype::TABLE_WHOLE},
    {"TABLE_ITEM_NUTBOLT", LandmarkSubtype::TABLE_ITEM_NUTBOLT},
    {"TABLE_ITEM_ELECTRIC", LandmarkSubtype::TABLE_ITEM_ELECTRIC},
    {"TABLE_ITEM_PILL", LandmarkSubtype::TABLE_ITEM_PILL},
    {"TABLE_ITEM_BANDAID", LandmarkSubtype::TABLE_ITEM_BANDAID},
    {"TABLE_BASKET_SURVEY_REPAIR", LandmarkSubtype::TABLE_BASKET_SURVEY_REPAIR},
    {"TABLE_BASKET_SEARCH_RESCUE", LandmarkSubtype::TABLE_BASKET_SEARCH_RESCUE},
    {"OCTAGON_WHOLE", LandmarkSubtype::OCTAGON_WHOLE},
    {"OCTAGON_IMAGE_REPAIR", LandmarkSubtype::OCTAGON_IMAGE_REPAIR},
    {"OCTAGON_IMAGE_RESCUE", LandmarkSubtype::OCTAGON_IMAGE_RESCUE},
    {"OCTAGON_IMAGE_SEARCH", LandmarkSubtype::OCTAGON_IMAGE_SEARCH},
    {"OCTAGON_IMAGE_SURVEY", LandmarkSubtype::OCTAGON_IMAGE_SURVEY},
    {"PINGER_DEPLOY", LandmarkSubtype::PINGER_DEPLOY},
    {"PINGER_RESTORE", LandmarkSubtype::PINGER_RESTORE},
};
}  // namespace

std::optional<std::uint16_t> landmark_type_from_string(std::string_view name) {
    for (const auto& [key, value] : kTypes) {
        if (key == name) {
            return value;
        }
    }
    return std::nullopt;
}

std::optional<int> landmark_subtype_from_string(std::string_view name) {
    if (name == "ANY") {
        return -1;
    }
    for (const auto& [key, value] : kSubtypes) {
        if (key == name) {
            return value;
        }
    }
    return std::nullopt;
}

}  // namespace vortex_bt_nodes
