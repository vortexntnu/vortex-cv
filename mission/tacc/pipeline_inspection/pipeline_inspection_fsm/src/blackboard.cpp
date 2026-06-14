#include "pipeline_inspection_fsm/states.hpp"

StateMachineConfig load_config(rclcpp::Node::SharedPtr node) {
    StateMachineConfig config;

    config.waypoint_manager_action_server =
        node->declare_parameter<std::string>("action_servers.waypoint_manager");
    config.landmark_polling_action_server =
        node->declare_parameter<std::string>("action_servers.landmark_polling");
    config.bearing_direction_action_server =
        node->declare_parameter<std::string>(
            "action_servers.bearing_direction");
    config.pipeline_bearing_direction_action_server =
        node->declare_parameter<std::string>(
            "action_servers.pipeline_bearing_direction");
    config.acoustic_short_timeout_sec =
        node->declare_parameter<double>("acoustic_short_timeout_sec", 15.0);
    config.acoustic_bearing_projection_distance =
        node->declare_parameter<double>("acoustic_bearing_projection_distance",
                                        3.0);
    config.acoustic_short_min_measurements =
        node->declare_parameter<int>("acoustic_short_min_measurements", 10);
    config.acoustic_short_max_measurements =
        node->declare_parameter<int>("acoustic_short_max_measurements", 15);

    config.acoustic_long_timeout_sec =
        node->declare_parameter<double>("acoustic_long_timeout_sec", 15.0);
    config.acoustic_search_projection_distance =
        node->declare_parameter<double>("acoustic_search_projection_distance",
                                        10.0);
    config.acoustic_long_min_measurements =
        node->declare_parameter<int>("acoustic_long_min_measurements", 10);
    config.acoustic_long_max_measurements =
        node->declare_parameter<int>("acoustic_long_max_measurements", 15);

    config.pipeline_bearing_timeout_sec =
        node->declare_parameter<double>("pipeline_bearing_timeout_sec", 15.0);
    config.pipeline_bearing_projection_distance =
        node->declare_parameter<double>("pipeline_bearing_projection_distance",
                                        5.0);
    config.pipeline_bearing_min_measurements =
        node->declare_parameter<int>("pipeline_bearing_min_measurements", 10);
    config.pipeline_bearing_max_measurements =
        node->declare_parameter<int>("pipeline_bearing_max_measurements", 15);

    config.acoustic_bearing_waypoint_altitude = node->declare_parameter<double>(
        "acoustic_bearing_waypoint_altitude", 2.0);
    config.bearing_waypoint_altitude =
        node->declare_parameter<double>("bearing_waypoint_altitude", 1.0);
    config.return_to_origin_altitude =
        node->declare_parameter<double>("return_to_origin_altitude", 2.0);
    config.start_mission_service =
        node->declare_parameter<std::string>("services.start_mission");
    config.start_pipeline_following_service =
        node->declare_parameter<std::string>(
            "services.start_pipeline_following");
    config.start_end_pipeline_detection_service =
        node->declare_parameter<std::string>(
            "services.start_end_pipeline_detection");
    config.end_of_pipeline_service =
        node->declare_parameter<std::string>("services.end_of_pipeline");
    config.irls_line_detected_service =
        node->declare_parameter<std::string>("services.irls_line_detected");
    config.odom_topic = node->declare_parameter<std::string>(
        "topics.odom", "/nautilus/nucleus/odom");
    config.start_in_camera_range =
        node->declare_parameter<bool>("start_in_camera_range", false);
    config.start_above_pipe =
        node->declare_parameter<bool>("start_above_pipe", false);
    config.enable_end_detection =
        node->declare_parameter<bool>("enable_end_detection", true);

    return config;
}

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& /*config*/) {
    return std::make_shared<yasmin::Blackboard>();
}
