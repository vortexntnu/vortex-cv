#include "pipeline_inspection_fsm/states.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

std::shared_ptr<yasmin::Blackboard> initialize_blackboard() {
    auto params = rclcpp::Node::make_shared("pipeline_inspection_params");

    const std::string wm_action = params->declare_parameter<std::string>(
        "action_servers.waypoint_manager");
    const std::string lp_action = params->declare_parameter<std::string>(
        "action_servers.landmark_polling");
    const std::string start_service =
        params->declare_parameter<std::string>("services.start_mission");
    const std::string pipeline_start_service =
        params->declare_parameter<std::string>(
            "services.start_pipeline_following");
    const std::string pipeline_end_service =
        params->declare_parameter<std::string>("services.end_of_pipeline");
    const std::string pkg_dir =
        ament_index_cpp::get_package_share_directory("pipeline_inspection_fsm");

    auto bb = std::make_shared<yasmin::Blackboard>();

    bb->set<std::string>("action_server.waypoint_manager", wm_action);
    bb->set<std::string>("action_server.landmark_polling", lp_action);
    bb->set<std::string>("service.start_mission", start_service);
    bb->set<std::string>("service.start_pipeline_following",
                         pipeline_start_service);
    bb->set<std::string>("service.end_of_pipeline", pipeline_end_service);
    bb->set<std::string>("waypoint_file_path",
                         pkg_dir + "/config/search_waypoints.yaml");
    bb->set<std::string>("convergence_file_path",
                         pkg_dir + "/config/pipeline_convergence.yaml");

    return bb;
}
