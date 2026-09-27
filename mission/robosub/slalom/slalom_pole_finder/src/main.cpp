#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "slalom_pole_finder/slalom_pole_finder_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<slalom_pole_finder::SlalomPoleFinderNode>());
  rclcpp::shutdown();
  return 0;
}
