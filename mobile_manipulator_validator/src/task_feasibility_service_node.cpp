#include <memory>

#include <mobile_manipulator_validator/task_feasibility_service_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_validator::TaskFeasibilityServiceNode>());
  rclcpp::shutdown();
  return 0;
}