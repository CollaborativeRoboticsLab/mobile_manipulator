#include <memory>

#include <mobile_manipulator_ros/mobile_manipulator_server.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_ros::MobileManipulatorServer>());
  rclcpp::shutdown();
  return 0;
}