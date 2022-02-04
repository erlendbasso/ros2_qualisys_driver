#include <memory>
#include <utility>
#include <string>
#include <thread>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
// #include "pendulum_utils/process_settings.hpp"
// #include "pendulum_utils/lifecycle_autostart.hpp"

#include "ros2_qualisys_driver/qualisys_driver_node.hpp"


int main(int argc, char *argv[])
{
  // rclcpp::init(argc, argv);
  // rclcpp::executors::StaticSingleThreadedExecutor exec;

  // const auto qualisys_driver_ptr =
  //                 std::make_shared<qualisys_driver::QualisysDriverNode>("qualisys_driver");

  // exec.add_node(qualisys_driver_ptr->get_node_base_interface());
  // exec.spin();
  // rclcpp::shutdown();

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  const auto qualisys_node =
      std::make_shared<qualisys_driver::QualisysDriverNode>(
          rclcpp::NodeOptions());

  executor.add_node(qualisys_node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}