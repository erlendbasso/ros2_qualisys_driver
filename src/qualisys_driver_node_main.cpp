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
          "qualisys_driver");

  std::thread cm_thread([qualisys_node]() {
    RCLCPP_INFO(qualisys_node->get_logger(), "update rate is Hz");

    rclcpp::Time begin = qualisys_node->now();

    // Use nanoseconds to avoid chrono's rounding
    std::this_thread::sleep_for(std::chrono::nanoseconds(
        1000000000 / qualisys_node->get_update_rate()));
    while (rclcpp::ok()) {
      rclcpp::Time begin_last = begin;
      begin = qualisys_node->now();

      qualisys_node->publish_pose();

      rclcpp::Time end = qualisys_node->now();
      std::this_thread::sleep_for(
          std::max(std::chrono::nanoseconds(0),
                   std::chrono::nanoseconds(1000000000 /
                                            qualisys_node->get_update_rate()) -
                       std::chrono::nanoseconds(end.nanoseconds() -
                                                begin.nanoseconds())));
    }
  });

  executor.add_node(qualisys_node->get_node_base_interface());

  executor.spin();

  cm_thread.join();

  rclcpp::shutdown();

  return 0;
}