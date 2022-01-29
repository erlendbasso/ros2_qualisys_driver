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
//  pendulum::utils::ProcessSettings settings;
//     if (!settings.init(argc, argv))
//     {
//         return EXIT_FAILURE;
//     }

//     int32_t ret = 0;
//     try
//     {
//         // configure process real-time settings
//         if (settings.configure_child_threads)
//         {
//             // process child threads created by ROS nodes will inherit the settings
//             settings.configure_process();
//         }
//         rclcpp::init(argc, argv);

//         // Create a static executor
//         rclcpp::executors::StaticSingleThreadedExecutor exec;

//         // Create qualisys driver node
//         const auto qualisys_driver_ptr =
//             std::make_shared<qualisys_driver::QualisysDriverNode>("qualisys_driver");

//         exec.add_node(qualisys_driver_ptr->get_node_base_interface());

//         // configure process real-time settings
//         if (!settings.configure_child_threads)
//         {
//             // process child threads created by ROS nodes will NOT inherit the settings
//             settings.configure_process();
//         }

//         if (settings.auto_start_nodes)
//         {
//             pendulum::utils::autostart(*qualisys_driver_ptr);
//         }

//         exec.spin();
//         rclcpp::shutdown();
//     }
//     catch (const std::exception &e)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("qualisys_driver"), e.what());
//         ret = 2;
//     }
//     catch (...)
//     {
//         RCLCPP_INFO(
//             rclcpp::get_logger("qualisys_driver"), "Unknown exception caught. "
//                                                  "Exiting...");
//         ret = -1;
//     }
//     return ret;

  rclcpp::init(argc, argv);
  rclcpp::executors::StaticSingleThreadedExecutor exec;

  const auto qualisys_driver_ptr =
                  std::make_shared<qualisys_driver::QualisysDriverNode>("qualisys_driver");

  exec.add_node(qualisys_driver_ptr->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();

  return 0;
}