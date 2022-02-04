#pragma once

#include <cmath>
#include <string>
#include <memory>
#include <chrono>
#include <set>
#include <vector>
#include <climits>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rcutils/logging_macros.h"

// #include "realtime_tools/realtime_publisher.h"

#include "ros2_qualisys_driver/RTProtocol.h"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace qualisys_driver
{
using namespace Eigen;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class QualisysDriverNode : public rclcpp_lifecycle::LifecycleNode
{

public:
  explicit QualisysDriverNode(const rclcpp::NodeOptions & options);

  explicit QualisysDriverNode(
      const std::string & node_name,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:


  void create_timer_callback();

  bool get_rt_packet();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  // Address of the server
  std::string server_address_;
  // Port of the server to be connected

  int base_port_;

  // Update period of the qualisys system
  std::chrono::milliseconds update_period_;

  // Port that the server should stream UDP packets to.
  // 0 indicates that TCP is used.
  int udp_port_;

  // Minor version of the QTM protocol
  int minor_protocol_version_;

  // Major protocol version is always 1, so only the minor version can be set
  const int major_protocol_version_{1};
  
  std::map<std::string, bool> is_subjects_tracked_;

  // Protocol to connect to the server
  CRTProtocol port_protocol_;

  // A pointer to the received packet
  // (no need to initialize)
  CRTPacket* prt_packet_;


  std::map<std::string, std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>> qualisys_pose_pubs_;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry odometry_message_;
};

}
