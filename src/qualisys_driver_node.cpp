#include "ros2_qualisys_driver/qualisys_driver_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace {

std::string sanitize_topic_token(const std::string &name) {
  std::string token;
  token.reserve(name.size());

  for (const unsigned char ch : name) {
    if (std::isalnum(ch) || ch == '_') {
      token.push_back(static_cast<char>(ch));
    } else {
      token.push_back('_');
    }
  }

  if (token.empty()) {
    return "unnamed_body";
  }
  if (!(std::isalpha(static_cast<unsigned char>(token.front())) ||
        token.front() == '_')) {
    token.insert(token.begin(), '_');
  }
  return token;
}

std::string normalize_topic_prefix(std::string prefix) {
  prefix.erase(prefix.begin(),
               std::find_if(prefix.begin(), prefix.end(),
                            [](char c) { return c != '/'; }));
  prefix.erase(std::find_if(prefix.rbegin(), prefix.rend(),
                            [](char c) { return c != '/'; })
                   .base(),
               prefix.end());
  return prefix.empty() ? "qualisys" : prefix;
}

bool matrix_has_non_finite_values(const float matrix[9]) {
  for (unsigned int i = 0; i < 9; ++i) {
    if (!std::isfinite(matrix[i])) {
      return true;
    }
  }
  return false;
}

}  // namespace

namespace qualisys_driver {

QualisysDriverNode::QualisysDriverNode(const rclcpp::NodeOptions &options)
    : QualisysDriverNode("Qualisys Driver", options) {}

QualisysDriverNode::QualisysDriverNode(const std::string &node_name,
                                       const rclcpp::NodeOptions &options)
    : LifecycleNode(node_name, options),
      server_address_{declare_parameter<std::string>("server_address", "127.0.0.1")},
      base_port_(declare_parameter<int>("base_port", 22222)),
      update_period_{std::chrono::milliseconds(
          declare_parameter<int>("update_period_ms", 10))},
      udp_port_(declare_parameter<int>("udp_port", 14570)),
      receive_timeout_us_(declare_parameter<int>("receive_timeout_us", 1000)),
      frame_id_(declare_parameter<std::string>("frame_id", "qualisys")),
      topic_prefix_(normalize_topic_prefix(
          declare_parameter<std::string>("topic_prefix", "qualisys"))),
      minor_protocol_version_(
          declare_parameter<int>("qtm_minor_protocol_version", MINOR_VERSION))
  {
    create_timer_callback();
}

void QualisysDriverNode::create_timer_callback() {
  auto state_timer_callback = [this]() {
    if (!get_rt_packet()) {
      return;
    }
    const unsigned int body_count = prt_packet_->Get6DOFBodyCount();

    for (unsigned int i = 0; i < body_count; i++) {
      const char *subject_name_ptr = port_protocol_.Get6DOFBodyName(i);
      if (subject_name_ptr == nullptr) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Received 6DOF body index %u without matching settings name", i);
        continue;
      }
      const std::string subject_name(subject_name_ptr);
      const std::string sanitized_subject_name =
          sanitize_topic_token(subject_name);

      auto subject_topic_name = subject_topic_names_.find(subject_name);
      if (subject_topic_name == subject_topic_names_.end()) {
        std::string topic_subject_name = sanitized_subject_name;
        unsigned int collision_index = 2;
        while (qualisys_pose_pubs_.find(topic_subject_name) !=
               qualisys_pose_pubs_.end()) {
          topic_subject_name =
              sanitized_subject_name + "_" + std::to_string(collision_index++);
        }
        subject_topic_name =
            subject_topic_names_.emplace(subject_name, topic_subject_name).first;
      }

      const std::string &topic_subject_name = subject_topic_name->second;

      if (qualisys_pose_pubs_.find(topic_subject_name) == qualisys_pose_pubs_.end()) {
        if (topic_subject_name != subject_name) {
          RCLCPP_WARN(get_logger(), "Sanitized subject '%s' to ROS topic token '%s'",
                      subject_name.c_str(), topic_subject_name.c_str());
        }
        RCLCPP_INFO(get_logger(), "New subject: %s ", subject_name.c_str());

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qualisys_pose_pubs_[topic_subject_name] =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                topic_prefix_ + "/" + topic_subject_name + "/pose", qos);

        is_subjects_tracked_[topic_subject_name] = false;
      }

      // Pose of the subject
      const unsigned int matrix_size = 9;
      float x, y, z;
      float rot_array[matrix_size];
      if (!prt_packet_->Get6DOFBody(i, x, y, z, rot_array)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Failed to read 6DOF body index %u", i);
        continue;
      }

      // Check if the subject is tracked by looking for NaN in the received data
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          matrix_has_non_finite_values(rot_array)) {
        if (is_subjects_tracked_[topic_subject_name]) {
          is_subjects_tracked_[topic_subject_name] = false;
          qualisys_pose_pubs_[topic_subject_name]->on_deactivate();
          RCLCPP_WARN(get_logger(), "Lost track of subject: %s ",
                      subject_name.c_str());
        }
        continue;
      }

      // Convert the rotation matrix to a quaternion
      Map<const Matrix<float, 3, 3, ColMajor>> rot_matrix(rot_array);
      Quaterniond quat(rot_matrix.cast<double>());
      quat.normalize();
      // Check if the subject is beeing tracked

      // Convert mm to m
      Vector3d pos(x / 1000.0, y / 1000.0, z / 1000.0);


      if (!is_subjects_tracked_[topic_subject_name]) {
        is_subjects_tracked_[topic_subject_name] = true;
        RCLCPP_INFO(get_logger(), "Tracking subject: %s ", subject_name.c_str());

        qualisys_pose_pubs_[topic_subject_name]->on_activate();
      }

      // const double packet_time = prt_packet_->GetTimeStamp() / 1e6;
      // const auto current_time = this->now();
      const auto current_time = this->get_clock()->now();

      geometry_msgs::msg::PoseStamped pose_message;
      pose_message.header.stamp = current_time;
      pose_message.header.frame_id = frame_id_;
      pose_message.pose.position.x = pos(0);
      pose_message.pose.position.y = pos(1);
      pose_message.pose.position.z = pos(2);
      pose_message.pose.orientation.x = quat.x();
      pose_message.pose.orientation.y = quat.y();
      pose_message.pose.orientation.z = quat.z();
      pose_message.pose.orientation.w = quat.w();

      qualisys_pose_pubs_[topic_subject_name]->publish(pose_message);
    }
  };

  timer_ = this->create_wall_timer(update_period_, state_timer_callback);
  // cancel immediately to prevent triggering it in this state
  timer_->cancel();
}

bool QualisysDriverNode::get_rt_packet() {
  prt_packet_ = port_protocol_.GetRTPacket();
  CRTPacket::EPacketType e_type;
  bool is_ok = false;

  const int received =
      port_protocol_.ReceiveRTPacket(e_type, true, receive_timeout_us_);
  if (received > 0) {
    switch (e_type) {
    // Case 1 - sHeader.nType 0 indicates an error
    case CRTPacket::PacketError:
      RCLCPP_ERROR_STREAM(
          get_logger(), "Error when streaming frames: "
                            << port_protocol_.GetRTPacket()->GetErrorString());
      break;

    // Case 2 - No more data
    case CRTPacket::PacketNoMoreData:
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No more data, check if RT capture is active");
      break;

    // Case 3 - Data received
    case CRTPacket::PacketData:
      is_ok = true;
      break;

    // Case 9 - None type, sent on disconnet
    case CRTPacket::PacketNone:
      break;

    default:
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Unhandled CRTPacket type, case: %i", e_type);
      break;
    }
  } else if (received < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "QTM error when receiving packet:\n%s",
                          port_protocol_.GetErrorString());
  }
  return is_ok;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
QualisysDriverNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");
  // qualisys stuff
  if (server_address_.empty()) {
    RCLCPP_FATAL(get_logger(), "Server_address parameter empty");
    return CallbackReturn::ERROR;
  }
  if (base_port_ <= 0 || base_port_ > USHRT_MAX) {
    RCLCPP_FATAL(get_logger(), "Invalid base port %i", base_port_);
    return CallbackReturn::ERROR;
  }
  if (update_period_.count() <= 0) {
    RCLCPP_FATAL(get_logger(), "update_period_ms must be positive");
    return CallbackReturn::ERROR;
  }
  if (receive_timeout_us_ < 0) {
    RCLCPP_FATAL(get_logger(), "receive_timeout_us must be non-negative");
    return CallbackReturn::ERROR;
  }

  unsigned short udp_stream_port = 0;
  unsigned short *udp_port_ptr = nullptr;
  if (udp_port_ > 0) {
    if (udp_port_ < 1024 || udp_port_ > USHRT_MAX) {
      RCLCPP_FATAL(get_logger(),
                   "Invalid UDP stream port %i. Valid range is 1024-65535, "
                   "or 0/-1 for TCP.",
                   udp_port_);
      return CallbackReturn::ERROR;
    }
    udp_stream_port = static_cast<unsigned short>(udp_port_);
    udp_port_ptr = &udp_stream_port;
  } else if (udp_port_ < -1) {
    RCLCPP_FATAL(get_logger(), "Invalid UDP port %i", udp_port_);
    return CallbackReturn::ERROR;
  }
  // Connecting to the server
  RCLCPP_INFO(get_logger(), "Connecting to QTM server at: %s : %i ",
              server_address_.c_str(), base_port_);

  if (!port_protocol_.Connect((char *)server_address_.data(), base_port_,
                              udp_port_ptr, major_protocol_version_,
                              minor_protocol_version_)) {
    RCLCPP_FATAL_STREAM(get_logger(), "Connection to QTM server at: "
                                          << server_address_ << ":"
                                          << base_port_
                                          << " failed\n"
                                             "Reason: "
                                          << port_protocol_.GetErrorString());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_logger(),
                     "Connected to " << server_address_ << ":" << base_port_);
  if (udp_stream_port > 0) {
    RCLCPP_INFO(get_logger(), "Streaming data to UDP port %i", udp_stream_port);
  }
  // Get 6DOF settings
  bool bDataAvailable = false;
  if (!port_protocol_.Read6DOFSettings(bDataAvailable) || bDataAvailable == false) {
    RCLCPP_FATAL_STREAM(
        get_logger(), "Reading 6DOF body settings failed during initialization\n"
                          << "QTM error: " << port_protocol_.GetErrorString());
    port_protocol_.Disconnect();
    return CallbackReturn::ERROR;
  }
  // Read system settings
  if (!port_protocol_.ReadGeneralSettings()) {
    RCLCPP_FATAL_STREAM(
        get_logger(), "Failed to read system settings during initialization\n"
                          << "QTM error: " << port_protocol_.GetErrorString());
    port_protocol_.Disconnect();
    return CallbackReturn::ERROR;
  }
  // Start streaming data frames
  unsigned int system_frequency = port_protocol_.GetSystemFrequency();
  if (system_frequency == 0) {
    RCLCPP_FATAL(get_logger(), "QTM reported invalid system frequency 0");
    port_protocol_.Disconnect();
    return CallbackReturn::ERROR;
  }
  CRTProtocol::EStreamRate stream_rate_mode =
      CRTProtocol::EStreamRate::RateAllFrames;
  double dt_ = update_period_.count() / (1000.0);
  double frame_rate = 1 / dt_;
  // RCLCPP_INFO(get_logger(), "dt_: %f", dt_);
  if (frame_rate < system_frequency && frame_rate > 0) {
    stream_rate_mode = CRTProtocol::EStreamRate::RateFrequency;
  } else {
    if (frame_rate > system_frequency) {
      RCLCPP_WARN(get_logger(),
                  "Requested capture rate %f larger than current system "
                  "capture rate %i .",
                  frame_rate, system_frequency);
    }
    frame_rate = system_frequency;
  }
  const auto rounded_frame_rate =
      static_cast<unsigned int>(std::round(frame_rate));
  const unsigned int stream_rate_arg =
      stream_rate_mode == CRTProtocol::EStreamRate::RateFrequency
          ? std::max(1u, rounded_frame_rate)
          : rounded_frame_rate;
  if (!port_protocol_.StreamFrames(stream_rate_mode,
                                   stream_rate_arg, // nRateArg
                                   udp_stream_port, // nUDPPort
                                   nullptr,         // nUDPAddr
                                   CRTProtocol::cComponent6d)) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Failed to start QTM frame streaming: "
                            << port_protocol_.GetErrorString());
    port_protocol_.Disconnect();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Frame rate: %f frames per second", frame_rate);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
QualisysDriverNode::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating");
  // qualisys_pub_->on_activate();
  // qualisys_pose_pub_->on_activate();
  timer_->reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
QualisysDriverNode::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  timer_->cancel();
  for (auto &publisher : qualisys_pose_pubs_) {
    if (is_subjects_tracked_[publisher.first]) {
      publisher.second->on_deactivate();
    }
    is_subjects_tracked_[publisher.first] = false;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn QualisysDriverNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  timer_->cancel();
  for (auto &publisher : qualisys_pose_pubs_) {
    if (is_subjects_tracked_[publisher.first]) {
      publisher.second->on_deactivate();
    }
  }
  qualisys_pose_pubs_.clear();
  is_subjects_tracked_.clear();
  subject_topic_names_.clear();
  if (port_protocol_.Connected()) {
    port_protocol_.StreamFramesStop();
    port_protocol_.Disconnect();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
QualisysDriverNode::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  timer_->cancel();
  if (port_protocol_.Connected()) {
    port_protocol_.StreamFramesStop();
    port_protocol_.Disconnect();
    RCLCPP_INFO_STREAM(get_logger(), "Disconnected from the QTM server at "
                                         << server_address_ << ":" << base_port_);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
} // namespace qualisys_driver

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(qualisys_driver::QualisysDriverNode)
