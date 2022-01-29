#include "ros2_qualisys_driver/qualisys_driver_node.hpp"

namespace qualisys_driver {

QualisysDriverNode::QualisysDriverNode(const rclcpp::NodeOptions &options)
    : QualisysDriverNode("Qualisys Driver", options) {}

QualisysDriverNode::QualisysDriverNode(const std::string &node_name,
                                       const rclcpp::NodeOptions &options)
    : LifecycleNode(node_name, options),
      server_address_(declare_parameter("server_address").get<std::string>()),
      base_port_(declare_parameter("base_port").get<int>()),
      update_period_{std::chrono::milliseconds{
          declare_parameter("update_period_ms").get<std::uint32_t>()}},
      udp_port_(declare_parameter("udp_port").get<int>()),
      minor_protocol_version_(
          declare_parameter("qtm_minor_protocol_version").get<int>()),
      subject_name_(declare_parameter("subject_name").get<std::string>()) {
  create_qualisys_publisher();
  create_timer_callback();
}

void QualisysDriverNode::create_qualisys_publisher() {
  qualisys_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/qualisys/" + subject_name_ + "/odom", rclcpp::QoS(1));

  realtime_qualisys_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      qualisys_pub_);

  auto &odometry_message = realtime_qualisys_pub_->msg_;
  odometry_message.header.frame_id = "odom";
  odometry_message.child_frame_id = "base_link";
}

void QualisysDriverNode::create_timer_callback() {
  auto state_timer_callback = [this]() {
    if (!get_rt_packet()) {
      return;
    }
    int body_count = prt_packet_->Get6DOFBodyCount();
    int subject_id = -1;

    for (int i = 0; i < body_count; i++) {
      if (subject_name_ == port_protocol_.Get6DOFBodyName(i)) {
        subject_id = i;
      }
    }

    if (subject_id == -1) {
      is_subject_tracked_ = false;
      RCLCPP_WARN(get_logger(),
                  "Specified subject name: %s is not tracked by QTM!",
                  subject_name_.c_str());
      return;
    }

    // Pose of the subject
    const unsigned int matrix_size = 9;
    float x, y, z;
    float rot_array[matrix_size];
    prt_packet_->Get6DOFBody(subject_id, x, y, z, rot_array);

    // Check if the subject is tracked by looking for NaN in the received data
    bool nan_in_matrix = false;
    for (unsigned int i = 0; i < matrix_size; i++) {
      if (std::isnan(rot_array[i])) {
        nan_in_matrix = true;
        break;
      }
    }
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || nan_in_matrix) {
      if (is_subject_tracked_) {
        is_subject_tracked_ = false;
        RCLCPP_WARN(get_logger(), "Lost track of subject: %s ",
                    subject_name_.c_str());
      }
      return;
    }

    // Convert the rotation matrix to a quaternion
    Matrix<float, 3, 3> rot_matrix(rot_array);
    Quaterniond quat(rot_matrix.cast<double>());
    // Check if the subject is beeing tracked

    // Convert mm to m
    Vector3d pos(x / 1000.0, y / 1000.0, z / 1000.0);

    if (!is_subject_tracked_) {
      is_subject_tracked_ = true;
      RCLCPP_INFO(get_logger(), "Tracking subject: %s ", subject_name_.c_str());
    }

    // const double packet_time = prt_packet_->GetTimeStamp() / 1e6;
    // const auto current_time = this->now();
    const auto current_time = this->get_clock()->now();

    if (realtime_qualisys_pub_->trylock()) {
      auto &odometry_message = realtime_qualisys_pub_->msg_;
      odometry_message.header.stamp = current_time;
      odometry_message.pose.pose.position.x = pos(0);
      odometry_message.pose.pose.position.y = pos(1);
      odometry_message.pose.pose.position.z = pos(2);
      odometry_message.pose.pose.orientation.x = quat.x();
      odometry_message.pose.pose.orientation.y = quat.y();
      odometry_message.pose.pose.orientation.z = quat.z();
      odometry_message.pose.pose.orientation.w = quat.w();
      realtime_qualisys_pub_->unlockAndPublish();
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

  if (port_protocol_.ReceiveRTPacket(e_type, true)) {
    switch (e_type) {
    // Case 1 - sHeader.nType 0 indicates an error
    case CRTPacket::PacketError:
      RCLCPP_ERROR_STREAM(
          get_logger(), "Error when streaming frames: "
                            << port_protocol_.GetRTPacket()->GetErrorString());
      break;

    // Case 2 - No more data
    case CRTPacket::PacketNoMoreData:
      RCLCPP_ERROR_STREAM(get_logger(),
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
      RCLCPP_WARN(get_logger(), "Unhandled CRTPacket type, case: %i", e_type);
      break;
    }
  } else { //
    RCLCPP_ERROR_STREAM(get_logger(), "QTM error when receiving packet:\n"
                                          << port_protocol_.GetErrorString());
  }
  return is_ok;
}

CallbackReturn
QualisysDriverNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");
  // qualisys stuff
  if (server_address_.empty()) {
    RCLCPP_FATAL(get_logger(), "Server_address parameter empty");
    return CallbackReturn::ERROR;
  }

  unsigned short udp_stream_port = 0;
  unsigned short *udp_port_ptr = nullptr;
  if (udp_port_ >= 0 && udp_port_ < USHRT_MAX) {
    udp_stream_port = static_cast<unsigned short>(udp_port_);
    udp_port_ptr = &udp_stream_port;
  } else if (udp_port_ < -1 || udp_port_ > USHRT_MAX) {
    RCLCPP_WARN(get_logger(), "Invalid UDP port %i, falling back to TCP",
                udp_port_);
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
  port_protocol_.Read6DOFSettings(bDataAvailable);
  if (bDataAvailable == false) {
    RCLCPP_FATAL_STREAM(
        get_logger(), "Reading 6DOF body settings failed during intialization\n"
                          << "QTM error: " << port_protocol_.GetErrorString());
    return CallbackReturn::ERROR;
  }
  // Read system settings
  if (!port_protocol_.ReadGeneralSettings()) {
    RCLCPP_FATAL_STREAM(
        get_logger(), "Failed to read system settings during intialization\n"
                          << "QTM error: " << port_protocol_.GetErrorString());
    return CallbackReturn::ERROR;
  }
  // Start streaming data frames
  int system_frequency = port_protocol_.GetSystemFrequency();
  CRTProtocol::EStreamRate stream_rate_mode =
      CRTProtocol::EStreamRate::RateAllFrames;
  double dt = 1.0 / (double)system_frequency;
  double dt_ = update_period_.count() / (1000.0);
  double frame_rate = 1 / dt_;
  // RCLCPP_INFO(get_logger(), "dt_: %f", dt_);
  if (frame_rate < system_frequency && frame_rate > 0) {
    stream_rate_mode = CRTProtocol::EStreamRate::RateFrequency;
    dt = 1 / frame_rate;
  } else {
    if (frame_rate > system_frequency) {
      RCLCPP_WARN(get_logger(),
                  "Requested capture rate %f larger than current system "
                  "capture rate %i .",
                  frame_rate, system_frequency);
    }
    frame_rate = system_frequency;
  }
  bDataAvailable = port_protocol_.StreamFrames(stream_rate_mode,
                                               frame_rate,      // nRateArg
                                               udp_stream_port, // nUDPPort
                                               nullptr,         // nUDPAddr
                                               CRTProtocol::cComponent6d);
  RCLCPP_INFO(get_logger(), "Frame rate: %f frames per second", frame_rate);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
QualisysDriverNode::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating");
  qualisys_pub_->on_activate();
  timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
QualisysDriverNode::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  qualisys_pub_->on_deactivate();
  timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn QualisysDriverNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
QualisysDriverNode::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  RCLCPP_INFO_STREAM(get_logger(), "Disconnected from the QTM server at "
                                       << server_address_ << ":" << base_port_);
  port_protocol_.StreamFramesStop();
  port_protocol_.Disconnect();

  return CallbackReturn::SUCCESS;
}
} // namespace qualisys_driver

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(qualisys_driver::QualisysDriverNode)