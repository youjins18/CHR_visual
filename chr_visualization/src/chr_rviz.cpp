#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace {
constexpr char kWorldFrame[] = "world_enu";
constexpr char kBaseFrame[] = "palletrone";
constexpr char kDesiredBaseFrame[] = "desired_position";
constexpr std::size_t kMinimumFlightLogLength = 66;

bool finite(double value) { return std::isfinite(value); }

bool finite(const tf2::Vector3 &value) {
  return finite(value.x()) && finite(value.y()) && finite(value.z());
}

bool finite(const tf2::Quaternion &value) {
  return finite(value.x()) && finite(value.y()) && finite(value.z()) &&
         finite(value.w()) && value.length2() > 1e-12;
}

tf2::Vector3 ned_to_enu(const tf2::Vector3 &ned) {
  return {ned.y(), ned.x(), -ned.z()};
}

tf2::Vector3 frd_to_flu(const tf2::Vector3 &frd) {
  return {frd.x(), -frd.y(), -frd.z()};
}

tf2::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  quaternion.normalize();
  return quaternion;
}

tf2::Quaternion ned_frd_to_enu_flu(const tf2::Quaternion &input) {
  tf2::Quaternion ned_from_frd = input;
  ned_from_frd.normalize();
  const tf2::Matrix3x3 enu_from_ned(
    0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0);
  const tf2::Matrix3x3 frd_from_flu(
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0);
  const tf2::Matrix3x3 enu_from_flu =
    enu_from_ned * tf2::Matrix3x3(ned_from_frd) * frd_from_flu;
  tf2::Quaternion result;
  enu_from_flu.getRotation(result);
  result.normalize();
  return result;
}

geometry_msgs::msg::Point point_message(const tf2::Vector3 &value) {
  geometry_msgs::msg::Point result;
  result.x = value.x();
  result.y = value.y();
  result.z = value.z();
  return result;
}

geometry_msgs::msg::Quaternion quaternion_message(const tf2::Quaternion &value) {
  geometry_msgs::msg::Quaternion result;
  result.x = value.x();
  result.y = value.y();
  result.z = value.z();
  result.w = value.w();
  return result;
}

visualization_msgs::msg::Marker marker_base(
    const rclcpp::Time &stamp, const std::string &frame,
    const std::string &name_space, int type) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame;
  marker.ns = name_space;
  marker.id = 0;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

visualization_msgs::msg::Marker arrow_marker(
    const rclcpp::Time &stamp, const std::string &name_space,
    const tf2::Vector3 &start, const tf2::Vector3 &end,
    float red, float green, float blue) {
  auto marker = marker_base(
    stamp, kWorldFrame, name_space, visualization_msgs::msg::Marker::ARROW);
  marker.points = {point_message(start), point_message(end)};
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  return marker;
}

visualization_msgs::msg::Marker sphere_marker(
    const rclcpp::Time &stamp, const std::string &name_space,
    const tf2::Vector3 &position, float red, float green, float blue) {
  auto marker = marker_base(
    stamp, kBaseFrame, name_space, visualization_msgs::msg::Marker::SPHERE);
  marker.pose.position = point_message(position);
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  return marker;
}
}  // namespace

class ChrRviz final : public rclcpp::Node {
 public:
  ChrRviz() : Node("chr_rviz") {
    visual_rate_hz_ = declare_parameter("visual_rate_hz", 30.0);
    preview_without_data_ = declare_parameter("preview_without_data", true);
    preview_base_height_m_ = declare_parameter("preview_base_height_m", 1.2);
    if (visual_rate_hz_ <= 0.0 || !std::isfinite(preview_base_height_m_)) {
      throw std::runtime_error("invalid visualization parameter");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    const auto sensor_qos = rclcpp::SensorDataQoS();
    flight_log_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/data_logging_msg", sensor_qos,
      std::bind(&ChrRviz::receive_flight_log, this, std::placeholders::_1));
    j1_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/j1_angle", sensor_qos,
      std::bind(&ChrRviz::receive_j1_angle, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/chr/visualization/joint_states", sensor_qos);
    flight_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/chr/visualization/flight_markers", 10);
    const auto period = std::chrono::duration<double>(1.0 / visual_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ChrRviz::publish_visualization, this));

    RCLCPP_INFO(
      get_logger(),
      "CHR GSC visualization ready: ENU/FLU, read-only, no control publishers");
  }

 private:
  void receive_flight_log(
      const std_msgs::msg::Float32MultiArray::SharedPtr message) {
    if (message->data.size() < kMinimumFlightLogLength) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "data_logging_msg has %zu fields; at least %zu are required",
        message->data.size(), kMinimumFlightLogLength);
      return;
    }
    const auto &data = message->data;
    const auto vector_at = [&data](std::size_t index) {
      return tf2::Vector3(data[index], data[index + 1], data[index + 2]);
    };

    const tf2::Vector3 base_position = ned_to_enu(vector_at(0));
    const tf2::Vector3 desired_position = ned_to_enu(vector_at(3));
    const tf2::Quaternion base_orientation = ned_frd_to_enu_flu(
      quaternion_from_rpy(data[12], data[13], data[14]));
    const tf2::Quaternion desired_orientation = ned_frd_to_enu_flu(
      quaternion_from_rpy(data[15], data[16], data[17]));
    const tf2::Vector3 linear_velocity = ned_to_enu(vector_at(6));
    const tf2::Vector3 desired_linear_velocity = ned_to_enu(vector_at(9));
    const tf2::Vector3 angular_velocity = tf2::quatRotate(
      base_orientation, frd_to_flu(vector_at(18)));
    const tf2::Vector3 desired_angular_velocity = tf2::quatRotate(
      desired_orientation, frd_to_flu(vector_at(21)));
    const tf2::Vector3 acceleration = tf2::quatRotate(
      base_orientation, frd_to_flu(vector_at(48))) + tf2::Vector3(0.0, 0.0, -9.81);
    const tf2::Vector3 desired_acceleration = ned_to_enu(vector_at(51));
    const tf2::Vector3 raw_torque(data[27], data[28], data[29] + data[30]);
    const tf2::Vector3 desired_torque = tf2::quatRotate(
      base_orientation, frd_to_flu(raw_torque));
    const tf2::Vector3 com_update = frd_to_flu(vector_at(63));

    if (!finite(base_position) || !finite(desired_position) ||
        !finite(base_orientation) || !finite(desired_orientation) ||
        !finite(linear_velocity) || !finite(desired_linear_velocity) ||
        !finite(angular_velocity) || !finite(desired_angular_velocity) ||
        !finite(acceleration) || !finite(desired_acceleration) ||
        !finite(desired_torque) || !finite(com_update)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "non-finite flight log sample ignored");
      return;
    }

    base_position_ = base_position;
    desired_base_position_ = desired_position;
    base_orientation_ = base_orientation;
    desired_base_orientation_ = desired_orientation;
    linear_velocity_ = linear_velocity;
    desired_linear_velocity_ = desired_linear_velocity;
    angular_velocity_ = angular_velocity;
    desired_angular_velocity_ = desired_angular_velocity;
    acceleration_ = acceleration;
    desired_acceleration_ = desired_acceleration;
    desired_torque_ = desired_torque;
    com_update_ = com_update;
    for (std::size_t index = 0; index < tilt_position_.size(); ++index) {
      if (std::isfinite(data[38 + index])) tilt_position_[index] = data[38 + index];
    }
    flight_log_received_ = true;
  }

  void receive_j1_angle(const std_msgs::msg::Float32::SharedPtr message) {
    if (!std::isfinite(message->data)) return;
    j1_angle_ = message->data;
    j1_angle_received_ = true;
  }

  void publish_transform(
      const rclcpp::Time &stamp, const std::string &child,
      const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = kWorldFrame;
    transform.child_frame_id = child;
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation = quaternion_message(orientation);
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_joint_state(const rclcpp::Time &stamp) {
    if (!preview_without_data_ && !flight_log_received_ && !j1_angle_received_) return;
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.name = {
      "rotor_0_tilt_joint", "rotor_1_tilt_joint",
      "rotor_2_tilt_joint", "rotor_3_tilt_joint", "J1"};
    message.position = {
      tilt_position_[0], tilt_position_[1], tilt_position_[2], tilt_position_[3],
      j1_angle_};
    joint_state_pub_->publish(message);
  }

  void publish_flight_markers(const rclcpp::Time &stamp) {
    if (!flight_log_received_) return;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(arrow_marker(
      stamp, "linear_velocity", base_position_,
      base_position_ + linear_velocity_, 1.0F, 0.0F, 0.0F));
    markers.markers.push_back(arrow_marker(
      stamp, "desired_linear_velocity", base_position_,
      base_position_ + desired_linear_velocity_, 0.0F, 1.0F, 0.0F));
    markers.markers.push_back(arrow_marker(
      stamp, "acceleration", base_position_,
      base_position_ + acceleration_, 1.0F, 0.0F, 0.0F));
    markers.markers.push_back(arrow_marker(
      stamp, "desired_acceleration", base_position_,
      base_position_ + 0.1 * desired_acceleration_, 0.0F, 1.0F, 0.0F));
    markers.markers.push_back(arrow_marker(
      stamp, "angular_velocity", base_position_,
      base_position_ + angular_velocity_, 1.0F, 0.0F, 0.0F));
    markers.markers.push_back(arrow_marker(
      stamp, "desired_angular_velocity", base_position_,
      base_position_ + 50.0 * desired_angular_velocity_, 0.0F, 1.0F, 0.0F));
    markers.markers.push_back(arrow_marker(
      stamp, "desired_torque", base_position_,
      base_position_ + desired_torque_, 0.0F, 1.0F, 0.0F));
    markers.markers.push_back(sphere_marker(
      stamp, "com_default_update", tf2::Vector3(0.0, 0.0, 0.0),
      0.0F, 0.0F, 1.0F));
    markers.markers.push_back(sphere_marker(
      stamp, "com_update", 10.0 * com_update_, 1.0F, 0.0F, 0.0F));
    flight_marker_pub_->publish(markers);
  }

  void publish_visualization() {
    const auto stamp = now();
    if (flight_log_received_) {
      publish_transform(stamp, kBaseFrame, base_position_, base_orientation_);
      publish_transform(
        stamp, kDesiredBaseFrame, desired_base_position_, desired_base_orientation_);
    } else if (preview_without_data_) {
      publish_transform(
        stamp, kBaseFrame, tf2::Vector3(0.0, 0.0, preview_base_height_m_),
        tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no data_logging_msg: showing the Z-up preview pose at %.2f m",
        preview_base_height_m_);
    }
    if (!j1_angle_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no j1_angle: showing J1=0 preview value");
    }
    publish_joint_state(stamp);
    publish_flight_markers(stamp);
  }

  double visual_rate_hz_{30.0};
  bool preview_without_data_{true};
  double preview_base_height_m_{1.2};
  bool flight_log_received_{false};
  bool j1_angle_received_{false};

  tf2::Vector3 base_position_{0.0, 0.0, 0.0};
  tf2::Vector3 desired_base_position_{0.0, 0.0, 0.0};
  tf2::Quaternion base_orientation_{0.0, 0.0, 0.0, 1.0};
  tf2::Quaternion desired_base_orientation_{0.0, 0.0, 0.0, 1.0};
  tf2::Vector3 linear_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 desired_linear_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 angular_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 desired_angular_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 acceleration_{0.0, 0.0, 0.0};
  tf2::Vector3 desired_acceleration_{0.0, 0.0, 0.0};
  tf2::Vector3 desired_torque_{0.0, 0.0, 0.0};
  tf2::Vector3 com_update_{0.0, 0.0, 0.0};

  double j1_angle_{0.0};
  std::array<double, 4> tilt_position_{0.0, 0.0, 0.0, 0.0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr flight_log_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j1_angle_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr flight_marker_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ChrRviz>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("chr_rviz"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
