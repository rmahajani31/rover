#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "custom_imu_propagator/imu_propagator.hpp"
#include "custom_imu_propagator/imu_sample.hpp"

namespace custom_imu_propagator
{

namespace
{

std::string doubleToString(double value)
{
  return std::to_string(value);
}

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string& key,
  const std::string& value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

}  // namespace

class ImuPropagatorNode : public rclcpp::Node
{
public:
  explicit ImuPropagatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("custom_imu_propagator", options)
  {
    declareParameters();
    readParameters();

    propagator_.configure(propagator_options_);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuPropagatorNode::imuCallback, this, std::placeholders::_1));

    if (publish_imu_diagnostics_) {
      diagnostics_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    }

    if (publish_imu_predicted_odom_) {
      RCLCPP_WARN(
        get_logger(),
        "publish_imu_predicted_odom is true, but standalone predicted odometry requires "
        "scan-to-map pose and LiDAR scan timestamps. This node currently buffers and inspects IMU only.");
    }

    RCLCPP_INFO(
      get_logger(),
      "custom_imu_propagator initialized: imu_topic=%s diagnostics_topic=%s",
      imu_topic_.c_str(),
      diagnostics_topic_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("imu_topic", "/livox/imu");
    declare_parameter<std::string>("predicted_odom_topic", "/custom/imu_predicted_odom");
    declare_parameter<std::string>("diagnostics_topic", "/custom/imu_propagator_diagnostics");

    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("imu_frame", "livox_frame");

    declare_parameter<bool>("use_imu_initial_guess", true);
    declare_parameter<bool>("use_imu_rotation", true);
    declare_parameter<bool>("use_imu_translation", false);

    declare_parameter<double>("max_imu_buffer_seconds", 5.0);
    declare_parameter<double>("max_allowed_imu_gap", 0.02);
    declare_parameter<double>("max_expected_yaw_change_deg_per_scan", 30.0);

    declare_parameter<std::vector<double>>("gravity", std::vector<double>{0.0, 0.0, -9.81});
    declare_parameter<std::vector<double>>("gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("accel_bias", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<double>("accel_scale", 9.80665);

    declare_parameter<bool>("publish_imu_predicted_odom", true);
    declare_parameter<bool>("publish_imu_diagnostics", true);
    declare_parameter<double>("diagnostics_publish_rate_hz", 5.0);
    declare_parameter<bool>("publish_live_propagation_diagnostics", true);
    declare_parameter<double>("propagation_window_seconds", 0.1);
    declare_parameter<int>("log_throttle_ms", 1000);
  }

  void readParameters()
  {
    imu_topic_ = get_parameter("imu_topic").as_string();
    predicted_odom_topic_ = get_parameter("predicted_odom_topic").as_string();
    diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    imu_frame_ = get_parameter("imu_frame").as_string();

    use_imu_initial_guess_ = get_parameter("use_imu_initial_guess").as_bool();

    propagator_options_.use_imu_rotation = get_parameter("use_imu_rotation").as_bool();
    propagator_options_.use_imu_translation = get_parameter("use_imu_translation").as_bool();
    propagator_options_.max_imu_buffer_seconds =
      get_parameter("max_imu_buffer_seconds").as_double();
    propagator_options_.max_allowed_imu_gap =
      get_parameter("max_allowed_imu_gap").as_double();
    propagator_options_.max_expected_yaw_change_deg_per_scan =
      get_parameter("max_expected_yaw_change_deg_per_scan").as_double();

    propagator_options_.gravity =
      readVector3Parameter("gravity", Eigen::Vector3d(0.0, 0.0, -9.81));
    propagator_options_.gyro_bias =
      readVector3Parameter("gyro_bias", Eigen::Vector3d::Zero());
    propagator_options_.accel_bias =
      readVector3Parameter("accel_bias", Eigen::Vector3d::Zero());
    accel_scale_ = get_parameter("accel_scale").as_double();

    publish_imu_predicted_odom_ = get_parameter("publish_imu_predicted_odom").as_bool();
    publish_imu_diagnostics_ = get_parameter("publish_imu_diagnostics").as_bool();
    diagnostics_publish_rate_hz_ = get_parameter("diagnostics_publish_rate_hz").as_double();
    publish_live_propagation_diagnostics_ =
      get_parameter("publish_live_propagation_diagnostics").as_bool();
    propagation_window_seconds_ = get_parameter("propagation_window_seconds").as_double();
    log_throttle_ms_ = get_parameter("log_throttle_ms").as_int();
  }

  Eigen::Vector3d readVector3Parameter(
    const std::string& name,
    const Eigen::Vector3d& fallback) const
  {
    const auto values = get_parameter(name).as_double_array();

    if (values.size() != 3) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter %s must have exactly 3 values; using fallback [%.3f, %.3f, %.3f]",
        name.c_str(),
        fallback.x(),
        fallback.y(),
        fallback.z());
      return fallback;
    }

    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ImuSample sample;
    sample.stamp = msg->header.stamp;
    sample.gyro = Eigen::Vector3d(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    sample.accel = accel_scale_ * Eigen::Vector3d(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);

    propagator_.addSample(sample);
    ++imu_samples_received_;

    latest_frame_id_ = msg->header.frame_id;
    latest_stamp_ = sample.stamp;
    has_latest_sample_ = true;
    latest_gyro_norm_ = sample.gyro.norm();
    latest_accel_norm_ = sample.accel.norm();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "IMU buffered: samples=%zu frame=%s gyro_norm=%.5f rad/s accel_norm=%.5f m/s^2",
      imu_samples_received_,
      latest_frame_id_.c_str(),
      latest_gyro_norm_,
      latest_accel_norm_);

    if (publish_imu_diagnostics_ && shouldPublishDiagnostics()) {
      publishDiagnostics(msg->header.stamp);
    }
  }

  bool shouldPublishDiagnostics()
  {
    if (diagnostics_publish_rate_hz_ <= 0.0) {
      return true;
    }

    const rclcpp::Time now = get_clock()->now();

    if (!has_last_diagnostics_publish_stamp_ || now < last_diagnostics_publish_stamp_) {
      last_diagnostics_publish_stamp_ = now;
      has_last_diagnostics_publish_stamp_ = true;
      return true;
    }

    const double period_sec = 1.0 / diagnostics_publish_rate_hz_;
    if ((now - last_diagnostics_publish_stamp_).seconds() >= period_sec) {
      last_diagnostics_publish_stamp_ = now;
      return true;
    }

    return false;
  }

  void publishDiagnostics(const builtin_interfaces::msg::Time& stamp)
  {
    if (!diagnostics_pub_) {
      return;
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "custom_imu_propagator";
    status.hardware_id = latest_frame_id_.empty() ? imu_frame_ : latest_frame_id_;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "imu_buffering";

    if (latest_accel_norm_ < 7.0 || latest_accel_norm_ > 12.5) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "accel_norm_outside_stationary_expectation";
    }

    status.values.push_back(
      makeKeyValue("samples_received", std::to_string(imu_samples_received_)));
    status.values.push_back(makeKeyValue("frame_id", latest_frame_id_));
    status.values.push_back(makeKeyValue("gyro_norm_rad_s", doubleToString(latest_gyro_norm_)));
    status.values.push_back(makeKeyValue("accel_norm_m_s2", doubleToString(latest_accel_norm_)));
    status.values.push_back(makeKeyValue("accel_scale", doubleToString(accel_scale_)));
    status.values.push_back(
      makeKeyValue("diagnostics_publish_rate_hz", doubleToString(diagnostics_publish_rate_hz_)));
    status.values.push_back(
      makeKeyValue(
        "publish_live_propagation_diagnostics",
        publish_live_propagation_diagnostics_ ? "true" : "false"));
    status.values.push_back(
      makeKeyValue("propagation_window_seconds", doubleToString(propagation_window_seconds_)));
    status.values.push_back(
      makeKeyValue("use_imu_initial_guess", use_imu_initial_guess_ ? "true" : "false"));
    status.values.push_back(
      makeKeyValue("use_imu_rotation", propagator_options_.use_imu_rotation ? "true" : "false"));
    status.values.push_back(
      makeKeyValue(
        "use_imu_translation",
        propagator_options_.use_imu_translation ? "true" : "false"));

    appendLivePropagationDiagnostics(status);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    array.status.push_back(status);

    diagnostics_pub_->publish(array);
  }

  void appendLivePropagationDiagnostics(diagnostic_msgs::msg::DiagnosticStatus& status) const
  {
    if (!publish_live_propagation_diagnostics_) {
      return;
    }

    if (!has_latest_sample_) {
      status.values.push_back(makeKeyValue("propagation_success", "false"));
      status.values.push_back(makeKeyValue("propagation_status", "no_imu_samples_yet"));
      return;
    }

    if (propagation_window_seconds_ <= 0.0) {
      status.values.push_back(makeKeyValue("propagation_success", "false"));
      status.values.push_back(makeKeyValue("propagation_status", "invalid_propagation_window"));
      return;
    }

    const rclcpp::Time start =
      latest_stamp_ - rclcpp::Duration::from_seconds(propagation_window_seconds_);
    const ImuPropagationResult result = propagator_.propagateBetween(start, latest_stamp_);

    status.values.push_back(
      makeKeyValue("propagation_success", result.success ? "true" : "false"));
    status.values.push_back(makeKeyValue("propagation_status", result.status));
    status.values.push_back(makeKeyValue("propagation_samples_used", std::to_string(result.samples_used)));
    status.values.push_back(makeKeyValue("propagation_dt_total", doubleToString(result.dt_total)));
    status.values.push_back(
      makeKeyValue("propagation_delta_roll_deg", doubleToString(result.delta_roll_deg)));
    status.values.push_back(
      makeKeyValue("propagation_delta_pitch_deg", doubleToString(result.delta_pitch_deg)));
    status.values.push_back(
      makeKeyValue("propagation_delta_yaw_deg", doubleToString(result.delta_yaw_deg)));
  }

  std::string imu_topic_;
  std::string predicted_odom_topic_;
  std::string diagnostics_topic_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string latest_frame_id_;

  bool use_imu_initial_guess_ = true;
  bool publish_imu_predicted_odom_ = true;
  bool publish_imu_diagnostics_ = true;
  bool publish_live_propagation_diagnostics_ = true;
  double accel_scale_ = 9.80665;
  double diagnostics_publish_rate_hz_ = 5.0;
  double propagation_window_seconds_ = 0.1;
  int log_throttle_ms_ = 1000;

  std::size_t imu_samples_received_ = 0;
  rclcpp::Time latest_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diagnostics_publish_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_diagnostics_publish_stamp_ = false;
  bool has_latest_sample_ = false;
  double latest_gyro_norm_ = 0.0;
  double latest_accel_norm_ = 0.0;

  ImuPropagatorOptions propagator_options_;
  ImuPropagator propagator_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace custom_imu_propagator

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<custom_imu_propagator::ImuPropagatorNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
