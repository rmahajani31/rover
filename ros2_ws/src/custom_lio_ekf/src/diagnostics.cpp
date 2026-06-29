#include "custom_lio_ekf/diagnostics.hpp"

#include <sstream>
#include <string>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace custom_lio_ekf
{

namespace
{

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string& key,
  const std::string& value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  return key_value;
}

std::string vectorToString(const Eigen::Vector3d& value)
{
  std::ostringstream stream;
  stream << value.x() << ", " << value.y() << ", " << value.z();
  return stream.str();
}

std::string covarianceDiagonalToString(const Vector18d& diagonal)
{
  std::ostringstream stream;
  for (int i = 0; i < kErrorStateDim; ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << diagonal(i);
  }
  return stream.str();
}

void addValue(
  diagnostic_msgs::msg::DiagnosticStatus& status,
  const std::string& key,
  const std::string& value)
{
  status.values.push_back(makeKeyValue(key, value));
}

void addValue(
  diagnostic_msgs::msg::DiagnosticStatus& status,
  const std::string& key,
  double value)
{
  addValue(status, key, std::to_string(value));
}

void addValue(
  diagnostic_msgs::msg::DiagnosticStatus& status,
  const std::string& key,
  std::size_t value)
{
  addValue(status, key, std::to_string(value));
}

void addValue(
  diagnostic_msgs::msg::DiagnosticStatus& status,
  const std::string& key,
  int value)
{
  addValue(status, key, std::to_string(value));
}

void addValue(
  diagnostic_msgs::msg::DiagnosticStatus& status,
  const std::string& key,
  bool value)
{
  addValue(status, key, value ? "true" : "false");
}

}  // namespace

LioEkfDiagnostics makeDiagnostics(
  const EkfState& state,
  const EkfPredictionStats& prediction,
  const LidarUpdateStats& lidar_update)
{
  LioEkfDiagnostics diagnostics;
  diagnostics.prediction = prediction;
  diagnostics.lidar_update = lidar_update;
  diagnostics.gyro_bias = state.b_g;
  diagnostics.accel_bias = state.b_a;
  diagnostics.gravity = state.g_W;
  diagnostics.covariance_diagonal = state.P.diagonal();
  return diagnostics;
}

diagnostic_msgs::msg::DiagnosticArray makeDiagnosticArray(
  const LioEkfDiagnostics& diagnostics,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& name)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "custom_lio_ekf";

  if (!diagnostics.map_initialized) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "local_map_not_initialized";
  } else if (!diagnostics.prediction.success) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = diagnostics.prediction.status;
  } else if (!diagnostics.lidar_update.success) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = diagnostics.lidar_update.status;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "success";
  }

  addValue(status, "map_initialized", diagnostics.map_initialized);
  addValue(status, "input_points", diagnostics.input_points);
  addValue(status, "map_points", diagnostics.map_points);

  addValue(status, "prediction_success", diagnostics.prediction.success);
  addValue(status, "prediction_status", diagnostics.prediction.status);
  addValue(status, "imu_intervals_integrated", diagnostics.prediction.intervals_integrated);
  addValue(status, "imu_dt_total", diagnostics.prediction.dt_total);

  addValue(status, "lidar_update_success", diagnostics.lidar_update.success);
  addValue(status, "lidar_update_status", diagnostics.lidar_update.status);
  addValue(status, "lidar_iterations", diagnostics.lidar_update.iterations);
  addValue(status, "valid_residuals", diagnostics.lidar_update.valid_residuals);
  addValue(status, "mean_abs_residual", diagnostics.lidar_update.mean_abs_residual);
  addValue(status, "rms_residual", diagnostics.lidar_update.rms_residual);
  addValue(status, "max_abs_residual", diagnostics.lidar_update.max_abs_residual);
  addValue(status, "delta_theta_norm", diagnostics.lidar_update.final_delta_theta_norm);
  addValue(status, "delta_position_norm", diagnostics.lidar_update.final_delta_position_norm);

  addValue(status, "prediction_time_ms", diagnostics.prediction_time_ms);
  addValue(status, "lidar_update_time_ms", diagnostics.lidar_update_time_ms);
  addValue(status, "map_update_time_ms", diagnostics.map_update_time_ms);

  // Keep startup diagnostics scalar-only, matching the scan-to-map diagnostics
  // pattern. The first cloud can arrive before IMU calibration initializes the
  // EKF state; publishing the status is more important than exporting state
  // internals in that transient path.
  addValue(status, "state_snapshot_available", diagnostics.map_initialized);
  if (diagnostics.map_initialized) {
    addValue(status, "gyro_bias", vectorToString(diagnostics.gyro_bias));
    addValue(status, "accel_bias", vectorToString(diagnostics.accel_bias));
    addValue(status, "gravity", vectorToString(diagnostics.gravity));
    addValue(
      status,
      "covariance_diagonal",
      covarianceDiagonalToString(diagnostics.covariance_diagonal));
  }

  array.status.push_back(status);
  return array;
}

}  // namespace custom_lio_ekf
