#include "custom_livox_costmap_projection/projection_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <algorithm>
#include <numeric>
#include <string>

namespace custom_livox_costmap_projection
{

namespace
{

diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string & key, const std::uint64_t value)
{
  return makeKeyValue(key, std::to_string(value));
}

diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string & key, const double value)
{
  return makeKeyValue(key, std::to_string(value));
}

std::string boolString(const bool value)
{
  return value ? "true" : "false";
}

double keptRatio(const FilterStats & stats)
{
  if (stats.raw_count == 0U) {
    return 0.0;
  }

  return static_cast<double>(stats.kept_count) / static_cast<double>(stats.raw_count);
}

}  // namespace

ProjectionDiagnostics::ProjectionDiagnostics(const ProjectionParameters & params)
: window_size_(static_cast<std::size_t>(std::max(params.diagnostics_frame_count_window, 1)))
{
}

diagnostic_msgs::msg::DiagnosticArray ProjectionDiagnostics::buildMessage(
  const rclcpp::Time & stamp,
  const FilterStats & stats,
  const TfStatus & tf_status)
{
  updateRollingWindow(stats);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "custom_livox_costmap_projection";
  status.hardware_id = "livox_mid360";

  const bool tf_ok = tf_status.cloud_to_target_success && tf_status.target_to_grid_success;

  if (!tf_ok) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "TF lookup failed";
  } else if (stats.raw_count == 0U) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Received empty input cloud";
  } else if (stats.kept_count == 0U) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "No obstacle points kept";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Projection healthy";
  }

  status.values.push_back(makeKeyValue(
    "tf_cloud_to_target_success", boolString(tf_status.cloud_to_target_success)));
  status.values.push_back(makeKeyValue(
    "tf_target_to_grid_success", boolString(tf_status.target_to_grid_success)));
  status.values.push_back(makeKeyValue("input_frame", tf_status.input_frame));
  status.values.push_back(makeKeyValue("target_frame", tf_status.target_frame));
  status.values.push_back(makeKeyValue("grid_frame", tf_status.grid_frame));
  status.values.push_back(makeKeyValue("tf_error_message", tf_status.error_message));

  status.values.push_back(makeKeyValue("raw_count", stats.raw_count));
  status.values.push_back(makeKeyValue("kept_count", stats.kept_count));
  status.values.push_back(makeKeyValue("rejected_nan", stats.rejected_nan));
  status.values.push_back(makeKeyValue("rejected_range", stats.rejected_range));
  status.values.push_back(makeKeyValue("rejected_low", stats.rejected_low));
  status.values.push_back(makeKeyValue("rejected_high", stats.rejected_high));
  status.values.push_back(makeKeyValue("rejected_self", stats.rejected_self));
  status.values.push_back(makeKeyValue("grid_occupied_count", stats.grid_occupied_count));

  status.values.push_back(makeKeyValue("kept_ratio", keptRatio(stats)));
  status.values.push_back(makeKeyValue("processing_time_ms", stats.processing_time_ms));
  status.values.push_back(makeKeyValue("average_kept_ratio", averageKeptRatio()));
  status.values.push_back(makeKeyValue("average_processing_time_ms", averageProcessingTimeMs()));

  array.status.push_back(status);
  return array;
}

void ProjectionDiagnostics::updateRollingWindow(const FilterStats & stats)
{
  processing_time_ms_window_.push_back(stats.processing_time_ms);
  kept_ratio_window_.push_back(keptRatio(stats));

  while (processing_time_ms_window_.size() > window_size_) {
    processing_time_ms_window_.pop_front();
  }

  while (kept_ratio_window_.size() > window_size_) {
    kept_ratio_window_.pop_front();
  }
}

double ProjectionDiagnostics::averageProcessingTimeMs() const
{
  if (processing_time_ms_window_.empty()) {
    return 0.0;
  }

  const double sum = std::accumulate(
    processing_time_ms_window_.begin(),
    processing_time_ms_window_.end(),
    0.0);

  return sum / static_cast<double>(processing_time_ms_window_.size());
}

double ProjectionDiagnostics::averageKeptRatio() const
{
  if (kept_ratio_window_.empty()) {
    return 0.0;
  }

  const double sum = std::accumulate(
    kept_ratio_window_.begin(),
    kept_ratio_window_.end(),
    0.0);

  return sum / static_cast<double>(kept_ratio_window_.size());
}

}  // namespace custom_livox_costmap_projection
