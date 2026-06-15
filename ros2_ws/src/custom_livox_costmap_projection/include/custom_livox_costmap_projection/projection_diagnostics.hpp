#pragma once

#include "custom_livox_costmap_projection/projection_parameters.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace custom_livox_costmap_projection
{

struct FilterStats
{
  std::uint64_t raw_count = 0;
  std::uint64_t kept_count = 0;

  std::uint64_t rejected_nan = 0;
  std::uint64_t rejected_range = 0;
  std::uint64_t rejected_low = 0;
  std::uint64_t rejected_high = 0;
  std::uint64_t rejected_self = 0;

  std::uint64_t grid_occupied_count = 0;

  double processing_time_ms = 0.0;
};

struct TfStatus
{
  bool cloud_to_target_success = true;
  bool target_to_grid_success = true;

  std::string input_frame;
  std::string target_frame;
  std::string grid_frame;
  std::string error_message;
};

class ProjectionDiagnostics
{
public:
  explicit ProjectionDiagnostics(const ProjectionParameters & params);

  diagnostic_msgs::msg::DiagnosticArray buildMessage(
    const rclcpp::Time & stamp,
    const FilterStats & stats,
    const TfStatus & tf_status);

private:
  void updateRollingWindow(const FilterStats & stats);

  double averageProcessingTimeMs() const;
  double averageKeptRatio() const;

  std::size_t window_size_;
  std::deque<double> processing_time_ms_window_;
  std::deque<double> kept_ratio_window_;
};

}  // namespace custom_livox_costmap_projection
