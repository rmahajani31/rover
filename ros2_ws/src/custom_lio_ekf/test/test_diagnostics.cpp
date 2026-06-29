#include "custom_lio_ekf/diagnostics.hpp"

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace custom_lio_ekf
{
namespace
{

bool hasKey(
  const diagnostic_msgs::msg::DiagnosticStatus& status,
  const std::string& key)
{
  for (const auto& value : status.values) {
    if (value.key == key) {
      return true;
    }
  }
  return false;
}

TEST(LioEkfDiagnostics, DefaultDiagnosticsMessageIsPublishable)
{
  LioEkfDiagnostics diagnostics;
  diagnostics.input_points = 42;
  diagnostics.map_points = 0;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;

  const auto message = makeDiagnosticArray(diagnostics, stamp, "custom_lio_ekf");

  ASSERT_EQ(message.status.size(), 1U);
  EXPECT_EQ(message.header.stamp.sec, stamp.sec);
  EXPECT_EQ(message.header.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(message.status.front().name, "custom_lio_ekf");
  EXPECT_EQ(message.status.front().hardware_id, "custom_lio_ekf");
  EXPECT_EQ(
    message.status.front().level,
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(message.status.front().message, "local_map_not_initialized");
  EXPECT_FALSE(message.status.front().values.empty());
  EXPECT_FALSE(hasKey(message.status.front(), "covariance_diagonal"));
}

TEST(LioEkfDiagnostics, SuccessfulUpdateReportsOk)
{
  LioEkfDiagnostics diagnostics;
  diagnostics.map_initialized = true;
  diagnostics.prediction.success = true;
  diagnostics.prediction.status = "success";
  diagnostics.lidar_update.success = true;
  diagnostics.lidar_update.status = "success";

  const auto message =
    makeDiagnosticArray(diagnostics, builtin_interfaces::msg::Time{}, "custom_lio_ekf");

  ASSERT_EQ(message.status.size(), 1U);
  EXPECT_EQ(
    message.status.front().level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(message.status.front().message, "success");
  EXPECT_FALSE(hasKey(message.status.front(), "covariance_diagonal"));
}

}  // namespace
}  // namespace custom_lio_ekf
