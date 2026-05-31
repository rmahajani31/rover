#pragma once

#include <cmath>
#include <numbers>
#include <string>

namespace livox_cloud_to_scan
{
    struct ScanProjectionParams 
    {
        std::string input_topic = "/livox/lidar";
        std::string output_topic = "/scan_from_livox";
        std::string target_frame = "base_link";

        double min_height = 0.08;
        double max_height = 0.70;

        double min_range = 0.30;
        double max_range = 8.0;

        double angle_min = -std::numbers::pi;
        double angle_max = std::numbers::pi;
        double angle_increment = 0.005;

        double scan_time = 0.1;
        double transform_timeout_sec = 0.05;

        bool missing_bins_as_inf = false;
        bool publish_debug_logs = true;

        [[nodiscard]] bool validate(std::string * error = nullptr) const
        {
            if (!std::isfinite(min_height) || !std::isfinite(max_height) ||
                !std::isfinite(min_range) || !std::isfinite(max_range) ||
                !std::isfinite(angle_min) || !std::isfinite(angle_max) ||
                !std::isfinite(angle_increment) || !std::isfinite(scan_time) ||
                !std::isfinite(transform_timeout_sec))
            {
                if (error != nullptr) {
                    *error = "projection parameters must be finite";
                }
                return false;
            }

            if (min_height > max_height) {
                if (error != nullptr) {
                    *error = "min_height must be less than or equal to max_height";
                }
                return false;
            }

            if (min_range < 0.0 || min_range > max_range) {
                if (error != nullptr) {
                    *error = "range bounds must satisfy 0 <= min_range <= max_range";
                }
                return false;
            }

            if (angle_increment <= 0.0) {
                if (error != nullptr) {
                    *error = "angle_increment must be greater than zero";
                }
                return false;
            }

            if (angle_max <= angle_min) {
                if (error != nullptr) {
                    *error = "angle_max must be greater than angle_min";
                }
                return false;
            }

            if (scan_time < 0.0) {
                if (error != nullptr) {
                    *error = "scan_time must be non-negative";
                }
                return false;
            }

            if (transform_timeout_sec < 0.0) {
                if (error != nullptr) {
                    *error = "transform_timeout_sec must be non-negative";
                }
                return false;
            }

            return true;
        }
    };
}
