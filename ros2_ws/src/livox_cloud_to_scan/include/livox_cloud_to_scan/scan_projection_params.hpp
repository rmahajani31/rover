#pragma once

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
        double transform_timeout_sec = 0.005;

        bool publish_debug_logs = true;
    }
}