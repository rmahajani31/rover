#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <numbers>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "livox_cloud_to_scan/cloud_projector.hpp"
#include "livox_cloud_to_scan/scan_projection_params.hpp"

class LivoxCloudToScanNode : public rclcpp::Node
{
    public:
        LivoxCloudToScanNode()
        : Node("livox_cloud_to_scan"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
        {
            declareParameters();
            loadParameters();
            validateParameters();

            projector_ = std::make_unique<livox_cloud_to_scan::CloudProjector>(params_);

            scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
                params_.output_topic, rclcpp::SensorDataQoS()
            );

            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                params_.input_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&LivoxCloudToScanNode::cloudCallback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "livox_cloud_to_scan started");
            RCLCPP_INFO(this->get_logger(), "Input topic: %s", params_.input_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Output topic: %s", params_.output_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Target frame: %s", params_.target_frame.c_str());
        }
    
    private:
        
        void declareParameters() {
            this->declare_parameter<std::string>("input_topic", "/livox/lidar");
            this->declare_parameter<std::string>("output_topic", "/scan_from_livox");
            this->declare_parameter<std::string>("target_frame", "base_link");

            this->declare_parameter<double>("min_height", 0.08);
            this->declare_parameter<double>("max_height", 0.70);
            this->declare_parameter<double>("min_range", 0.30);
            this->declare_parameter<double>("max_range", 8.0);

            this->declare_parameter<double>("angle_min", -std::numbers::pi);
            this->declare_parameter<double>("angle_max", std::numbers::pi);
            this->declare_parameter<double>("angle_increment", 0.005);

            this->declare_parameter<double>("scan_time", 0.1);
            this->declare_parameter<double>("transform_timeout_sec", 0.05);

            this->declare_parameter<bool>("publish_debug_logs", true);
        }

        void loadParameters() {
            params_.input_topic = this->get_parameter("input_topic").as_string();
            params_.output_topic = this->get_parameter("output_topic").as_string();
            params_.target_frame = this->get_parameter("target_frame").as_string();
        
            params_.min_height = this->get_parameter("min_height").as_double();
            params_.max_height = this->get_parameter("max_height").as_double();
            params_.min_range = this->get_parameter("min_range").as_double();
            params_.max_range = this->get_parameter("max_range").as_double();
        
            params_.angle_min = this->get_parameter("angle_min").as_double();
            params_.angle_max = this->get_parameter("angle_max").as_double();
            params_.angle_increment = this->get_parameter("angle_increment").as_double();
        
            params_.scan_time = this->get_parameter("scan_time").as_double();
            params_.transform_timeout_sec =
            this->get_parameter("transform_timeout_sec").as_double();
        
            params_.publish_debug_logs =
            this->get_parameter("publish_debug_logs").as_bool();
        }

        void validateParameters() {
            std::string error;
            if (!params_.validate(&error)) {
                RCLCPP_FATAL(this->get_logger(), "Invalid scan projection parameters: %s", error.c_str());
                throw std::invalid_argument(error);
            }
        }

        void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped transform_msg;

            try {
                transform_msg = tf_buffer_.lookupTransform(
                    params_.target_frame,
                    msg->header.frame_id,
                    msg->header.stamp,
                    rclcpp::Duration::from_seconds(params_.transform_timeout_sec)
                );
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "TF lookup failed from %s to %s: %s",
                    msg->header.frame_id.c_str(),
                    params_.target_frame.c_str(),
                    ex.what());
                return;
            }

            std::string projection_error;
            auto scan = projector_->project(*msg, transform_msg, &projection_error);
            if (!projection_error.empty()) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Skipping malformed cloud on %s: %s",
                    params_.input_topic.c_str(),
                    projection_error.c_str());
                return;
            }

            scan_pub_->publish(scan);
            
            if (params_.publish_debug_logs) {
                RCLCPP_INFO_THROTTLE(
                  this->get_logger(),
                  *this->get_clock(),
                  3000,
                  "Projected cloud frame=%s points=%u x %u into scan bins=%zu",
                  msg->header.frame_id.c_str(),
                  msg->width,
                  msg->height,
                  scan.ranges.size());
              }
        }

        livox_cloud_to_scan::ScanProjectionParams params_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
      
        std::unique_ptr<livox_cloud_to_scan::CloudProjector> projector_;
      
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<LivoxCloudToScanNode>());
    } catch (const std::exception & ex) {
        RCLCPP_FATAL(rclcpp::get_logger("livox_cloud_to_scan"), "Node startup failed: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
