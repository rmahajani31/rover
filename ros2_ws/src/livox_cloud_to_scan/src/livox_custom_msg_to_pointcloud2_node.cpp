#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class LivoxCustomMsgToPointCloud2Node : public rclcpp::Node
{
public:
  LivoxCustomMsgToPointCloud2Node()
  : Node("livox_custom_msg_to_pointcloud2")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/points");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "livox_frame");

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());

    custom_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        this->customMsgCallback(msg);
      });

    RCLCPP_INFO(this->get_logger(), "livox_custom_msg_to_pointcloud2 started");
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame fallback: %s", frame_id_.c_str());
  }

private:
  static void writeFloat(std::uint8_t * data, const std::size_t offset, const float value)
  {
    std::memcpy(data + offset, &value, sizeof(float));
  }

  void customMsgCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    if (cloud.header.frame_id.empty()) {
      cloud.header.frame_id = frame_id_;
    }

    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(msg->points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    cloud.fields.resize(4);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;
    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    for (std::size_t i = 0; i < msg->points.size(); ++i) {
      const auto & point = msg->points[i];
      auto * data = cloud.data.data() + i * cloud.point_step;
      writeFloat(data, 0, point.x);
      writeFloat(data, 4, point.y);
      writeFloat(data, 8, point.z);
      writeFloat(data, 12, static_cast<float>(point.reflectivity));
    }

    pointcloud_pub_->publish(cloud);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomMsgToPointCloud2Node>());
  rclcpp::shutdown();
  return 0;
}
