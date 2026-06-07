#include <memory>
#include <string>
#include <functional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

class Fastlio2Nav2Adapter : public rclcpp::Node
{
public:
  Fastlio2Nav2Adapter()
  : Node("fastlio2_nav2_adapter")
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/fastlio_odom");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/nav2_odom");

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");

    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    force_planar_output_ = declare_parameter<bool>("force_planar_output", true);
    apply_lidar_to_base_correction_ =
      declare_parameter<bool>("apply_lidar_to_base_correction", false);
    reject_old_timestamps_ = declare_parameter<bool>("reject_old_timestamps", true);
    max_stamp_age_sec_ = declare_parameter<double>("max_stamp_age_sec", 0.5);
    print_debug_ = declare_parameter<bool>("print_debug", true);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, rclcpp::QoS(20));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_,
      rclcpp::QoS(50),
      std::bind(&Fastlio2Nav2Adapter::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FAST-LIO2 Nav2 adapter started");
    RCLCPP_INFO(get_logger(), "Input odom topic: %s", input_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output odom topic: %s", output_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "lidar_frame: %s", lidar_frame_.c_str());
    RCLCPP_INFO(get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "force_planar_output: %s",
      force_planar_output_ ? "true" : "false");

    if (apply_lidar_to_base_correction_) {
      RCLCPP_WARN(
        get_logger(),
        "apply_lidar_to_base_correction is configured but not implemented in adapter v1");
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (reject_old_timestamps_) {
      const rclcpp::Time stamp(msg->header.stamp);
      const rclcpp::Time now = this->now();
      const double age_sec = (now - stamp).seconds();

      if (age_sec > max_stamp_age_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Rejecting old FAST-LIO2 odom message. Age: %.3f sec",
          age_sec);
        return;
      }
    }

    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;

    if (force_planar_output_) {
      out.pose.pose.position.z = 0.0;
      out.pose.pose.orientation = planarizeOrientation(out.pose.pose.orientation);
      out.twist.twist.linear.z = 0.0;
      out.twist.twist.angular.x = 0.0;
      out.twist.twist.angular.y = 0.0;
    }

    odom_pub_->publish(out);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = out.header.stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = out.pose.pose.position.x;
      tf_msg.transform.translation.y = out.pose.pose.position.y;
      tf_msg.transform.translation.z = out.pose.pose.position.z;
      tf_msg.transform.rotation = out.pose.pose.orientation;

      tf_broadcaster_->sendTransform(tf_msg);
    }

    if (print_debug_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Published %s: x=%.3f y=%.3f z=%.3f",
        output_odom_topic_.c_str(),
        out.pose.pose.position.x,
        out.pose.pose.position.y,
        out.pose.pose.position.z);
    }
  }

  geometry_msgs::msg::Quaternion planarizeOrientation(
    const geometry_msgs::msg::Quaternion & orientation) const
  {
    tf2::Quaternion input(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(input).getRPY(roll, pitch, yaw);

    tf2::Quaternion planar;
    planar.setRPY(0.0, 0.0, yaw);
    planar.normalize();

    geometry_msgs::msg::Quaternion output;
    output.x = planar.x();
    output.y = planar.y();
    output.z = planar.z();
    output.w = planar.w();
    return output;
  }

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;

  bool publish_tf_;
  bool force_planar_output_;
  bool apply_lidar_to_base_correction_;
  bool reject_old_timestamps_;
  bool print_debug_;
  double max_stamp_age_sec_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fastlio2Nav2Adapter>());
  rclcpp::shutdown();
  return 0;
}
