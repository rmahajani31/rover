#include <chrono>
#include <functional>
#include <stdexcept>
#include <thread>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rover_odometry/odometry_node.hpp"
#include "rover_odometry/odometry_utils.hpp"

namespace rover_odometry
{
    OdometryNode::OdometryNode()
    : Node("odometry")
    {
        declareParameters();
        loadParameters();

        i2c_ = std::make_unique<PinpointI2C>(bus_, addr_, parseEndian(endian_string_));

        initializeDevice();
        initializeState();

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&OdometryNode::update, this));

        RCLCPP_INFO(
        this->get_logger(),
        "odometry started. x=%.3fm, y=%.3fm, yaw=%.3frad",
        x_mm_ / 1000.0,
        y_mm_ / 1000.0,
        yaw_rad_);
    }

    void OdometryNode::declareParameters()
    {
        this->declare_parameter<int>("bus", 1);
        this->declare_parameter<int>("addr", 0x31);
        this->declare_parameter<std::string>("endian", "little");
        this->declare_parameter<double>("publish_rate_hz", 50.0);
        this->declare_parameter<std::vector<double>>("pod_offsets_mm", {160.87, -201.3});
        this->declare_parameter<std::vector<bool>>("encoder_directions", {true, true});
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
    }

    void OdometryNode::loadParameters()
    {
        bus_ = this->get_parameter("bus").as_int();
        addr_ = this->get_parameter("addr").as_int();
        endian_string_ = this->get_parameter("endian").as_string();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
        pod_offsets_mm_ = this->get_parameter("pod_offsets_mm").as_double_array();
        encoder_directions_ = this->get_parameter("encoder_directions").as_bool_array();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();

        if (pod_offsets_mm_.size() != 2) {
        throw std::invalid_argument("pod_offsets_mm must contain exactly 2 values");
        }

        if (encoder_directions_.size() != 2) {
        throw std::invalid_argument("encoder_directions must contain exactly 2 values");
        }

        if (publish_rate_hz_ <= 0.0) {
        throw std::invalid_argument("publish_rate_hz must be greater than zero");
        }
    }

    void OdometryNode::initializeDevice()
    {
        i2c_->resetPosAndImu();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        i2c_->setPodOffsetsMm(
        static_cast<float>(pod_offsets_mm_[0]),
        static_cast<float>(pod_offsets_mm_[1]));
        std::this_thread::sleep_for(std::chrono::seconds(1));

        i2c_->setEncoderDirections(encoder_directions_[0], encoder_directions_[1]);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void OdometryNode::initializeState()
    {
        x_mm_ = i2c_->readF32(8);
        y_mm_ = i2c_->readF32(9);
        yaw_rad_ = i2c_->readF32(10);
        vx_mm_s_ = i2c_->readF32(11);
        vy_mm_s_ = i2c_->readF32(12);
        yaw_rate_rad_s_ = i2c_->readF32(13);
    }

    void OdometryNode::update()
    {
        x_mm_ = i2c_->readF32(8);
        y_mm_ = i2c_->readF32(9);
        yaw_rad_ = i2c_->readF32(10);
        vx_mm_s_ = i2c_->readF32(11);
        vy_mm_s_ = i2c_->readF32(12);
        yaw_rate_rad_s_ = i2c_->readF32(13);

        const auto now = this->get_clock()->now();
        const auto quaternion = yawToQuaternion(wrapPi(yaw_rad_));

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = x_mm_ / 1000.0;
        tf_msg.transform.translation.y = y_mm_ / 1000.0;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = quaternion;
        tf_broadcaster_->sendTransform(tf_msg);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        odom_msg.pose.pose.position.x = x_mm_ / 1000.0;
        odom_msg.pose.pose.position.y = y_mm_ / 1000.0;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = quaternion;
        odom_msg.twist.twist.linear.x = vx_mm_s_ / 1000.0;
        odom_msg.twist.twist.linear.y = vy_mm_s_ / 1000.0;
        odom_msg.twist.twist.angular.z = yaw_rate_rad_s_;

        const double xy_var = 0.01 + 0.05 * std::abs(yaw_rate_rad_s_);
        const double yaw_var = 0.02 + 0.10 * std::abs(yaw_rate_rad_s_);

        odom_msg.pose.covariance.fill(0.0);
        odom_msg.pose.covariance[0] = xy_var;
        odom_msg.pose.covariance[7] = xy_var;
        odom_msg.pose.covariance[35] = yaw_var;

        odom_pub_->publish(odom_msg);
    }
}
