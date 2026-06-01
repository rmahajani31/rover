#include "gamepad_adapter/adapter_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

using std::placeholders::_1;

namespace
{
    double clip11(double x)
    {
    return std::clamp(x, -1.0, 1.0);
    }

    double clamp01(double x)
    {
        return std::clamp(x, 0.0, 1.0);
    }

    double apply_expo(double x, double expo)
    {
        expo = clamp01(expo);
        // Blend linear and cubic response: low stick values become gentler
        // while full-scale commands still reach the configured limit.
        return ((1.0 - expo) * x) + (expo * x * x * x);
    }

    double step_toward(double current, double target, double max_step)
    {
        const double delta = target - current;
        if (delta > max_step) {
            return current + max_step;
        }
        if (delta < -max_step) {
            return current - max_step;
        }
        return target;
    }
}

namespace gamepad_adapter
{
    AdapterNode::AdapterNode()
    : rclcpp::Node("adapter_node"),
    last_publish_time_(std::chrono::steady_clock::now())
    {
        declare_and_load_parameters();

        pub_cmd_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&AdapterNode::on_joy, this, _1));

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&AdapterNode::on_timer, this));

        log_mapping();

        if (start_estop_active_) {
            set_estop_state(
            true,
            "Startup E-STOP active. Move the joystick to let it initialize, then press the E-STOP button again to enable motion.",
            true);
        }
    }

    void AdapterNode::declare_and_load_parameters()
    {
        declare_parameter("throttle_axis", 1);
        declare_parameter("steer_axis", 3);
        declare_parameter("deadzone", 0.06);
        declare_parameter("max_linear_x", 0.35);
        declare_parameter("max_angular_z", 0.8);
        declare_parameter("throttle_expo", 0.5);
        declare_parameter("steer_expo", 0.6);
        declare_parameter("publish_rate_hz", 30.0);
        declare_parameter("accel_rate_linear", 0.6);
        declare_parameter("decel_rate_linear", 1.5);
        declare_parameter("accel_rate_angular", 1.2);
        declare_parameter("decel_rate_angular", 2.5);

        declare_parameter("invert_throttle", false);
        declare_parameter("invert_steer", false);
        declare_parameter("start_swapped", false);
        declare_parameter("start_estop_active", false);

        declare_parameter("btn_estop", std::vector<int64_t>{0});
        declare_parameter("btn_swap", -1);
        declare_parameter("btn_inv_throttle", -1);
        declare_parameter("btn_inv_steer", -1);

        throttle_axis_ = get_parameter("throttle_axis").as_int();
        steer_axis_ = get_parameter("steer_axis").as_int();
        deadzone_ = get_parameter("deadzone").as_double();
        max_linear_x_ = get_parameter("max_linear_x").as_double();
        max_angular_z_ = get_parameter("max_angular_z").as_double();
        throttle_expo_ = get_parameter("throttle_expo").as_double();
        steer_expo_ = get_parameter("steer_expo").as_double();
        publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
        accel_rate_linear_ = get_parameter("accel_rate_linear").as_double();
        decel_rate_linear_ = get_parameter("decel_rate_linear").as_double();
        accel_rate_angular_ = get_parameter("accel_rate_angular").as_double();
        decel_rate_angular_ = get_parameter("decel_rate_angular").as_double();

        invert_throttle_ = get_parameter("invert_throttle").as_bool();
        invert_steer_ = get_parameter("invert_steer").as_bool();
        swapped_ = get_parameter("start_swapped").as_bool();
        start_estop_active_ = get_parameter("start_estop_active").as_bool();

        const auto btn_estop_param = get_parameter("btn_estop").as_integer_array();
        btn_estop_.clear();
        btn_estop_.reserve(btn_estop_param.size());
        for (const auto value : btn_estop_param) {
            btn_estop_.push_back(static_cast<int>(value));
        }

        btn_swap_ = get_parameter("btn_swap").as_int();
        btn_inv_throttle_ = get_parameter("btn_inv_throttle").as_int();
        btn_inv_steer_ = get_parameter("btn_inv_steer").as_int();
    }

    void AdapterNode::log_mapping() const
    {
        RCLCPP_INFO(
            get_logger(),
            "axes: throttle=%d steer=%d | invert(thr)=%s invert(steer)=%s | swapped=%s deadzone=%.3f",
            throttle_axis_, steer_axis_,
            invert_throttle_ ? "true" : "false",
            invert_steer_ ? "true" : "false",
            swapped_ ? "true" : "false",
            deadzone_);

        std::string estop_buttons = "[";
        for (std::size_t i = 0; i < btn_estop_.size(); ++i) {
            estop_buttons += std::to_string(btn_estop_[i]);
            if (i + 1 < btn_estop_.size()) {
            estop_buttons += ", ";
            }
        }
        estop_buttons += "]";

        RCLCPP_INFO(
            get_logger(),
            "buttons: estop=%s swap=%d inv_thr=%d inv_steer=%d",
            estop_buttons.c_str(), btn_swap_, btn_inv_throttle_, btn_inv_steer_);

        RCLCPP_INFO(
            get_logger(),
            "limits: max_linear_x=%.3f max_angular_z=%.3f expo(thr)=%.2f expo(steer)=%.2f",
            max_linear_x_, max_angular_z_, throttle_expo_, steer_expo_);

        RCLCPP_INFO(
            get_logger(),
            "smoothing: publish_rate_hz=%.1f accel_linear=%.2f decel_linear=%.2f accel_angular=%.2f decel_angular=%.2f",
            publish_rate_hz_, accel_rate_linear_, decel_rate_linear_,
            accel_rate_angular_, decel_rate_angular_);

        RCLCPP_INFO(
            get_logger(), "start_estop_active=%s", start_estop_active_ ? "true" : "false");
    }

    void AdapterNode::zero_motion_state()
    {
        desired_linear_x_ = 0.0;
        desired_angular_z_ = 0.0;
        current_linear_x_ = 0.0;
        current_angular_z_ = 0.0;
    }

    void AdapterNode::set_estop_state(
    bool estop_active,
    const std::string & log_message,
    bool warn)
    {
        estop_state_ = estop_active;
        // Always publish a zero command on E-stop transitions so stale velocity
        // commands do not keep the rover moving.
        zero_motion_state();
        publish_twist(0.0, 0.0);

        if (warn) {
            RCLCPP_WARN(get_logger(), "%s", log_message.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "%s", log_message.c_str());
        }
    }

    bool AdapterNode::edge_pressed(int idx, const std::vector<int> & buttons) const
    {
        if (idx < 0 || !have_prev_buttons_) {
            return false;
        }

        const int prev = (static_cast<std::size_t>(idx) < prev_buttons_.size()) ? prev_buttons_[idx] : 0;
        const int now = (static_cast<std::size_t>(idx) < buttons.size()) ? buttons[idx] : 0;
        return prev == 0 && now == 1;
    }

    bool AdapterNode::any_edge_pressed(
    const std::vector<int> & indices,
    const std::vector<int> & buttons) const
    {
        for (const int idx : indices) {
            if (idx >= 0 && edge_pressed(idx, buttons)) {
            return true;
            }
        }
        return false;
    }

    double AdapterNode::apply_deadzone(double x) const
    {
        return (std::abs(x) < deadzone_) ? 0.0 : x;
    }

    double AdapterNode::scale_axis(double x, double expo, double max_value) const
    {
        const double shaped = apply_expo(apply_deadzone(clip11(x)), expo);
        return shaped * max_value;
    }

    double AdapterNode::step_command(
        double current,
        double target,
        double accel_rate,
        double decel_rate,
        double dt) const
      {
        const bool same_direction =
            (current == 0.0) || (target == 0.0) || ((current > 0.0) == (target > 0.0));
        const bool reducing_magnitude = std::abs(target) < std::abs(current);
        const bool use_decel = reducing_magnitude || !same_direction;
        const double max_step = std::max(0.0, (use_decel ? decel_rate : accel_rate) * dt);
        return step_toward(current, target, max_step);
      }
      
      void AdapterNode::publish_twist(double linear_x, double angular_z)
      {
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = get_clock()->now();
        twist.twist.linear.x = linear_x;
        twist.twist.angular.z = angular_z;
        pub_cmd_->publish(twist);
      }

      void AdapterNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const auto & axes = msg->axes;
        std::vector<int> buttons(msg->buttons.begin(), msg->buttons.end());

        if (any_edge_pressed(btn_estop_, buttons)) {
            const bool next_state = !estop_state_;
            if (next_state) {
            set_estop_state(
                true,
                "E-STOP enabled. Motion inhibited until the E-STOP button is pressed again.",
                true);
            } else {
            set_estop_state(
                false,
                "E-STOP cleared. Teleop motion re-enabled.");
            }
            prev_buttons_ = buttons;
            have_prev_buttons_ = true;
            return;
        }

        if (edge_pressed(btn_swap_, buttons)) {
            swapped_ = !swapped_;
            RCLCPP_INFO(get_logger(), "SWAP toggled -> swapped=%s", swapped_ ? "true" : "false");
        }

        if (edge_pressed(btn_inv_throttle_, buttons)) {
            invert_throttle_ = !invert_throttle_;
            RCLCPP_INFO(
            get_logger(), "Invert throttle -> %s", invert_throttle_ ? "true" : "false");
        }

        if (edge_pressed(btn_inv_steer_, buttons)) {
            invert_steer_ = !invert_steer_;
            RCLCPP_INFO(
            get_logger(), "Invert steer -> %s", invert_steer_ ? "true" : "false");
        }

        const auto safe_get = [&axes](int idx) -> double {
            return (idx >= 0 && static_cast<std::size_t>(idx) < axes.size()) ? axes[idx] : 0.0;
            };

        int ax_t = throttle_axis_;
        int ax_s = steer_axis_;
        if (swapped_) {
            std::swap(ax_t, ax_s);
        }

        double throttle = safe_get(ax_t);
        double steer = safe_get(ax_s);

        if (invert_throttle_) {
            throttle = -throttle;
        }
        if (invert_steer_) {
            steer = -steer;
        }

        if (estop_state_) {
            desired_linear_x_ = 0.0;
            desired_angular_z_ = 0.0;
        } else {
            // on_timer applies acceleration limits; this callback only updates
            // the desired target from the latest joystick state.
            desired_linear_x_ = scale_axis(throttle, throttle_expo_, max_linear_x_);
            desired_angular_z_ = scale_axis(steer, steer_expo_, max_angular_z_);
        }

        prev_buttons_ = buttons;
        have_prev_buttons_ = true;
    }

    void AdapterNode::on_timer()
    {
        const auto now = std::chrono::steady_clock::now();
        const std::chrono::duration<double> elapsed = now - last_publish_time_;
        const double dt = std::max(1e-3, elapsed.count());
        last_publish_time_ = now;

        current_linear_x_ = step_command(
            current_linear_x_,
            desired_linear_x_,
            accel_rate_linear_,
            decel_rate_linear_,
            dt);

        current_angular_z_ = step_command(
            current_angular_z_,
            desired_angular_z_,
            accel_rate_angular_,
            decel_rate_angular_,
            dt);

        publish_twist(current_linear_x_, current_angular_z_); 
    }
}
