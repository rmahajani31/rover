#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

namespace gamepad_adapter
{
    class AdapterNode : public rclcpp::Node
    {
        public:
            AdapterNode();
        
        private:
            void declare_and_load_parameters();
            void log_mapping() const;
            void zero_motion_state();
            void set_estop_state(bool estop_active, const std::string & log_message, bool warn = false);
        
            bool edge_pressed(int idx, const std::vector<int> & buttons) const;
            bool any_edge_pressed(const std::vector<int> & indices, const std::vector<int> & buttons) const;
        
            double apply_deadzone(double x) const;
            double scale_axis(double x, double expo, double max_value) const;
            double step_command(
            double current,
            double target,
            double accel_rate,
            double decel_rate,
            double dt) const;
        
            void publish_twist(double linear_x, double angular_z);
        
            void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
            void on_timer();
            
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
            rclcpp::TimerBase::SharedPtr timer_;

            int throttle_axis_{1};
            int steer_axis_{3};
          
            double deadzone_{0.06};
            double max_linear_x_{0.35};
            double max_angular_z_{0.8};
            double throttle_expo_{0.5};
            double steer_expo_{0.6};
            double publish_rate_hz_{30.0};
            double accel_rate_linear_{0.6};
            double decel_rate_linear_{1.5};
            double accel_rate_angular_{1.2};
            double decel_rate_angular_{2.5};
          
            bool invert_throttle_{false};
            bool invert_steer_{false};
            bool swapped_{false};
            bool start_estop_active_{false};
          
            std::vector<int> btn_estop_{0};
            int btn_swap_{-1};
            int btn_inv_throttle_{-1};
            int btn_inv_steer_{-1};
          
            bool estop_state_{false};
            double desired_linear_x_{0.0};
            double desired_angular_z_{0.0};
            double current_linear_x_{0.0};
            double current_angular_z_{0.0};

            std::vector<int> prev_buttons_;
            bool have_prev_buttons_{false};
          
            std::chrono::steady_clock::time_point last_publish_time_;
    };
}