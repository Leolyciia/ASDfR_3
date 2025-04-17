#ifndef SEQUENCE_CONTROLLER_HPP_
#define SEQUENCE_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <vector> 
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using namespace std::chrono_literals;

// Control parameters)
struct ControlParameters {
    double turn_gain = 1.25;
    double forward_gain = 2.0;
    double target_x_pixel = 160.0;
    double target_size_px = 150.0;
    double max_turn_speed = 1.2;
    double max_forward_speed = 1.6;
    double max_backward_speed = 1.6;
    double max_wheel_speed = 2;
    double centering_threshold_px = 15.0;
    double turning_deadzone_px = 50.0;
    double sample_time_s = 0.03;
};

class SequenceController : public rclcpp::Node {
public:
    SequenceController();
    ~SequenceController() = default;

private:
    // --- Parameter handling ---
    ControlParameters params_;
    void declare_and_load_parameters(); 

    // --- State variables ---
    double current_pos_left_m_ = 0.0;
    double current_pos_right_m_ = 0.0;
    geometry_msgs::msg::Point latest_light_pos_;
    rclcpp::Time last_ball_detection_time_;
    bool is_ball_detected_recently_ = false;
    const rclcpp::Duration detection_timeout_ = rclcpp::Duration(1, 0); // Hardcoded timeout which is 1 second

    // --- ROS communications ---
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_light_pos_;
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr subscription_xeno2ros_;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_ros2xeno_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Callback functions ---
    void light_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void xeno_feedback_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg);
    void control_loop_callback();

    // --- Control logic helpers ---
    struct VelocityCommands {
        double forward = 0.0;
        double turn = 0.0;
    };
    struct WheelVelocities {
        double left = 0.0;
        double right = 0.0;
    };

    void update_detection_status();
    VelocityCommands calculate_velocity_commands();
    WheelVelocities convert_to_wheel_velocities(const VelocityCommands& cmd);
    void publish_wheel_velocities(const WheelVelocities& wheel_vel);

};

#endif // SEQUENCE_CONTROLLER_HPP_