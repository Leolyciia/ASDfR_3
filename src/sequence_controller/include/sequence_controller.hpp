#ifndef SEQUENCE_CONTROLLER_HPP_
#define SEQUENCE_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm> // Needed for std::clamp

#include <rclcpp/rclcpp.hpp>
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using namespace std::chrono_literals;

// --- Constants ---
const double WHEEL_BASE_WIDTH = 0.21; // meters - Updated as per your request
const double PI = 3.141592653589793;
const double TARGET_DISTANCE_M = 0.30; // meters - Example: Drive 1 meter straight
const double TARGET_TURN_ANGLE_RAD = PI / 2.0; // 90 degrees in radians
const double FORWARD_SPEED = 0.5; // m/s - Example forward speed
const double TURN_SPEED = 0.4; // rad/s equivalent for wheels - Example turning speed

// --- Control Parameters ---
// Simplified parameters for this task
struct ControlParameters {
    double max_wheel_speed = 0.9; // Maximum allowable individual wheel speed
    double sample_time_s = 0.03; // Control loop sample time
};

// --- Robot States ---
enum class RobotState {
    DRIVING_STRAIGHT,
    TURNING,
    STOPPED
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
    RobotState current_state_ = RobotState::DRIVING_STRAIGHT;
    double current_pos_left_m_ = 0.0;
    double current_pos_right_m_ = 0.0;
    double start_pos_left_m_ = 0.0;  // Encoder reading when starting DRIVING_STRAIGHT
    double start_pos_right_m_ = 0.0; // Encoder reading when starting DRIVING_STRAIGHT
    double start_turn_pos_left_m_ = 0.0; // Encoder reading when starting TURNING
    double start_turn_pos_right_m_ = 0.0;// Encoder reading when starting TURNING
    bool initial_encoder_reading_received_ = false; // Flag to wait for first encoder reading


    // --- ROS communications ---
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr subscription_xeno2ros_;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_ros2xeno_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Callback functions ---
    void xeno_feedback_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg);
    void control_loop_callback();

    // --- Control logic helpers ---
    struct WheelVelocities {
        double left = 0.0;
        double right = 0.0;
    };

    void publish_wheel_velocities(const WheelVelocities& wheel_vel);

};

#endif // SEQUENCE_CONTROLLER_HPP_