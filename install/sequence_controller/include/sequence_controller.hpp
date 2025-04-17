#ifndef SEQUENCE_CONTROLLER_HPP_
#define SEQUENCE_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm> 

#include <rclcpp/rclcpp.hpp>
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using namespace std::chrono_literals;

// --- Constants ---
const double WHEEL_BASE_WIDTH = 0.21; 
const double PI = 3.141592653589793;
const double TARGET_DISTANCE_M = 0.10; 
const double TARGET_TURN_ANGLE_RAD = PI / 2.0;
const double FORWARD_SPEED = 0.5;
const double TURN_SPEED = 0.4;
const int STOPPING_DURATION_CYCLES = 20; 


// --- Control Parameters ---
struct ControlParameters {
    double max_wheel_speed = 0.9; 
    double sample_time_s = 0.03; // Control loop sample time
};

// --- Robot States ---
enum class RobotState {
    DRIVING_STRAIGHT,
    STOPPING_BEFORE_TURN,
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
    // Store current pose from feedback
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_turn_theta_ = 0.0;
    bool initial_pose_received_ = false;
    int stopping_cycles_counter_ = 0;


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
    double normalize_angle(double angle); 

};

#endif // SEQUENCE_CONTROLLER_HPP_