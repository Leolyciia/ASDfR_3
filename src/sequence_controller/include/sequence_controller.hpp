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

// --- State machine ---
enum class RobotState {
    IDLE,
    MOVING_FORWARD,
    TURNING,
    FINISHED
};

// --- Control parameters ---
struct ControlParameters {
    double forward_speed = 0.2;       // Speed for moving forward (m/s)
    double turn_speed = 0.3;          // Rotational speed for turning (rad/s) 
    double forward_duration_s = 5.0;  // Duration to move forward (seconds)
    double turn_duration_90_deg_s = 2.0; // Duration to turn ~90 degrees (seconds) 
    double sample_time_s = 0.03;      // Control loop frequency
    double max_wheel_speed = 0.3;     // Safety limit for individual wheel speed (m/s)
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
    RobotState current_state_ = RobotState::IDLE;
    rclcpp::Time state_start_time_;

    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr subscription_xeno2ros_; 
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_ros2xeno_;
    rclcpp::TimerBase::SharedPtr timer_;
    void control_loop_callback();


    struct MotionCommand {
        double forward = 0.0; // Linear velocity
        double turn = 0.0;    // Angular velocity
    };
    struct WheelVelocities {
        double left = 0.0;
        double right = 0.0;
    };

    WheelVelocities convert_to_wheel_velocities(const MotionCommand& cmd);
    void publish_wheel_velocities(const WheelVelocities& wheel_vel);
    void stop_robot(); 

};

#endif // SEQUENCE_CONTROLLER_HPP_