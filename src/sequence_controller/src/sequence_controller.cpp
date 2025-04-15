#include <chrono>
#include <string>
#include <cmath>      
#include <algorithm> 

#include <rclcpp/rclcpp.hpp>
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// States
enum class MotionState {
    INIT,
    DRIVING_STRAIGHT,
    TURNING,
    STOPPED
};

class SequenceController : public rclcpp::Node {
public:
    SequenceController() : Node("sequence_controller_simple_test") {
        sample_time_s_ = 0.03; 

        // --- Parameters ---
        this->declare_parameter<double>("straight_speed", 0.2);   // Speed for driving straight (m/s)
        this->declare_parameter<double>("straight_duration", 3.0); // How long to drive straight (seconds)
        this->declare_parameter<double>("turn_speed_diff", 0.4); // Speed difference for turning (m/s)
        this->declare_parameter<double>("turn_duration", 2.5);    // How long to turn (seconds) - TUNE THIS FOR 90 DEG!
        this->declare_parameter<double>("max_wheel_speed", 0.6);  // Max individual wheel speed (m/s) - Robot limit

        // --- Subscription
        subscription_xeno2ros_ =
            this->create_subscription<xrf2_msgs::msg::Xeno2Ros>(
                "/Xeno2Ros", 10, // Topic published by the bridge (feedback from Xenomai)
                std::bind(&SequenceController::handle_xeno_feedback, this, _1));

        // --- Publisher 
        publisher_ros2xeno_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
            "/Ros2Xeno", 10); // Topic subscribed by the bridge

        // --- Timer ---
        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::duration<double>(sample_time_s_),
            std::bind(&SequenceController::control_loop_callback, this));

        // --- Initialize state ---
        current_motion_state_ = MotionState::INIT;

        RCLCPP_INFO(this->get_logger(), "Sequence controller initialized.");
    }

private:
    double current_pos_left_m_ = 0.0;
    double current_pos_right_m_ = 0.0;

    // State machine variables
    MotionState current_motion_state_;
    rclcpp::Time state_start_time_;
    double sample_time_s_;

    // ROS Comms
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr subscription_xeno2ros_;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_ros2xeno_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Functions ---
    void handle_xeno_feedback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
        current_pos_left_m_ = msg->encoder_left;
        current_pos_right_m_ = msg->encoder_right;
    }

    // --- Main control ---
    void control_loop_callback() {
        // Get parameters
        double straight_speed = this->get_parameter("straight_speed").as_double();
        double straight_duration = this->get_parameter("straight_duration").as_double();
        double turn_speed_diff = this->get_parameter("turn_speed_diff").as_double();
        double turn_duration = this->get_parameter("turn_duration").as_double();
        double max_wheel_speed = this->get_parameter("max_wheel_speed").as_double();

        double target_vel_left = 0.0;
        double target_vel_right = 0.0;
        rclcpp::Time current_time = this->now();

        switch (current_motion_state_) {
            case MotionState::INIT:
                RCLCPP_INFO(this->get_logger(), "Starting sequence: Driving Straight");
                current_motion_state_ = MotionState::DRIVING_STRAIGHT;
                state_start_time_ = current_time;
                target_vel_left = straight_speed;
                target_vel_right = straight_speed;
                break; 

            case MotionState::DRIVING_STRAIGHT:
            { 
                rclcpp::Duration time_in_state = current_time - state_start_time_;
                target_vel_left = straight_speed;
                target_vel_right = straight_speed;
                if (time_in_state.seconds() >= straight_duration) {
                    RCLCPP_INFO(this->get_logger(), "Driving straight complete. Starting Turn.");
                    current_motion_state_ = MotionState::TURNING;
                    state_start_time_ = current_time; 
                    target_vel_left = -turn_speed_diff / 2.0;
                    target_vel_right = turn_speed_diff / 2.0;
                }
                break; 
            } 

            case MotionState::TURNING:
            { 
                rclcpp::Duration time_in_state = current_time - state_start_time_;
                // Turn left 
                target_vel_left = -turn_speed_diff / 2.0;
                target_vel_right = turn_speed_diff / 2.0;

                if (time_in_state.seconds() >= turn_duration) {
                    RCLCPP_INFO(this->get_logger(), "Turn complete. Stopping.");
                    current_motion_state_ = MotionState::STOPPED;
                    // Set final command to stop
                    target_vel_left = 0.0;
                    target_vel_right = 0.0;
                }
                break; 
            } // End 

            case MotionState::STOPPED:
                target_vel_left = 0.0;
                target_vel_right = 0.0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Sequence complete. Robot stopped. Final Pos L: %.3f, R: %.3f", current_pos_left_m_, current_pos_right_m_);
                break;
        }

        // Clamp final wheel velocities
        target_vel_left = std::clamp(target_vel_left, -max_wheel_speed, max_wheel_speed);
        target_vel_right = std::clamp(target_vel_right, -max_wheel_speed, max_wheel_speed);

        if (current_motion_state_ != MotionState::STOPPED || timer_->is_canceled() == false) {
             auto ros_cmd_msg = xrf2_msgs::msg::Ros2Xeno();
             ros_cmd_msg.example_a = target_vel_left;  // Target SetVelLeft for LoopController
             ros_cmd_msg.example_b = target_vel_right; // Target SetVelRight for LoopController
             publisher_ros2xeno_->publish(ros_cmd_msg);
        }

    }
}; 

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}