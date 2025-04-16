#include "sequence_controller.hpp"

// Constructor
SequenceController::SequenceController()
    : Node("sequence_controller")
{
    RCLCPP_INFO(this->get_logger(), "Inialise");

    // declaring and loading the parameters
    declare_and_load_parameters();

    // --- subscriptions ---
    subscription_xeno2ros_ = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>(
        "/Xeno2Ros", 10,
        std::bind(&SequenceController::xeno_feedback_callback, this, std::placeholders::_1));

    // --- Publisher ---
    publisher_ros2xeno_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
        "/Ros2Xeno", 10);

    // --- timer ---
    timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        std::chrono::duration<double>(params_.sample_time_s),
        std::bind(&SequenceController::control_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "SequenceController Node initialised successfully for timed sequence.");

}

// --- Parameter handling ---
void SequenceController::declare_and_load_parameters() {
    // Declare new parameters
    this->declare_parameter<double>("forward_speed", params_.forward_speed);
    this->declare_parameter<double>("turn_speed", params_.turn_speed);
    this->declare_parameter<double>("forward_duration_s", params_.forward_duration_s);
    this->declare_parameter<double>("turn_duration_90_deg_s", params_.turn_duration_90_deg_s);
    this->declare_parameter<double>("sample_time_s", params_.sample_time_s);
    this->declare_parameter<double>("max_wheel_speed", params_.max_wheel_speed);

    // Load actual values into the struct
    this->get_parameter("forward_speed", params_.forward_speed);
    this->get_parameter("turn_speed", params_.turn_speed);
    this->get_parameter("forward_duration_s", params_.forward_duration_s);
    this->get_parameter("turn_duration_90_deg_s", params_.turn_duration_90_deg_s);
    this->get_parameter("sample_time_s", params_.sample_time_s);
    this->get_parameter("max_wheel_speed", params_.max_wheel_speed);

    RCLCPP_INFO(this->get_logger(), "Declared and loaded sequence parameters.");
    RCLCPP_INFO(this->get_logger(), "Forward: %.2f m/s for %.2f s", params_.forward_speed, params_.forward_duration_s);
    RCLCPP_INFO(this->get_logger(), "Turn: %.2f rad/s for %.2f s", params_.turn_speed, params_.turn_duration_90_deg_s);
}


SequenceController::WheelVelocities SequenceController::convert_to_wheel_velocities(const MotionCommand& cmd) {
    WheelVelocities wheel_vel;
    wheel_vel.left = cmd.forward - cmd.turn / 2.0; 
    wheel_vel.right = cmd.forward + cmd.turn / 2.0;

    wheel_vel.left = std::clamp(wheel_vel.left, -params_.max_wheel_speed, params_.max_wheel_speed);
    wheel_vel.right = std::clamp(wheel_vel.right, -params_.max_wheel_speed, params_.max_wheel_speed);
    RCLCPP_DEBUG(this->get_logger(), "Command Fwd: %.2f Turn: %.2f | Wheel Vel L:%.2f, R:%.2f",
                 cmd.forward, cmd.turn, wheel_vel.left, wheel_vel.right);
    return wheel_vel;
}

void SequenceController::publish_wheel_velocities(const WheelVelocities& wheel_vel) {
    auto ros_cmd_msg = std::make_unique<xrf2_msgs::msg::Ros2Xeno>();
    ros_cmd_msg->example_a = wheel_vel.left;  
    ros_cmd_msg->example_b = wheel_vel.right; 
    publisher_ros2xeno_->publish(std::move(ros_cmd_msg));
}

void SequenceController::stop_robot() {
    publish_wheel_velocities({0.0, 0.0});
    RCLCPP_INFO(this->get_logger(), "Commanding robot to stop.");
}


// --- Main control loop ---
void SequenceController::control_loop_callback() {
    MotionCommand current_cmd = {0.0, 0.0};
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Duration time_in_state = now - state_start_time_;

    switch (current_state_) {
        case RobotState::IDLE:
            RCLCPP_INFO(this->get_logger(), "Starting sequence: Moving Forward.");
            current_state_ = RobotState::MOVING_FORWARD;
            state_start_time_ = now; 
        
        case RobotState::MOVING_FORWARD:
            if (time_in_state.seconds() < params_.forward_duration_s) {
                current_cmd.forward = params_.forward_speed;
                current_cmd.turn = 0.0;
                RCLCPP_DEBUG(this->get_logger(), "State: FORWARD (%.2f/%.2f s)", time_in_state.seconds(), params_.forward_duration_s);
            } else {
                RCLCPP_INFO(this->get_logger(), "Forward complete. Starting Turn.");
                current_state_ = RobotState::TURNING;
                state_start_time_ = now; // Reset timer for the new state
                current_cmd.forward = 0.0; // Stop forward motion before turning
                current_cmd.turn = params_.turn_speed; // Start turning
                RCLCPP_DEBUG(this->get_logger(), "State: TURNING (%.2f/%.2f s)", 0.0, params_.turn_duration_90_deg_s);
            }
            break;

        case RobotState::TURNING:
            if (time_in_state.seconds() < params_.turn_duration_90_deg_s) {
                current_cmd.forward = 0.0;
                current_cmd.turn = params_.turn_speed; 
                 RCLCPP_DEBUG(this->get_logger(), "State: TURNING (%.2f/%.2f s)", time_in_state.seconds(), params_.turn_duration_90_deg_s);
            } else {
                RCLCPP_INFO(this->get_logger(), "Turn complete. Sequence Finished.");
                current_state_ = RobotState::FINISHED;
                stop_robot(); 
            }
            break;

        case RobotState::FINISHED:
            return; 
    }

    // Convert motion command to wheel velocities and publish 
    if (current_state_ != RobotState::FINISHED) {
        WheelVelocities wheel_vel = convert_to_wheel_velocities(current_cmd);
        publish_wheel_velocities(wheel_vel);
    }
}

// --- Main function ---
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto sequence_controller_node = std::make_shared<SequenceController>();
    rclcpp::spin(sequence_controller_node);
    rclcpp::shutdown();
    return 0;
}