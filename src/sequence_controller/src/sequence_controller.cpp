#include "sequence_controller.hpp"

// Constructor
SequenceController::SequenceController()
    : Node("sequence_controller")
{
    RCLCPP_INFO(this->get_logger(), "Initialising SequenceController node for distance and turn...");

    // Declare and load parameters
    declare_and_load_parameters();

    // --- Subscription ---
    subscription_xeno2ros_ = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>(
        "/Xeno2Ros", 10, // Use rclcpp::SensorDataQoS() ? Check what xeno_loop provides
        std::bind(&SequenceController::xeno_feedback_callback, this, std::placeholders::_1));

    // --- Publisher ---
    publisher_ros2xeno_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
        "/Ros2Xeno", 10);

    // --- Timer ---
    timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        std::chrono::duration<double>(params_.sample_time_s),
        std::bind(&SequenceController::control_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "SequenceController Node initialised successfully.");
}

// --- Parameter handling ---
void SequenceController::declare_and_load_parameters() {
    // Declare only the parameters needed now
    this->declare_parameter<double>("max_wheel_speed", params_.max_wheel_speed);
    this->declare_parameter<double>("sample_time_s", params_.sample_time_s);

    // Load actual values into the struct
    this->get_parameter("max_wheel_speed", params_.max_wheel_speed);
    this->get_parameter("sample_time_s", params_.sample_time_s);

    RCLCPP_INFO(this->get_logger(), "Declared and loaded parameters: max_wheel_speed=%.2f, sample_time_s=%.3f",
                params_.max_wheel_speed, params_.sample_time_s);
}

// --- Callback functions ---
void SequenceController::xeno_feedback_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
    double new_pos_left = msg->encoder_left;
    double new_pos_right = msg->encoder_right;

    // Store the initial reading when received for the first time
    if (!initial_encoder_reading_received_) {
        start_pos_left_m_ = new_pos_left;
        start_pos_right_m_ = new_pos_right;
        initial_encoder_reading_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial encoder readings received: Left=%.4f m, Right=%.4f m",
                    start_pos_left_m_, start_pos_right_m_);
    }

    current_pos_left_m_ = new_pos_left;
    current_pos_right_m_ = new_pos_right;

    // Optional: Log encoder values periodically for debugging
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, // Log every 1 second
    //                      "Encoders: Left=%.4f m, Right=%.4f m",
    //                      current_pos_left_m_, current_pos_right_m_);
}


void SequenceController::publish_wheel_velocities(const WheelVelocities& wheel_vel) {
    auto ros_cmd_msg = std::make_unique<xrf2_msgs::msg::Ros2Xeno>();

    // Clamp velocities to the maximum allowed wheel speed
    ros_cmd_msg->example_a = std::clamp(wheel_vel.left, -params_.max_wheel_speed, params_.max_wheel_speed);
    ros_cmd_msg->example_b = std::clamp(wheel_vel.right, -params_.max_wheel_speed, params_.max_wheel_speed);

    publisher_ros2xeno_->publish(std::move(ros_cmd_msg));
}

// --- Main control loop ---
void SequenceController::control_loop_callback() {
    // Wait until we have received the first encoder reading
    if (!initial_encoder_reading_received_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial encoder readings...");
        // Publish zero velocity while waiting
        publish_wheel_velocities({0.0, 0.0});
        return;
    }

    WheelVelocities wheel_vel = {0.0, 0.0}; // Default to stopped

    switch (current_state_) {
        case RobotState::DRIVING_STRAIGHT: {
            // Calculate average distance traveled since starting this state
            double dist_left = current_pos_left_m_ - start_pos_left_m_;
            double dist_right = current_pos_right_m_ - start_pos_right_m_;
            double avg_distance = (dist_left + dist_right) / 2.0;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, // Log every 0.5 seconds
                                 "State: DRIVING_STRAIGHT, Distance: %.3f / %.3f m",
                                 avg_distance, TARGET_DISTANCE_M);

            if (avg_distance < TARGET_DISTANCE_M) {
                // Continue driving straight
                wheel_vel.left = FORWARD_SPEED;
                wheel_vel.right = FORWARD_SPEED;
            } else {
                // Reached target distance, transition to TURNING
                RCLCPP_INFO(this->get_logger(), "Target distance reached. Transitioning to TURNING.");
                current_state_ = RobotState::TURNING;
                // Record encoder positions at the start of the turn
                start_turn_pos_left_m_ = current_pos_left_m_;
                start_turn_pos_right_m_ = current_pos_right_m_;
                // Start turning (e.g., right turn: left wheel forward, right wheel backward)
                wheel_vel.left = TURN_SPEED; // Adjust speed/direction as needed
                wheel_vel.right = -TURN_SPEED;
            }
            break;
        }

        case RobotState::TURNING: {
            // Calculate distance traveled by each wheel *during the turn*
            double turn_dist_left = current_pos_left_m_ - start_turn_pos_left_m_;
            double turn_dist_right = current_pos_right_m_ - start_turn_pos_right_m_;

            // Calculate angle turned based on differential distance and wheel base
            // Assuming right turn: right wheel moves backward (negative dist), left moves forward (positive dist)
            // Angle = (arc_length_right - arc_length_left) / wheel_base
            // arc_length_right is negative, arc_length_left is positive for a right turn
            double angle_turned_rad = (turn_dist_right - turn_dist_left) / WHEEL_BASE_WIDTH;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, // Log every 0.2 seconds
                                 "State: TURNING, Angle: %.3f / %.3f rad",
                                 std::abs(angle_turned_rad), TARGET_TURN_ANGLE_RAD);


            // Note: std::abs() is used to compare against the target angle magnitude.
            // The sign of angle_turned_rad indicates direction (+ for left, - for right in this setup).
            if (std::abs(angle_turned_rad) < TARGET_TURN_ANGLE_RAD) {
                // Continue turning
                wheel_vel.left = TURN_SPEED; // Keep applying turn speeds
                wheel_vel.right = -TURN_SPEED;
            } else {
                // Target angle reached, transition to STOPPED
                RCLCPP_INFO(this->get_logger(), "Target angle reached. Transitioning to STOPPED.");
                current_state_ = RobotState::STOPPED;
                wheel_vel.left = 0.0;
                wheel_vel.right = 0.0;
            }
            break;
        }

        case RobotState::STOPPED: {
            // Stay stopped
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "State: STOPPED");
            wheel_vel.left = 0.0;
            wheel_vel.right = 0.0;
            // Optional: Cancel the timer if the sequence is truly finished
            // timer_->cancel();
            break;
        }
    }

    // Publish the calculated wheel velocities
    publish_wheel_velocities(wheel_vel);
}

// --- Main function ---
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto sequence_controller_node = std::make_shared<SequenceController>();
    rclcpp::spin(sequence_controller_node);
    rclcpp::shutdown();
    return 0;
}