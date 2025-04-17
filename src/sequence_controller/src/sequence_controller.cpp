#include "sequence_controller.hpp"
#include <cmath>

// Constructor
SequenceController::SequenceController()
    : Node("sequence_controller")
{
    RCLCPP_INFO(this->get_logger(), "Initialising SequenceController node for distance and turn using ODOMETRY...");

    // Declare and load parameters
    declare_and_load_parameters();

    // --- Subscription ---
    subscription_xeno2ros_ = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>(
        "/Xeno2Ros", 10,
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
    this->declare_parameter<double>("max_wheel_speed", params_.max_wheel_speed);
    this->declare_parameter<double>("sample_time_s", params_.sample_time_s);
    this->get_parameter("max_wheel_speed", params_.max_wheel_speed);
    this->get_parameter("sample_time_s", params_.sample_time_s);
    RCLCPP_INFO(this->get_logger(), "Declared and loaded parameters: max_wheel_speed=%.2f, sample_time_s=%.3f",
                params_.max_wheel_speed, params_.sample_time_s);
}

double SequenceController::normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}


// --- Callback functions ---
void SequenceController::xeno_feedback_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_theta_ = msg->theta;

    if (!initial_pose_received_) {
        start_x_ = current_x_;
        start_y_ = current_y_;
        initial_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose received: X=%.3f m, Y=%.3f m, Theta=%.3f rad",
                    start_x_, start_y_, current_theta_);
    }
}


void SequenceController::publish_wheel_velocities(const WheelVelocities& wheel_vel) {
    auto ros_cmd_msg = std::make_unique<xrf2_msgs::msg::Ros2Xeno>();
    ros_cmd_msg->example_a = std::clamp(wheel_vel.left, -params_.max_wheel_speed, params_.max_wheel_speed);
    ros_cmd_msg->example_b = std::clamp(wheel_vel.right, -params_.max_wheel_speed, params_.max_wheel_speed);
    publisher_ros2xeno_->publish(std::move(ros_cmd_msg));
}

// --- Main control loop ---
void SequenceController::control_loop_callback() {
    // Wait until we have received the first pose reading
    if (!initial_pose_received_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial pose readings...");
        publish_wheel_velocities({0.0, 0.0});
        return;
    }

    WheelVelocities wheel_vel = {0.0, 0.0}; // Default to stopped

    switch (current_state_) {
        case RobotState::DRIVING_STRAIGHT: {
            // Calculate distance traveled from the start pose
            double delta_x = current_x_ - start_x_;
            double delta_y = current_y_ - start_y_;
            double distance_traveled = std::sqrt(delta_x * delta_x + delta_y * delta_y);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "State: DRIVING_STRAIGHT, Distance: %.3f / %.3f m",
                                 distance_traveled, TARGET_DISTANCE_M);

            if (distance_traveled < TARGET_DISTANCE_M) {
                // Continue driving straight
                wheel_vel.left = FORWARD_SPEED;
                wheel_vel.right = FORWARD_SPEED;
            } else {
                // Reached target distance, transition to STOPPING_BEFORE_TURN
                RCLCPP_INFO(this->get_logger(), "Target distance reached. Transitioning to STOPPING_BEFORE_TURN.");
                current_state_ = RobotState::STOPPING_BEFORE_TURN;
                stopping_cycles_counter_ = 0; // Reset counter
                wheel_vel.left = 0.0;         // Command stop
                wheel_vel.right = 0.0;
            }
            break;
        }

        case RobotState::STOPPING_BEFORE_TURN: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                                 "State: STOPPING_BEFORE_TURN, Cycle: %d / %d",
                                 stopping_cycles_counter_, STOPPING_DURATION_CYCLES);

            // Command stop
            wheel_vel.left = 0.0;
            wheel_vel.right = 0.0;

            stopping_cycles_counter_++;

            if (stopping_cycles_counter_ >= STOPPING_DURATION_CYCLES) {
                 // Waited long enough, transition to TURNING
                RCLCPP_INFO(this->get_logger(), "Stopping period complete. Transitioning to TURNING.");
                current_state_ = RobotState::TURNING;
                start_turn_theta_ = current_theta_;
                wheel_vel.left = TURN_SPEED;
                wheel_vel.right = -TURN_SPEED;
            }
            break;
        }

        case RobotState::TURNING: {
            double angle_diff = current_theta_ - start_turn_theta_;
            double angle_turned_rad = normalize_angle(angle_diff);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                 "State: TURNING, Angle Turned: %.3f / Target: %.3f rad (Current Theta: %.3f, Start Theta: %.3f)",
                                 std::abs(angle_turned_rad), TARGET_TURN_ANGLE_RAD, current_theta_, start_turn_theta_);

            // Compare absolute angle turned to target angle
            if (std::abs(angle_turned_rad) < TARGET_TURN_ANGLE_RAD) {
                // Continue turning
                wheel_vel.left = TURN_SPEED;
                wheel_vel.right = -TURN_SPEED;
            } else {
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