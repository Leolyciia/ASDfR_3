#include "sequence_controller.hpp" 

// Constructor
SequenceController::SequenceController()
    : Node("sequence_controller"),
      latest_light_pos_(),
      last_ball_detection_time_(this->get_clock()->now())
{
    RCLCPP_INFO(this->get_logger(), "Initialising SequenceController node...");

    // declaring and loading the parameters
    declare_and_load_parameters();

    // Initialise state
    latest_light_pos_.x = -1.0;
    latest_light_pos_.y = -1.0;
    latest_light_pos_.z = -1.0;

    // --- subscriptions ---
    subscription_light_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
        "light_position", 10,
        std::bind(&SequenceController::light_pos_callback, this, std::placeholders::_1));

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

    RCLCPP_INFO(this->get_logger(), "SequenceController Node initialised successfully.");
}

// --- Parameter handling ---
void SequenceController::declare_and_load_parameters() {
    this->declare_parameter<double>("turn_gain", params_.turn_gain);
    this->declare_parameter<double>("forward_gain", params_.forward_gain);
    this->declare_parameter<double>("target_x_pixel", params_.target_x_pixel);
    this->declare_parameter<double>("target_size_px", params_.target_size_px);
    this->declare_parameter<double>("max_turn_speed", params_.max_turn_speed);
    this->declare_parameter<double>("max_forward_speed", params_.max_forward_speed);
    this->declare_parameter<double>("max_backward_speed", params_.max_backward_speed);
    this->declare_parameter<double>("max_wheel_speed", params_.max_wheel_speed);
    this->declare_parameter<double>("centering_threshold_px", params_.centering_threshold_px);
    this->declare_parameter<double>("turning_deadzone_px", params_.turning_deadzone_px);
    this->declare_parameter<double>("sample_time_s", params_.sample_time_s);

    // Load actual values into the struct
    this->get_parameter("turn_gain", params_.turn_gain);
    this->get_parameter("forward_gain", params_.forward_gain);
    this->get_parameter("target_x_pixel", params_.target_x_pixel);
    this->get_parameter("target_size_px", params_.target_size_px);
    this->get_parameter("max_turn_speed", params_.max_turn_speed);
    this->get_parameter("max_forward_speed", params_.max_forward_speed);
    this->get_parameter("max_backward_speed", params_.max_backward_speed);
    this->get_parameter("max_wheel_speed", params_.max_wheel_speed);
    this->get_parameter("centering_threshold_px", params_.centering_threshold_px);
    this->get_parameter("turning_deadzone_px", params_.turning_deadzone_px);
    this->get_parameter("sample_time_s", params_.sample_time_s);

    RCLCPP_INFO(this->get_logger(), "Declared and loaded parameters.");
}

// --- Callback functions ---
void SequenceController::light_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    latest_light_pos_ = *msg;
    if (latest_light_pos_.z > 0) {
        last_ball_detection_time_ = this->get_clock()->now();
    }
}

void SequenceController::xeno_feedback_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
    current_pos_left_m_ = msg->encoder_left;
    current_pos_right_m_ = msg->encoder_right;
}


// --- Control logic helpers ---
void SequenceController::update_detection_status() {
    rclcpp::Time current_time = this->get_clock()->now();
    is_ball_detected_recently_ = (current_time - last_ball_detection_time_) < detection_timeout_;
}

SequenceController::VelocityCommands SequenceController::calculate_velocity_commands() {
    VelocityCommands cmd;
    if (latest_light_pos_.x >= 0 && latest_light_pos_.z > 0 && is_ball_detected_recently_) {
        double error_x = params_.target_x_pixel - latest_light_pos_.x;
        if (std::fabs(error_x) > params_.turning_deadzone_px) {
            cmd.turn = params_.turn_gain * error_x;
            cmd.turn = std::clamp(cmd.turn, -params_.max_turn_speed, params_.max_turn_speed);
        }

        
    if (std::fabs(error_x) < params_.turning_deadzone_px) {
        double current_ball_size = latest_light_pos_.z; // diameter from ball_tracker
        double error_size = params_.target_size_px - current_ball_size; // target - current
        double size_deadzone = 20.0; // tolerance for the ballie

        // Only calculate forward command if outside the deadzone
        if (std::fabs(error_size) > size_deadzone) {
            cmd.forward = params_.forward_gain * error_size;
            // Clamp forward/backward speed separately
            cmd.forward = std::clamp(cmd.forward, -params_.max_backward_speed, params_.max_forward_speed);
        } else {
            // Inside the deadzone, set forward command to zero
            cmd.forward = 0.0;
        }
    } else {
        // Not centered enough, set forward command to zero 
        cmd.forward = 0.0;
    }


        // RCLCPP_INFO(this->get_logger(),
        //     "Ball X:%.1f, ErrX:%.1f, Size:%.1f, ErrSize:%.1f | TurnCmd:%.2f, FwdCmd:%.2f",
        //     latest_light_pos_.x, error_x, latest_light_pos_.z, (params_.target_size_px - latest_light_pos_.z), cmd.turn, cmd.forward);
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No valid/recent ball detection, stopping.");
    }
    return cmd;
}

SequenceController::WheelVelocities SequenceController::convert_to_wheel_velocities(const VelocityCommands& cmd) {
    WheelVelocities wheel_vel;
    wheel_vel.left = cmd.forward - cmd.turn;
    wheel_vel.right = cmd.forward + cmd.turn;
    wheel_vel.left = std::clamp(wheel_vel.left, -params_.max_wheel_speed, params_.max_wheel_speed);
    wheel_vel.right = std::clamp(wheel_vel.right, -params_.max_wheel_speed, params_.max_wheel_speed);
    RCLCPP_DEBUG(this->get_logger(), "Wheel Velocities - Left:%.2f, Right:%.2f", wheel_vel.left, wheel_vel.right);
    return wheel_vel;
}

void SequenceController::publish_wheel_velocities(const WheelVelocities& wheel_vel) {
    auto ros_cmd_msg = std::make_unique<xrf2_msgs::msg::Ros2Xeno>();
    ros_cmd_msg->example_a = wheel_vel.left;
    ros_cmd_msg->example_b = wheel_vel.right;
    publisher_ros2xeno_->publish(std::move(ros_cmd_msg));
}

// --- Main control loop ---
// void SequenceController::control_loop_callback() {
//     update_detection_status();
//     VelocityCommands velocity_cmd = calculate_velocity_commands();
//     WheelVelocities wheel_vel = convert_to_wheel_velocities(velocity_cmd);
//     publish_wheel_velocities(wheel_vel);
// }

void SequenceController::control_loop_callback() {
    // Timestamp and loop period
    rclcpp::Time   now    = this->get_clock()->now();
    static auto    last   = now;
    double         t_sec  = now.seconds();
    double         dt_ms  = (now - last).nanoseconds() * 1e-6;
    last = now;

    // compute cmd.turn and cmd.forward
    update_detection_status();
    VelocityCommands cmd = calculate_velocity_commands();
    WheelVelocities   wheel = convert_to_wheel_velocities(cmd);
    publish_wheel_velocities(wheel);

    // compute the error_x and error_size for logging
    double error_x = params_.target_x_pixel - latest_light_pos_.x;
    double error_size = params_.target_size_px - latest_light_pos_.z;

    // detection flag
    int detected = (latest_light_pos_.z > 0 && is_ball_detected_recently_) ? 1 : 0;

    // print: time[s],dt_ms,detected,errX,errSize,turnCmd,fwdCmd,leftVel,rightVel
    RCLCPP_INFO(this->get_logger(),
        "%.6f,%.3f,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f",
        t_sec,
        dt_ms,
        detected,
        error_x,
        error_size,
        cmd.turn,
        cmd.forward,
        wheel.left,
        wheel.right
    );
}

// --- Main function ---
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto sequence_controller_node = std::make_shared<SequenceController>();
    rclcpp::spin(sequence_controller_node);
    rclcpp::shutdown();
    return 0;
}