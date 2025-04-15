#include <chrono>
#include <string>
#include <cmath> // For fabs
#include <algorithm> // For std::clamp


#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include "xrf2_msgs/msg/xeno2_ros.hpp"  // Xeno2Ros message from XRF2_msgs
#include "xrf2_msgs/msg/ros2_xeno.hpp"// Ros2Xeno message from XRF2_msgs

using std::placeholders::_1;

using namespace std::chrono_literals;

class SequenceController : public rclcpp::Node {
  public:
    SequenceController() : Node("sequence_controller"){
        sample_time_s_ = 0.03;

        // --- Parameters but we need to tune them ---
        this->declare_parameter<double>("turn_gain", 0.006);   // P-gain for turning based on horizontal pixel error
        this->declare_parameter<double>("forward_gain", 0.008); // P-gain for forward/backward speed based on size error of the ballie
        this->declare_parameter<double>("target_x_pixel", 320.0); // Middle of the camera screen (width = 640)
        this->declare_parameter<double>("target_size_px", 80.0);  // Target ball diameter in pixels 
        this->declare_parameter<double>("max_turn_speed", 0.6);   // Max turning component rad/s 
        this->declare_parameter<double>("max_forward_speed", 0.3);// Max forward speed component (m/s)
        this->declare_parameter<double>("max_backward_speed", 0.1);// Max backward speed component (m/s)
        this->declare_parameter<double>("max_wheel_speed", 0.6);   // Max individual wheel speed (m/s) - Robot limit
        this->declare_parameter<double>("centering_threshold_px", 30.0); // Pixel threshold to allow forward/backward motion

        // --- Subscriptions ---
        subscription_light_pos_ =
            this->create_subscription<geometry_msgs::msg::Point>(
                "light_position", 10, // Topic published by ball_tracker
                std::bind(&SequenceController::update_light_pos, this, _1));

        subscription_xeno2ros_ =
            this->create_subscription<xrf2_msgs::msg::Xeno2Ros>(
                "/Xeno2Ros", 10, // Topic published by the bridge (feedback from Xenomai)
                std::bind(&SequenceController::handle_xeno_feedback, this, _1));

        // --- Publisher ---
        publisher_ros2xeno_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>(
            "/Ros2Xeno", 10); // Topic subscribed by the bridge (commands to Xenomai)

        // --- Timer ---
        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::duration<double>(sample_time_s_),
            std::bind(&SequenceController::control_loop_callback, this));

        // For no detection initiallly
        light_pos_.x = -1.0;
        light_pos_.y = -1.0;
        light_pos_.z = -1.0; //diameter ball

        last_ball_detection_time_ = this->get_clock()->now();
    }


private:
    // Feedback from Xenomai 
    double current_pos_left_m_ = 0.0;
    double current_pos_right_m_ = 0.0;

    // Latest ball detection data
    geometry_msgs::msg::Point light_pos_;
    rclcpp::Time last_ball_detection_time_;

    // Control loop variables
    double sample_time_s_;

    // ROS Comms
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_light_pos_;
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr subscription_xeno2ros_;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_ros2xeno_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Function  ---
    void update_light_pos(const geometry_msgs::msg::Point &msg) {
        light_pos_ = msg;
        if (light_pos_.z > 0){
            last_ball_detection_time_ = this->get_clock()->now();
        }
    }

    void handle_xeno_feedback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
        current_pos_left_m_ = msg->encoder_left;
        current_pos_right_m_ = msg->encoder_right;
    }

    // --- Main Control Logic ---
    void control_loop_callback() {
        // Get parameters
        double turn_gain = this->get_parameter("turn_gain").as_double();
        double forward_gain = this->get_parameter("forward_gain").as_double();
        double target_x = this->get_parameter("target_x_pixel").as_double();
        double target_size = this->get_parameter("target_size_px").as_double();
        double max_turn_speed = this->get_parameter("max_turn_speed").as_double();
        double max_forward_speed = this->get_parameter("max_forward_speed").as_double();
        double max_backward_speed = this->get_parameter("max_backward_speed").as_double();
        double max_wheel_speed = this->get_parameter("max_wheel_speed").as_double();
        double centering_threshold = this->get_parameter("centering_threshold_px").as_double();

        double target_vel_left = 0.0;
        double target_vel_right = 0.0;
        double forward_velocity_cmd = 0.0;
        double turn_velocity_cmd = 0.0;

        rclcpp::Time current_time = this->get_clock()->now();
        bool ball_detected_recently = (current_time - last_ball_detection_time_) < rclcpp::Duration(1, 0);

        if (light_pos_.x >= 0 && light_pos_.z > 0 && ball_detected_recently) { // Check for valid position and size
            // --- Calculate turning velocity ---
            double error_x = target_x - light_pos_.x;
            turn_velocity_cmd = turn_gain * error_x;
            turn_velocity_cmd = std::clamp(turn_velocity_cmd, -max_turn_speed, max_turn_speed);

            // --- Calculate forward/backward velocity (only if centered) ---
            if (std::fabs(error_x) < centering_threshold) {
                double current_ball_size = light_pos_.z; // Diameter from ball_tracker
                double error_size = target_size - current_ball_size; // Target - Current
                forward_velocity_cmd = forward_gain * error_size;

                // Clamp forward/backward speed separately
                forward_velocity_cmd = std::clamp(forward_velocity_cmd, -max_backward_speed, max_forward_speed);
            } else {
                // Not centered enough, don't do forward/backward motion
                forward_velocity_cmd = 0.0;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Ball X:%.1f, ErrX:%.1f, Size:%.1f, ErrSize:%.1f | TurnCmd:%.2f, FwdCmd:%.2f",
                light_pos_.x, error_x, light_pos_.z, (target_size - light_pos_.z), turn_velocity_cmd, forward_velocity_cmd);

            // --- Convert forward/turn to wheel velocities ---
            target_vel_left = forward_velocity_cmd - turn_velocity_cmd;
            target_vel_right = forward_velocity_cmd + turn_velocity_cmd;

        } else {
            // No ball detected or detection is old, command robot to stop
            target_vel_left = 0.0;
            target_vel_right = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No valid/recent ball detection, stopping.");
        }

        // Clamp final wheel velocities
        target_vel_left = std::clamp(target_vel_left, -max_wheel_speed, max_wheel_speed);
        target_vel_right = std::clamp(target_vel_right, -max_wheel_speed, max_wheel_speed);

        // --- Prepare and publish command message ---
        auto ros_cmd_msg = xrf2_msgs::msg::Ros2Xeno();

        // Assign to message fields (VERIFY FIELD NAMES 'example_a'/'example_b')
        ros_cmd_msg.example_a = target_vel_left;  // Target SetVelLeft for LoopController
        ros_cmd_msg.example_b = target_vel_right; // Target SetVelRight for LoopController

        publisher_ros2xeno_->publish(ros_cmd_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();

    return 0;
}
