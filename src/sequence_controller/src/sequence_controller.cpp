#include <chrono>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include "xrf2_msgs/msg/xeno2_ros.hpp"  // Xeno2Ros message from XRF2_msgs


using std::placeholders::_1;

using namespace std::chrono_literals;

class SequenceController : public rclcpp::Node {
  public:
    SequenceController() : Node("sequence_controller"), count_(0) {
        sample_time_s_ = 0.03;

        subscription_light_pos_ =
            this->create_subscription<geometry_msgs::msg::Point>(
                "light_position", 10,
                std::bind(&SequenceController::update_light_pos, this, _1));

        subscription_xeno2ros_ = this->create_subscription<xrf2_msgs::msg::Xeno2Ros>(
            "/Xeno2Ros", 10, 
            std::bind(&SequenceController::handle_xeno_feedback, this, _1));


        publisher_left_ = this->create_publisher<std_msgs::msg::Float64>(
            "left_motor_setpoint_vel", 10);

        publisher_right_ = this->create_publisher<std_msgs::msg::Float64>(
            "right_motor_setpoint_vel", 10);

        timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            std::chrono::duration<double>(sample_time_s_),
            std::bind(&SequenceController::sequence_controller, this));

        this->declare_parameter("gain", 0.2);
        this->declare_parameter("width", 360);
    }

  private:
    double encoder_left_ = 0.0;
    double encoder_right_ = 0.0;
    void sequence_controller() {
        auto gain = this->get_parameter("gain").as_double();
        auto width = this->get_parameter("width").as_int();

        double e = gain * (light_pos_.x - (width / 2));

        RCLCPP_INFO(this->get_logger(), "light_pos.x: %f, e: %f", light_pos_.x,
                    e);

        auto vel_left = std_msgs::msg::Float64();
        auto vel_right = std_msgs::msg::Float64();

        vel_left.data = e;
        vel_right.data = -e;

        publisher_left_->publish(vel_left);
        publisher_right_->publish(vel_right);

        rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr subscription_xeno2ros_;

    }

    void update_light_pos(const geometry_msgs::msg::Point &msg) {
        if (msg.x == -1)
            return;

        light_pos_.x = msg.x;
        light_pos_.y = msg.y;
    }

    void handle_xeno_feedback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Encoder Left: %.2f, Encoder Right: %.2f", 
                    msg->encoder_left, msg->encoder_right);
    
        encoder_left_ = msg->encoder_left;
        encoder_right_ = msg->encoder_right;
    }

    size_t count_;
    double sample_time_s_;

    geometry_msgs::msg::Point light_pos_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        subscription_light_pos_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        subscription_dim_;
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr
        subscription_xeno2ros_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();

    return 0;
}
