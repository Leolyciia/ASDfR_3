
#ifndef SEQUENCE_CONTROLLER_HPP_
#define SEQUENCE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using namespace std::chrono_literals;

class SequenceController : public rclcpp::Node {
public:
    SequenceController();
private:
    enum class Phase { DRIVE, WAIT, TURN, DONE } phase_ = Phase::DRIVE;
    rclcpp::Subscription<xrf2_msgs::msg::Xeno2Ros>::SharedPtr sub_pose_;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool pose_initialized_ = false;
    double start_x_ = 0.0, start_y_ = 0.0, start_theta_ = 0.0;
    double pose_x_ = 0.0, pose_y_ = 0.0, pose_theta_ = 0.0;
    rclcpp::Time phase_start_time_;
    static constexpr double DRIVE_SPEED = 0.5;
    static constexpr double TURN_SPEED = 0.5;
    static constexpr double DRIVE_DISTANCE = 0.1;
    static constexpr double TURN_ANGLE = -M_PI/4.0;
    static constexpr double WAIT_DURATION = 1.0;
    double sample_time_s_;
    void pose_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg);
    void control_loop_callback();
};

#endif // SEQUENCE_CONTROLLER_HPP_
