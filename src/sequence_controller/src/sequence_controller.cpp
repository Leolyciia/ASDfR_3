#include "sequence_controller.hpp"

SequenceController::SequenceController()
: Node("sequence_controller")
{
    declare_parameter<double>("sample_time_s", 0.01);
    get_parameter("sample_time_s", sample_time_s_);
    sub_pose_ = create_subscription<xrf2_msgs::msg::Xeno2Ros>(
        "/Xeno2Ros", 10,
        std::bind(&SequenceController::pose_callback, this, std::placeholders::_1));
    pub_cmd_ = create_publisher<xrf2_msgs::msg::Ros2Xeno>(
        "/Ros2Xeno", 10);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(sample_time_s_),
        std::bind(&SequenceController::control_loop_callback, this));
}

void SequenceController::pose_callback(const xrf2_msgs::msg::Xeno2Ros::SharedPtr msg)
{
    if (!pose_initialized_) {
        start_x_ = msg->x;
        start_y_ = msg->y;
        start_theta_ = msg->theta;
        pose_initialized_ = true;
    }
    pose_x_ = msg->x;
    pose_y_ = msg->y;
    pose_theta_ = msg->theta;
}

void SequenceController::control_loop_callback()
{
    xrf2_msgs::msg::Ros2Xeno cmd;
    switch (phase_) {
        case Phase::DRIVE: {
            if (!pose_initialized_) {
                cmd.example_a = 0.0;
                cmd.example_b = 0.0;
            } else {
                double dx = pose_x_ - start_x_;
                double dy = pose_y_ - start_y_;
                double dist = std::hypot(dx, dy);
                if (dist < DRIVE_DISTANCE) {
                    cmd.example_a = DRIVE_SPEED;
                    cmd.example_b = DRIVE_SPEED;
                } else {
                    cmd.example_a = 0.0;
                    cmd.example_b = 0.0;
                    phase_ = Phase::WAIT;
                    phase_start_time_ = now();
                }
            }
            break;
        }
        case Phase::WAIT: {
            if ((now() - phase_start_time_).seconds() < WAIT_DURATION) {
                cmd.example_a = 0.0;
                cmd.example_b = 0.0;
            } else {
                phase_ = Phase::TURN;
                start_theta_ = pose_theta_;
            }
            break;
        }
        case Phase::TURN: {
            double dtheta = pose_theta_ - start_theta_;
            if (dtheta > M_PI) dtheta -= 2 * M_PI;
            if (dtheta < -M_PI) dtheta += 2 * M_PI;
            if (dtheta > TURN_ANGLE) {
                cmd.example_a = TURN_SPEED;
                cmd.example_b = -TURN_SPEED;
            } else {
                cmd.example_a = 0.0;
                cmd.example_b = 0.0;
                phase_ = Phase::DONE;
            }
            break;
        }
        case Phase::DONE: {
            cmd.example_a = 0.0;
            cmd.example_b = 0.0;
            break;
        }
    }
    pub_cmd_->publish(cmd);
}
