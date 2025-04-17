#include "XenoLoopRunner.hpp"
#include <cmath>
#include <cstring>

XenoLoopRunner::XenoLoopRunner(uint write_decimator_freq, uint monitor_freq)
: XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
  file(1, "./xrf2_logging/TEMPLATE", "bin"),
  controller()
{
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    controller.SetFinishTime(0.0);
}

XenoLoopRunner::~XenoLoopRunner() {}

int XenoLoopRunner::initialising()
{
    total_pos_left = 0.0;
    total_pos_right = 0.0;
    prev_encoder_left_raw = 0;
    prev_encoder_right_raw = 0;
    feedback_initialized = false;
    controller.Reset(0.0);
    logger.initialise();
    ico_io.init();
    std::memset(&actuate_data, 0, sizeof(actuate_data));
    ico_io.update_io(actuate_data, &sample_data);
    prev_encoder_left_raw = sample_data.channel1;
    prev_encoder_right_raw = sample_data.channel2;
    feedback_initialized = true;
    return 1;
}

int XenoLoopRunner::initialised()
{
    return 1;
}

int XenoLoopRunner::run()
{
    logger.start();
    monitor.printf("Run\n");

    // Read and log
    data_to_be_logged.this_is_a_int++;

    ico_io.update_io(actuate_data, &sample_data);
    uint16_t raw_left = sample_data.channel2;
    uint16_t raw_right = sample_data.channel1;

    // Compute encoder deltas with wrap-around
    int32_t diff_left = static_cast<int32_t>(raw_left) - static_cast<int32_t>(prev_encoder_left_raw);
    if (diff_left > ENCODER_WRAP_THRESHOLD) diff_left -= ENCODER_RANGE;
    else if (diff_left < -ENCODER_WRAP_THRESHOLD) diff_left += ENCODER_RANGE;

    int32_t diff_right = static_cast<int32_t>(raw_right) - static_cast<int32_t>(prev_encoder_right_raw);
    if (diff_right > ENCODER_WRAP_THRESHOLD) diff_right -= ENCODER_RANGE;
    else if (diff_right < -ENCODER_WRAP_THRESHOLD) diff_right += ENCODER_RANGE;

    prev_encoder_left_raw = raw_left;
    prev_encoder_right_raw = raw_right;

    // Odometry computation
    double left_distance = (diff_left / ((2048.0 * GEAR_RATIO) * 4)) * (2 * M_PI * WHEEL_RADIUS);
    double right_distance = (diff_right / ((2048.0 * GEAR_RATIO) * 4)) * (2 * M_PI * WHEEL_RADIUS);
    double delta_d = (right_distance + left_distance) / 2.0;
    double delta_theta = (right_distance - left_distance) / WHEEL_BASE;

    odom_data.theta += delta_theta;
    odom_data.x += delta_d * std::cos(odom_data.theta);
    odom_data.y += delta_d * std::sin(odom_data.theta);

    // Update total motor positions for controller
    total_pos_left += (diff_left / (1024.0 * GEAR_RATIO * 4)) * (2 * M_PI);
    total_pos_right += (diff_right / (1024.0 * GEAR_RATIO * 4)) * (2 * M_PI);

    // Prepare inputs for real-time controller
    u[0] = total_pos_left;
    u[1] = total_pos_right;
    u[2] = ros_msg.example_a;
    u[3] = ros_msg.example_b;
    controller.Calculate(u, y);

    // PWM output
    const int16_t max_pwm = 2047;
    int16_t pwm_left  = static_cast<int16_t>(std::clamp(y[0],  -max_pwm, max_pwm));
    int16_t pwm_right = static_cast<int16_t>(std::clamp(y[1],  -max_pwm, max_pwm));

    actuate_data.pwm1 = -pwm_right;  // Right motor
    actuate_data.pwm2 =  pwm_left;   // Left motor

    // Publish back to ROS
    xeno_msg.encoder_left  = total_pos_left;
    xeno_msg.encoder_right = total_pos_right;
    xeno_msg.x             = odom_data.x;
    xeno_msg.y             = odom_data.y;
    xeno_msg.theta         = odom_data.theta;

    monitor.printf("Pos: %.2f, %.2f  Theta: %.2f\n", odom_data.x, odom_data.y, odom_data.theta);

    return controller.IsFinished() ? 1 : 0;
}

int XenoLoopRunner::stopping()
{
    logger.stop();
    return 1;
}

int XenoLoopRunner::stopped()
{
    return 0;
}

int XenoLoopRunner::pausing()
{
    return 1;
}

int XenoLoopRunner::paused()
{
    return 0;
}

int XenoLoopRunner::error()
{
    return 0;
}
