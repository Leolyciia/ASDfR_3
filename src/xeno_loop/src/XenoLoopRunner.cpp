
#include "XenoLoopRunner.hpp"
#include <cmath>
#include <cstring>

XenoLoopRunner::XenoLoopRunner(uint write_decimator_freq, uint monitor_freq)
: XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
  file(1, "./xrf2_logging/TEMPLATE", "bin"),
  controller()
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    controller.SetFinishTime(0.0);
}

XenoLoopRunner::~XenoLoopRunner()
{
}

// Helper function: calculate delta counts
int16_t XenoLoopRunner::calculate_delta_counts(uint16_t current_raw, uint16_t previous_raw) {
    int32_t diff = static_cast<int32_t>(current_raw) - static_cast<int32_t>(previous_raw);
    if (diff > ENCODER_WRAP_THRESHOLD) {
        return diff - ENCODER_RANGE;
    } else if (diff < -ENCODER_WRAP_THRESHOLD) {
        return diff + ENCODER_RANGE;
    } else {
        return diff;
    }
}

void XenoLoopRunner::updateWheelPositions(uint16_t current_encoder_left_raw, uint16_t current_encoder_right_raw) {
    if (!feedback_initialized) {
        prev_encoder_left_raw = current_encoder_left_raw;
        prev_encoder_right_raw = current_encoder_right_raw;
        feedback_initialized = true;
        return;
    }

    // Calculate delta counts
    int16_t delta_counts_left = calculate_delta_counts(current_encoder_left_raw, prev_encoder_left_raw);
    int16_t delta_counts_right = calculate_delta_counts(-current_encoder_right_raw, -prev_encoder_right_raw);

    // odometry calculation
    double left_distance  = (static_cast<double>(delta_counts_left)  / ((2048.0 * GEAR_RATIO) * 4)) * (2 * M_PI * WHEEL_RADIUS);
    double right_distance = (static_cast<double>(delta_counts_right) / ((2048.0 * GEAR_RATIO) * 4)) * (2 * M_PI * WHEEL_RADIUS);
    double delta_d = (right_distance + left_distance) / 2.0;
    double delta_theta = (right_distance - left_distance) / WHEEL_BASE;

    odom_data.theta += delta_theta;
    odom_data.x += delta_d * std::cos(odom_data.theta);
    odom_data.y += delta_d * std::sin(odom_data.theta);

    double displacement_step_left = (static_cast<double>(delta_counts_left)  / (1024.0 * GEAR_RATIO * 4)) * (2 * M_PI);
    double displacement_step_right = (static_cast<double>(delta_counts_right) / (1024.0 * GEAR_RATIO * 4)) * (2 * M_PI);

    total_pos_left += displacement_step_left;
    total_pos_right += displacement_step_right;

    prev_encoder_left_raw = current_encoder_left_raw;
    prev_encoder_right_raw = current_encoder_right_raw;
}

int XenoLoopRunner::initialising()
{
    evl_printf("Initialising...\n");
    total_pos_left = 0.0;
    total_pos_right = 0.0;
    prev_encoder_left_raw = 0;
    prev_encoder_right_raw = 0;
    feedback_initialized = false;
    controller.Reset(0.0);
    logger.initialise();
    ico_io.init();
    memset(&actuate_data, 0, sizeof(actuate_data));
    ico_io.update_io(actuate_data, &sample_data);
    prev_encoder_left_raw = sample_data.channel1;
    prev_encoder_right_raw = sample_data.channel2;
    feedback_initialized = true;
    evl_printf("Initialising complete.\n");
    return 1;
}

int XenoLoopRunner::initialised()
{
    evl_printf("Hello from initialised\n");
    return 1;
}

int XenoLoopRunner::run()
{
    logger.start();
    monitor.printf("Hello from run\n");

    // Toggle logger variables
    data_to_be_logged.this_is_a_bool = !data_to_be_logged.this_is_a_bool;
    data_to_be_logged.this_is_a_int++;
    if (data_to_be_logged.this_is_a_char == 'R')
        data_to_be_logged.this_is_a_char = 'A';
    else if (data_to_be_logged.this_is_a_char == 'A')
        data_to_be_logged.this_is_a_char = 'M';
    else
        data_to_be_logged.this_is_a_char = 'R';
    data_to_be_logged.this_is_a_float  /= 2.0f;
    data_to_be_logged.this_is_a_double /= 4.0;

    ico_io.update_io(actuate_data, &sample_data);

    // Get raw encoder counts
    uint16_t raw_left_encoder  = sample_data.channel2;
    uint16_t raw_right_encoder = sample_data.channel1;

    // Update positions and odometry
    updateWheelPositions(raw_left_encoder, raw_right_encoder);

    // Prepare controller inputs
    u[0] = total_pos_left;
    u[1] = total_pos_right;
    u[2] = ros_msg.example_a;
    u[3] = ros_msg.example_b;

    controller.Calculate(u, y);

    const int16_t max_abs_pwm = 2047;
    int16_t pwm_left_cmd  = static_cast<int16_t>(std::clamp(y[0], (double)-max_abs_pwm, (double)max_abs_pwm));
    int16_t pwm_right_cmd = static_cast<int16_t>(std::clamp(y[1], (double)-max_abs_pwm, (double)max_abs_pwm));


    // Actuate motors
    actuate_data.pwm1 = -pwm_right_cmd;
    actuate_data.pwm2 =  pwm_left_cmd;

    // Publish feedback to ROS
    xeno_msg.encoder_left  = total_pos_left;
    xeno_msg.encoder_right = total_pos_right;
    xeno_msg.x = odom_data.x;
    xeno_msg.y = odom_data.y;
    xeno_msg.theta = odom_data.theta;

    monitor.printf(
        "Run - PosL:%.4f PosR:%.4f | SetVelL:%.2f SetVelR:%.2f | SteerL:%.1f SteerR:%.1f\n",
        total_pos_left, total_pos_right, u[2], u[3], y[0], y[1]
    );

    if (controller.IsFinished())
        return 1;
    return 0;
}

int XenoLoopRunner::stopping()
{
    logger.stop();
    evl_printf("Hello from stopping\n");
    return 1;
}

int XenoLoopRunner::stopped()
{
    monitor.printf("Hello from stopped\n");
    return 0;
}

int XenoLoopRunner::pausing()
{
    evl_printf("Hello from pausing\n");
    return 1;
}

int XenoLoopRunner::paused()
{
    monitor.printf("Hello from paused\n");
    return 0;
}

int XenoLoopRunner::error()
{
    monitor.printf("Hello from error\n");
    return 0;
}
