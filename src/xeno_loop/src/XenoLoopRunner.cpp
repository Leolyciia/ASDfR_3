#include "XenoLoopRunner.hpp"
#include <cmath>

XenoLoopRunner::XenoLoopRunner(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"./xrf2_logging/RELBOT","bin"), 
    controller()
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);

    logger.addVariable("x_pos", double_);
    logger.addVariable("y_pos", double_);
    logger.addVariable("theta", double_);

    controller.SetFinishTime(0.0);
}

XenoLoopRunner::~XenoLoopRunner()
{
    // Destructor
}

int16_t XenoLoopRunner::calculate_delta_counts_left(uint16_t current_raw, uint16_t previous_raw) {
    int32_t diff = static_cast<int32_t>(current_raw) - static_cast<int32_t>(previous_raw);

    // Handle wrap-around
    if (diff > ENCODER_WRAP_THRESHOLD) {
        return diff - ENCODER_RANGE; // Wrapped backward
    } else if (diff < -ENCODER_WRAP_THRESHOLD) {
        return diff + ENCODER_RANGE; // Wrapped forward
    } else {
        return diff; // No wrap-around
    }
}

int16_t XenoLoopRunner::calculate_delta_counts_right(uint16_t current_raw, uint16_t previous_raw) {
    int32_t diff = static_cast<int32_t>(current_raw) - static_cast<int32_t>(previous_raw);

    // Handle wrap-around
    if (diff > ENCODER_WRAP_THRESHOLD) {
        return diff - ENCODER_RANGE; // Wrapped backward
    } else if (diff < -ENCODER_WRAP_THRESHOLD) {
        return diff + ENCODER_RANGE; // Wrapped forward
    } else {
        return diff; // No wrap-around
    }
}

// double XenoLoopRunner::normalize_angle(double angle) {
//     angle = fmod(angle + M_PI, 2.0 * M_PI);
//     if (angle < 0.0)
//         angle += 2.0 * M_PI;
//     return angle - M_PI;
// }


void XenoLoopRunner::updateOdometryAndWheelPositions(uint16_t current_encoder_left_raw, uint16_t current_encoder_right_raw) {
    if (!feedback_initialized_) {
        prev_encoder_left_raw_ = current_encoder_left_raw;
        prev_encoder_right_raw_ = current_encoder_right_raw;
        feedback_initialized_ = true;
        return;
    }


    int16_t delta_counts_left = calculate_delta_counts_left(current_encoder_left_raw, prev_encoder_left_raw_);
    int16_t delta_counts_right = calculate_delta_counts_right(-current_encoder_right_raw, -prev_encoder_right_raw_);

    double displacement_step_left = (static_cast<double>(delta_counts_left) / (1024.0 * GEAR_RATIO * 4)) * (2 * M_PI);
    // Right wheel count decreases backward 
    double displacement_step_right = (static_cast<double>(delta_counts_right) / (1024.0 * GEAR_RATIO * 4)) * (2 * M_PI);

    total_pos_left_m_ += displacement_step_left;
    total_pos_right_m_ += displacement_step_right;

    // --- Calculate odometry ---
    double delta_s = (displacement_step_right + displacement_step_left) / 2.0;
    double delta_theta = (displacement_step_right - displacement_step_left) / WHEEL_BASE_WIDTH;

    // Update pose
    theta_ += delta_theta;
    x_pos_ += delta_s * cos(theta_ + delta_theta / 2.0); 
    y_pos_ += delta_s * sin(theta_ + delta_theta / 2.0); 
    
    // theta_ = normalize_angle(theta_);

    prev_encoder_left_raw_ = current_encoder_left_raw;
    prev_encoder_right_raw_ = current_encoder_right_raw;
}

int XenoLoopRunner::initialising()
{
    evl_printf("Initialising...\n");
    // Reset odometry state
    x_pos_ = 0.0;
    y_pos_ = 0.0;
    theta_ = 0.0;
    // Reset feedback state
    total_pos_left_m_ = 0.0;
    total_pos_right_m_ = 0.0;
    prev_encoder_left_raw_ = 0;
    prev_encoder_right_raw_ = 0;
    feedback_initialized_ = false;
    // Reset controller
    controller.Reset(0.0);

    // Initialise logger if not already done
    if(!logger.isInitialised())
        logger.initialise();
    // Initialise FPGA if not already done
    if(ico_io.init()<0) 
        return -1; // Indicate error

    // Reset actuation data
    memset(&actuate_data, 0, sizeof(actuate_data));
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;

    ico_io.update_io(actuate_data, &sample_data);
    prev_encoder_left_raw_ = sample_data.channel1; 
    prev_encoder_right_raw_ = sample_data.channel2; 
    feedback_initialized_ = true; // Mark as initialized after first read

    u[0] = total_pos_left_m_;
    u[1] = total_pos_right_m_;
    u[2] = 0.0; 
    u[3] = 0.0; 

    evl_printf("Initialising complete.\n");
    return 1; // Transition to initialised state
}

int XenoLoopRunner::initialised()
{
    monitor.printf("State: Initialised. Waiting for Start command.\n");
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    ico_io.update_io(actuate_data, &sample_data);
    return 1; 
}

int XenoLoopRunner::run()
{
    // Ensure logger is started
    if(!logger.isStarted())
        logger.start();

    // --- Get Inputs ---
    ico_io.update_io(actuate_data, &sample_data);
    uint16_t raw_left_encoder = sample_data.channel1;
    uint16_t raw_right_encoder = sample_data.channel2;

    // --- Update odometry & wheel positions ---
    updateOdometryAndWheelPositions(raw_left_encoder, raw_right_encoder);

    // --- Prepare controller inputs ---
    u[0] = total_pos_left_m_;  
    u[1] = total_pos_right_m_; 
    u[2] = ros_msg.example_a;  
    u[3] = ros_msg.example_b;  

    // --- Run controller ---
    controller.Calculate(u, y); 

    const int16_t max_abs_pwm = 2047; 
    int16_t pwm_left_cmd = static_cast<int16_t>(std::clamp(y[0], (double)-max_abs_pwm, (double)max_abs_pwm));
    int16_t pwm_right_cmd = static_cast<int16_t>(std::clamp(y[1], (double)-max_abs_pwm, (double)max_abs_pwm));

    actuate_data.pwm1 = -pwm_right_cmd; 
    actuate_data.pwm2 = pwm_left_cmd;   

    xeno_msg.encoder_left = total_pos_left_m_;  // Send accumulated distance
    xeno_msg.encoder_right = total_pos_right_m_; // Send accumulated distance
    xeno_msg.x = x_pos_;                       // Send calculated X position
    xeno_msg.y = y_pos_;                       // Send calculated Y position
    xeno_msg.theta = theta_;                   // Send calculated orientation


    memcpy(&data_to_be_logged.this_is_a_double, &x_pos_, sizeof(double));

    // --- Monitoring output ---
    monitor.printf("Run - Pose[x:%.3f y:%.3f th:%.3f] | Enc[L:%.3f R:%.3f] | Cmd[L:%.1f R:%.1f]\n",
                   x_pos_, y_pos_, theta_,
                   total_pos_left_m_, total_pos_right_m_,
                   y[0], y[1]); 

    if(controller.IsFinished())
        return 1; // Transition to stopping state

    return 0; // Continue running
}

int XenoLoopRunner::stopping()
{
    monitor.printf("State: Stopping...\n");
    // Ensure motors are stopped
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    ico_io.update_io(actuate_data, &sample_data);

    // Stop logger
    if(logger.isStarted())
        logger.stop();
    return 1; // Transition to stopped state
}

int XenoLoopRunner::stopped()
{
    monitor.printf("State: Stopped. Waiting for Reset or Quit command.\n");
    // Keep motors stopped
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    ico_io.update_io(actuate_data, &sample_data);
    return 0; // Stay in stopped state
}

int XenoLoopRunner::pausing()
{
     monitor.printf("State: Pausing...\n");
    // Ensure motors are stopped quickly
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    ico_io.update_io(actuate_data, &sample_data);

    // Stop logger if running
    if(logger.isStarted())
        logger.stop();
    return 1; // Transition to paused state
}

int XenoLoopRunner::paused()
{
    monitor.printf("State: Paused. Waiting for Start or Stop command.\n");
    // Keep motors stopped
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    ico_io.update_io(actuate_data, &sample_data);
    return 0; // Stay in paused state
}

int XenoLoopRunner::error()
{
    monitor.printf("State: Error! Check logs. Waiting for Reset or Quit command.\n");
    // Ensure motors are stopped
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    ico_io.update_io(actuate_data, &sample_data);

     // Stop logger if running
    if(logger.isStarted())
        logger.stop();
    return 0; // Stay in error state
}