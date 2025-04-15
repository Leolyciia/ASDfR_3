#include "Template20Sim.hpp"

Template20Sim::Template20Sim(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"./xrf2_logging/TEMPLATE","bin"), // change template to your project name
    controller()
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    // Add variables to logger to be logged, has to be done before you can log data
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    
    // To infinite run the controller, uncomment line below
    controller.SetFinishTime(0.0);
}

Template20Sim::~Template20Sim()
{
    
}

// Helper function: calculate delta counts
int16_t Template20Sim::calculate_delta_counts(uint16_t current_raw, uint16_t previous_raw) {
    int32_t diff = static_cast<int32_t>(current_raw) - static_cast<int32_t>(previous_raw);

    if (diff > ENCODER_WRAP_THRESHOLD) {
        return diff - ENCODER_RANGE; // Wrapped backward
    } else if (diff < -ENCODER_WRAP_THRESHOLD) {
        return diff + ENCODER_RANGE; // Wrapped forward
    } else {
        return diff; // No wrap-around
    }
}

// Helper function: for updating the wheel positions
void Template20Sim::updateWheelPositions(uint16_t current_encoder_left_raw, uint16_t current_encoder_right_raw) {
    if (!feedback_initialized) {
        prev_encoder_left_raw = current_encoder_left_raw;
        prev_encoder_right_raw = current_encoder_right_raw;
        feedback_initialized = true;
        // Don't calculate displacement on the very first run
        return;
    }

    // Calculate delta counts
    int16_t delta_counts_left = calculate_delta_counts(current_encoder_left_raw, prev_encoder_left_raw);
    int16_t delta_counts_right = calculate_delta_counts(current_encoder_right_raw, prev_encoder_right_raw);

    // Calculate step displacement 
    // Left wheel count increases backward 
    double displacement_step_left = static_cast<double>(delta_counts_left) * DIST_PER_COUNT;
    // Right wheel count decreases backward 
    double displacement_step_right = -static_cast<double>(delta_counts_right) * DIST_PER_COUNT;

    // Accumulate total position
    total_pos_left += displacement_step_left;
    total_pos_right += displacement_step_right;

    // Store current raw counts for the next iteration
    prev_encoder_left_raw = current_encoder_left_raw;
    prev_encoder_right_raw = current_encoder_right_raw;
}

int Template20Sim::initialising()
{
    // Set physical and cyber system up for use in a 
    // Return 1 to go to initialised state

    evl_printf("Initialising...\n");      // Do something

    // Reset position feedback states
    total_pos_left = 0.0;
    total_pos_right = 0.0;
    prev_encoder_left_raw = 0; 
    prev_encoder_right_raw = 0;
    feedback_initialized = false;
    controller.Reset(0.0);
    // The logger has to be initialised at only once
    logger.initialise();
    // The FPGA has to be initialised at least once
    ico_io.init();

    // Reset actuation data
    memset(&actuate_data, 0, sizeof(actuate_data));
    actuate_data.pwm1 = 0; // Left
    actuate_data.pwm2 = 0; // Right
    actuate_data.val1 = false;
    actuate_data.val2 = false;


    // Dummy read for initialising prev_encoder values
    ico_io.update_io(actuate_data, &sample_data);
    prev_encoder_left_raw = sample_data.channel1;
    prev_encoder_right_raw = sample_data.channel2;
    feedback_initialized = true; // Mark as initialized after first read
    evl_printf("Initialising complete.\n");

    return 1;
}

int Template20Sim::initialised()
{
    // Keep the physical syste in a state to be used in the run state
    // Call start() or return 1 to go to run state

    evl_printf("Hello from initialised\n");       // Do something

    return 1;
}

int Template20Sim::run()
{
    // Do what you need to do
    // Return 1 to go to stopping state

    // Start logger
    logger.start();                             
    monitor.printf("Hello from run\n");  
    //  Change some data for logger            
    data_to_be_logged.this_is_a_bool = !data_to_be_logged.this_is_a_bool;
    data_to_be_logged.this_is_a_int++;
    if(data_to_be_logged.this_is_a_char == 'R')
        data_to_be_logged.this_is_a_char = 'A';
    else if (data_to_be_logged.this_is_a_char == 'A')
        data_to_be_logged.this_is_a_char = 'M';
    else
        data_to_be_logged.this_is_a_char = 'R';
    data_to_be_logged.this_is_a_float = data_to_be_logged.this_is_a_float/2;
    data_to_be_logged.this_is_a_double = data_to_be_logged.this_is_a_double/4; 

    ico_io.update_io(actuate_data, &sample_data);

    // Get raw encoder counts
    uint16_t raw_left_encoder = sample_data.channel1;  // Left wheel 
    uint16_t raw_right_encoder = sample_data.channel2; // Right wheel 

    // Update accumulated wheel positions
    updateWheelPositions(raw_left_encoder, raw_right_encoder);

    u[0] = total_pos_left;  // PosLeft feedback
    u[1] = total_pos_right; // PosRight feedback
    // Get velocity setpoints from ROS message
    u[2] = ros_msg.example_a; // Placeholder
    u[3] = ros_msg.example_b; // Placeholder

    controller.Calculate(u, y); // y[0]=SteerLeft, y[1]=SteerRight

    const int16_t max_abs_pwm = 2047;
    int16_t pwm_left_cmd = static_cast<int16_t>(std::clamp(y[0], (double)-max_abs_pwm, (double)max_abs_pwm));
    int16_t pwm_right_cmd = static_cast<int16_t>(std::clamp(y[1], (double)-max_abs_pwm, (double)max_abs_pwm));

    // Map controller outputs to correct PWM channels
    // ASKK TA ABOUT THISSSS WHAT DOES THE ROBOT SEE AS LEFT/RIGHT??? 
    actuate_data.pwm1 = pwm_right_cmd; // PMOD P1 -> Right Motor
    actuate_data.pwm2 = pwm_left_cmd;  // PMOD P2 -> Left Motor
    actuate_data.val1 = (pwm_right_cmd >= 0);
    actuate_data.val2 = (pwm_left_cmd >= 0);

    xeno_msg.encoder_left = total_pos_left;
    xeno_msg.encoder_right = total_pos_right;

    // Out
    monitor.printf("Run - PosL:%.4f PosR:%.4f | SetVelL:%.2f SetVelR:%.2f | SteerL:%.1f SteerR:%.1f\n",
                   total_pos_left, total_pos_right, u[2], u[3], y[0], y[1]);
    
    if(controller.IsFinished())
        return 1;


    return 0;
}

int Template20Sim::stopping()
{
    // Bring the physical system to a stop and set it in a state that the system can be deactivated
    // Return 1 to go to stopped state
    logger.stop();                                // Stop logger
    evl_printf("Hello from stopping\n");          // Do something

    return 1;
}

int Template20Sim::stopped()
{
    // A steady state in which the system can be deactivated whitout harming the physical system

    monitor.printf("Hello from stopping\n");          // Do something

    return 0;
}

int Template20Sim::pausing()
{
    // Bring the physical system to a stop as fast as possible without causing harm to the physical system

    evl_printf("Hello from pausing\n");           // Do something
    return 1 ;
}

int Template20Sim::paused()
{
    // Keep the physical system in the current physical state

    monitor.printf("Hello from paused\n");            // Do something
    return 0;
}

int Template20Sim::error()
{
    // Error detected in the system 
    // Can go to error if the previous state returns 1 from every other state function but initialising 

    monitor.printf("Hello from error\n");             // Do something

    return 0;
}
