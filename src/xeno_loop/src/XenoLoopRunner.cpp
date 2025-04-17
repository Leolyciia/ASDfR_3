
#include "XenoLoopRunner.hpp"
#include <cmath>
#include <string.h>

XenoLoopRunner::XenoLoopRunner(uint write_decimator_freq, uint monitor_freq)
    : XenoFrt20Sim(write_decimator_freq, monitor_freq, file_handle, nullptr), // Pass file_handle to base class
    file_handle(true, "log_file_path", "txt")
{
    evl_printf("%s: XenoLooper constructed\n", __FUNCTION__);
    initialized = false;
}

XenoLoopRunner::~XenoLoopRunner()
{
    evl_printf("%s: Destructing XenoLoopRunner\n", __FUNCTION__);
}

int XenoLoopRunner::initialising()
{
    evl_printf("Initialising...\n");

    prev_encoder_left_raw = 0;
    prev_encoder_right_raw = 0;
    initialized = false;

    if (ico_io.init() != 0) {
        evl_printf("ERROR: Failed to initialize ICO IO!\n");
        return -1;
    }

    memset(&actuate_data, 0, sizeof(actuate_data));
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;

    if (ico_io.update_io(actuate_data, &sample_data) != 0) {
        evl_printf("ERROR: Failed initial ICO IO update\n");
        return -1;
    }

    prev_encoder_right_raw = sample_data.channel1;
    prev_encoder_left_raw = sample_data.channel2;
    initialized = true;

    evl_printf("Initialising complete. Initial Enc R: %u, L: %u\n",
               prev_encoder_right_raw, prev_encoder_left_raw);

    return 1;
}

int XenoLoopRunner::initialised()
{
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    if (ico_io.update_io(actuate_data, &sample_data) != 0) {
        evl_printf("ERROR: Failed initialised ICO IO update\n");
        return -1;
    }
    evl_printf("Initialised state. Motors stopped.\n");
    return 1;
}

int XenoLoopRunner::run()
{
    test_pwm_left = 150;
    test_pwm_right = 150;

    actuate_data.pwm1 = test_pwm_right;
    actuate_data.pwm2 = test_pwm_left;

    if (ico_io.update_io(actuate_data, &sample_data) != 0) {
        evl_printf("ERROR: Failed run() ICO IO update!\n");
        current_error = 1;
        return -1;
    }

    uint16_t current_encoder_right_raw = sample_data.channel1;
    uint16_t current_encoder_left_raw = sample_data.channel2;

    int32_t delta_right = static_cast<int32_t>(current_encoder_right_raw) - static_cast<int32_t>(prev_encoder_right_raw);
    int32_t delta_left = static_cast<int32_t>(current_encoder_left_raw) - static_cast<int32_t>(prev_encoder_left_raw);

    evl_printf("Run - PWM L:%d R:%d | RawEnc L:%u R:%u | Delta L:%d R:%d\n",
               actuate_data.pwm2, actuate_data.pwm1,
               current_encoder_left_raw, current_encoder_right_raw,
               (int)delta_left, (int)delta_right);

    prev_encoder_left_raw = current_encoder_left_raw;
    prev_encoder_right_raw = current_encoder_right_raw;

    return 0;
}

int XenoLoopRunner::stopping()
{
    evl_printf("Stopping...\n");

    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    if (ico_io.update_io(actuate_data, &sample_data) != 0) {
        evl_printf("ERROR: Failed stopping() ICO IO update!\n");
    }

    evl_printf("Stopping complete. Motors stopped.\n");
    return 1;
}

int XenoLoopRunner::stopped()
{
    evl_printf("Stopped state.\n");
    return 0;
}

int XenoLoopRunner::pausing()
{
    evl_printf("Pausing...\n");
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    if (ico_io.update_io(actuate_data, &sample_data) != 0) {
        evl_printf("ERROR: Failed pausing() ICO IO update!\n");
    }
    evl_printf("Pausing complete. Motors stopped.\n");
    return 1;
}

int XenoLoopRunner::paused()
{
    evl_printf("Paused state.\n");
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    if (ico_io.update_io(actuate_data, &sample_data) != 0) {
        evl_printf("ERROR: Failed paused() ICO IO update!\n");
    }
    return 0;
}

int XenoLoopRunner::error()
{
    evl_printf("Error state entered!\n");
    actuate_data.pwm1 = 0;
    actuate_data.pwm2 = 0;
    ico_io.update_io(actuate_data, &sample_data);
    evl_printf("Attempted emergency stop in error state.\n");
    return 0;
}
