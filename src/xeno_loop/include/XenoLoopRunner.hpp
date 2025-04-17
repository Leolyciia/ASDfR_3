#ifndef XENOLOOPRUNNER_HPP
#define XENOLOOPRUNNER_HPP

#include "XenoFrt20Sim.hpp" 


class XenoLoopRunner : public XenoFrt20Sim
{
public:
    XenoLoopRunner(uint write_decimator_freq, uint monitor_freq);
    ~XenoLoopRunner();

private:
    // --- Encoder reading ---
    uint16_t prev_encoder_left_raw; 
    uint16_t prev_encoder_right_raw; 
    bool initialized;             

    // PWM test value 
    int16_t test_pwm_left = 0;
    int16_t test_pwm_right = 0;


protected:
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;

    int current_error = 0;
};

#endif // XENOLOOPRUNNER_HPP