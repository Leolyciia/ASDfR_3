
#ifndef XENOLOOPRUNNER_HPP_
#define XENOLOOPRUNNER_HPP_

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>

#pragma pack(1)
struct ThisIsAStruct {
    int    this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float  this_is_a_float = 10.0;
    char   this_is_a_char = 'R';
    bool   this_is_a_bool = false;
};

class XenoLoopRunner : public XenoFrt20Sim {
public:
    XenoLoopRunner(uint write_decimator_freq, uint monitor_freq);
    ~XenoLoopRunner();

private:
    XenoFileHandler file;
    ThisIsAStruct   data_to_be_logged;
    LoopController  controller;

    // --- Odometry of our relbottie---
    struct Odometry {
        double x     = 0.0;
        double y     = 0.0;
        double theta = 0.0;
    } odom_data;

    // --- Parameters ---
    static constexpr double WHEEL_DIAMETER = 0.101;              
    static constexpr double WHEEL_RADIUS = WHEEL_DIAMETER * 0.5;   
    static constexpr double WHEEL_BASE = 0.20;                  
    static constexpr double GEAR_RATIO = 15.58;
    static constexpr double ENCODER_COUNTS_PER_MOTOR_REV = 4096.0;
    static constexpr double COUNTS_PER_WHEEL_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO;
    static constexpr double WHEEL_CIRCUMFERENCE = 2.0 * M_PI * WHEEL_RADIUS;
    static constexpr uint16_t ENCODER_RANGE = 16384;              
    static constexpr int16_t  ENCODER_WRAP_THRESHOLD = ENCODER_RANGE / 2;

    double u[4];
    double y[2];

    // --- Encoder thingies---
    double    total_pos_left = 0.0;
    double    total_pos_right = 0.0;
    uint16_t  prev_encoder_left_raw = 0;
    uint16_t  prev_encoder_right_raw = 0;
    bool      feedback_initialized = false;

    // --- Helpers ---
    int16_t calculate_delta_counts(uint16_t current_raw, uint16_t previous_raw);
    void    updateWheelPositions(uint16_t current_encoder_left_raw,
                                 uint16_t current_encoder_right_raw);

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

#endif // XENOLOOPRUNNER_HPP_
