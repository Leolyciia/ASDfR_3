#ifndef XENOLOOPRUNNER_HPP
#define XENOLOOPRUNNER_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"
#include <cmath> 

#pragma pack (1)    //https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct ThisIsAStruct
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
};

#pragma pack(0)

class XenoLoopRunner : public XenoFrt20Sim
{
public:
    XenoLoopRunner(uint write_decimator_freq, uint monitor_freq);
    ~XenoLoopRunner();
private:
    XenoFileHandler file;
    struct ThisIsAStruct data_to_be_logged;
    LoopController controller;

    #ifndef M_PI
    static constexpr double M_PI = 3.14159265358979323846;
    #endif
    static constexpr double WHEEL_DIAMETER = 0.101; 
    static constexpr double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0; 
    static constexpr double GEAR_RATIO = 15.58;
    static constexpr double ENCODER_COUNTS_PER_MOTOR_REV = 4096.0;
    static constexpr double COUNTS_PER_WHEEL_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO;
    static constexpr double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
    static constexpr double DIST_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;
    static constexpr uint16_t ENCODER_MAX_COUNT = 16383; // 14-bit max value
    static constexpr uint16_t ENCODER_RANGE = 16384;    // 2^14
    static constexpr int16_t ENCODER_WRAP_THRESHOLD = ENCODER_RANGE / 2;
    // Add Wheel Base
    static constexpr double WHEEL_BASE_WIDTH = 0.21;

    double u[4]; // Controller inputs
    double y[2]; // Controller outputs 

    // --- Feedback calculation state ---
    double total_pos_left_m_ = 0.0;  
    double total_pos_right_m_ = 0.0; 
    uint16_t prev_encoder_left_raw_ = 0;
    uint16_t prev_encoder_right_raw_ = 0;
    bool feedback_initialized_ = false;

    // --- Odometry state ---
    double x_pos_ = 0.0;  
    double y_pos_ = 0.0;   
    double theta_ = 0.0; 

    // --- Helper functions ---
    int16_t calculate_delta_counts_left(uint16_t current_raw, uint16_t previous_raw);
    int16_t calculate_delta_counts_right(uint16_t current_raw, uint16_t previous_raw);
    void updateOdometryAndWheelPositions(uint16_t current_encoder_left_raw, uint16_t current_encoder_right_raw);
    double normalize_angle(double angle); 

protected:
    //Functions
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;

    // current error
    int current_error = 0;
};

#endif // XENOLOOPRUNNER_HPP