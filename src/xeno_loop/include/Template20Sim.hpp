#ifndef TEMPLATE20SIM_HPP
#define TEMPLATE20SIM_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

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

class Template20Sim : public XenoFrt20Sim
{
public:
    Template20Sim(uint write_decimator_freq, uint monitor_freq);
    ~Template20Sim();
private:
    XenoFileHandler file;
    struct ThisIsAStruct data_to_be_logged;
    LoopController controller;

    // --- Physical and encoder constants ---
    static constexpr double PI = 3.141592653589793;
    static constexpr double WHEEL_DIAMETER = 0.101;      
    static constexpr double GEAR_RATIO = 15.58;
    static constexpr double ENCODER_COUNTS_PER_MOTOR_REV = 4096.0; 
    static constexpr double COUNTS_PER_WHEEL_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO;
    static constexpr double WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
    static constexpr double DIST_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;
    static constexpr uint16_t ENCODER_MAX_COUNT = 16383; // 14-bit max value 
    static constexpr uint16_t ENCODER_RANGE = 16384;    // 2^14
    static constexpr int16_t ENCODER_WRAP_THRESHOLD = ENCODER_RANGE / 2;

    double u[4];
    double y[2];

    // --- Feedback calculation state ---
    double total_pos_left;  // Accumulated distance for left wheel
    double total_pos_right; // Accumulated distance for right wheel
    uint16_t prev_encoder_left_raw;
    uint16_t prev_encoder_right_raw;
    bool feedback_initialized;

    // --- Helper functions ---
    int16_t calculate_delta_counts_left(uint16_t current_raw, uint16_t previous_raw);
    int16_t calculate_delta_counts_right(uint16_t current_raw, uint16_t previous_raw);
    void updateWheelPositions(uint16_t current_encoder_left_raw, uint16_t current_encoder_right_raw);

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

#endif // TEMPLATE20SIM_HPP