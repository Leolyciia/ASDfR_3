#ifndef XENOLOOPRUNNER_HPP_
#define XENOLOOPRUNNER_HPP_

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"
#include "xrf2_msgs/msg/xeno2_ros.hpp"
#include <cmath>

#pragma pack(1)
struct ThisIsAStruct {
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
};

class XenoLoopRunner : public XenoFrt20Sim {
public:
    XenoLoopRunner(uint write_decimator_freq, uint monitor_freq);
    ~XenoLoopRunner();
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;
private:
    ThisIsAStruct data_to_be_logged;
    struct Odometry { double x = 0.0; double y = 0.0; double theta = 0.0; } odom_data;
};

#endif // XENOLOOPRUNNER_HPP_
