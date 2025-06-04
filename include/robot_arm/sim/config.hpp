#ifndef ROBOT_ARM_SIM_CONFIG_HPP
#define ROBOT_ARM_SIM_CONFIG_HPP

#include <string_view>

// All angles are in degrees [deg]
// All lengths are in centimeters [cm]

namespace robot_arm::sim::config
{
    // Enable or disable debug mode
    constexpr bool ENABLE_DEBUG = true;

    // Raylib window configuration
    constexpr int SCREEN_WIDTH = 1600;                               // Width of the Raylib window
    constexpr int SCREEN_HEIGHT = 1200;                              // Height of the Raylib window
    constexpr std::string_view WINDOW_TITLE = "Robot Arm Simulator"; // Title of the Raylib window
}

#endif
