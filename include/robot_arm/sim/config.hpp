#ifndef ROBOT_ARM_SIM_CONFIG_HPP
#define ROBOT_ARM_SIM_CONFIG_HPP

#include <vector>
#include <string_view>
#include <raylib.h>

#include "robot_arm/sim/utils.hpp"

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

    // Camera configuration
    constexpr float CAMERA_FOVY = 45.0f;                       // Field of view in Y direction for the camera
    constexpr Vector3 CAMERA_POSITION = {50.0f, 50.0f, 50.0f}; // Initial camera position
    constexpr Vector3 CAMERA_TARGET = {0.0f, 30.0f, 0.0f};     // Initial camera target position
    constexpr Vector3 CAMERA_UP = {0.0f, 1.0f, 0.0f};          // Up direction for the camera
    constexpr int CAMERA_PROJECTION = CAMERA_PERSPECTIVE;      // Camera projection type

    // Robot to model pairing (Joint index to model path)
    const std::vector<std::string_view> ROBOT_MODELS = {
        "resources/J1.obj", // Joint 1 model
        "resources/J2.obj", // Joint 2 model
        "resources/J3.obj", // Joint 3 model
        "resources/J4.obj", // Joint 4 model
        ""                  // End effector
    };

    const std::vector<utils::Transform> ROBOT_MODELS_LOCAL_TRANSFORMS = {
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f}, // Joint 1 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f}, // Joint 2 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f}, // Joint 3 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f}, // Joint 4 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f}  // End effector model
    };

    // Primitive object
    constexpr float OBJECT_RADIUS = 2.0f;            // Radius of the primitive object in the simulator
    constexpr float OBJECT_MOUNTING_DISTANCE = 3.0f; // Distance from the end effector to the
                                                     // object mounting point, measured from object center

}

#endif
