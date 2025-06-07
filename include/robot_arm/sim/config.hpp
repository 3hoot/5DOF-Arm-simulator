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
    constexpr Vector3 CAMERA_POSITION = {30.0f, 30.0f, 30.0f}; // Initial camera position
    constexpr Vector3 CAMERA_TARGET = {0.0f, 30.0f, 0.0f};     // Initial camera target position
    constexpr Vector3 CAMERA_UP = {0.0f, 1.0f, 0.0f};          // Up direction for the camera
    constexpr int CAMERA_PROJECTION = CAMERA_PERSPECTIVE;      // Camera projection type

    // Robot to model pairing (Joint index to model path)
    const std::vector<std::string_view> ROBOT_MODELS = {
        "resources/J0.obj", // Base model
        "resources/J1.obj", // Joint 1 model
        "resources/J2.obj", // Joint 2 model
        "resources/J3.obj", // Joint 3 model
        "resources/J4.obj", // Joint 4 model
        "",                 // Joint 5 has no model (is a part of joint 4)
        "resources/J5.obj", // End effector model
        ""                  // End effector offset model (not used, but can be added later)
    };

    const std::vector<utils::Transform> ROBOT_MODELS_LOCAL_TRANSFORMS = {
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f},   // Base model
        {{0.0f, 0.0f, -12.5f}, {1.0f, 0.0f, 0.0f}, 90.0f}, // Joint 1 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f},   // Joint 2 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f},   // Joint 3 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f},   // Joint 4 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f},   // Joint 5 model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f},   // End effector model
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f}    // End effector offset model
    };

    // Primitive models
    constexpr std::string_view PRIMITIVE_CUBE_MODEL = "resources/primitives/cube.obj";
    constexpr std::string_view PRIMITIVE_SPHERE_MODEL = "resources/primitives/sphere.obj";

}

#endif
