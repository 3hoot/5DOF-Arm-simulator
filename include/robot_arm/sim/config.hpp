#ifndef ROBOT_ARM_SIM_CONFIG_HPP
#define ROBOT_ARM_SIM_CONFIG_HPP

#include <vector>
#include <string_view>
#include <raylib.h>

#include "robot_arm/sim/utils.hpp"
#include "simulator.hpp"

// All angles are in degrees [deg]
// All lengths are in centimeters [cm]

namespace robot_arm::sim::config
{
    // Enable or disable debug mode
    constexpr bool ENABLE_DEBUG = true;  // Enable or disable debug information
    constexpr bool ENABLE_WIRES = false; // Enable or disable wireframe mode for models
    constexpr bool ENABLE_HELP = true;   // Enable or disable help text

    // Raylib window configuration
    constexpr int SCREEN_WIDTH = 1600;                               // Width of the Raylib window
    constexpr int SCREEN_HEIGHT = 1200;                              // Height of the Raylib window
    constexpr std::string_view WINDOW_TITLE = "Robot Arm Simulator"; // Title of the Raylib window
    constexpr int TARGET_FPS = 60;                                   // Target frames per second for the Raylib window

    // Camera configuration
    constexpr float CAMERA_FOVY = 45.0f;                       // Field of view in Y direction for the camera
    constexpr Vector3 CAMERA_POSITION = {50.0f, 50.0f, 50.0f}; // Initial camera position
    constexpr Vector3 CAMERA_TARGET = {0.0f, 30.0f, 0.0f};     // Initial camera target position
    constexpr Vector3 CAMERA_UP = {0.0f, 1.0f, 0.0f};          // Up direction for the camera
    constexpr int CAMERA_PROJECTION = CAMERA_PERSPECTIVE;      // Camera projection type

    // Simulator configuration
    constexpr float SIMULATOR_CAMERA_SPEED = 0.2f;
    constexpr SimulatorMode SIMULATOR_DEFAULT_MODE = SimulatorMode::IndividualJointControl;

    // Robot to model pairing (Joint index to model path)
    const std::vector<std::string_view>
        ROBOT_MODELS = {
            "resources/J0.obj", // Base model
            "resources/J1.obj", // Joint 1 model
            "resources/J2.obj", // Joint 2 model
            "resources/J3.obj", // Joint 3 model
            "resources/J4.obj", // Joint 4 model
            "",                 // Joint 5 has no model (is a part of joint 4)
            "resources/J5.obj"  // End effector model
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

    // Robot related constants
    constexpr double ROBOT_SETTINGS_SMOOTHING_FACTOR = 0.1;              // Smoothing factor for robot settings
    constexpr double ROBOT_SETTINGS_DIFFERENCE_THRESHOLD = 0.01;         // Threshold for robot settings difference
    constexpr double ROBOT_SETTING_INCREMENT = std::numbers::pi / 180.0; // Increment for robot settings (2.5 degrees)
    constexpr double ROBOT_EE_MOVEMENT_INCREMENT = 0.1;                  // Increment for end effector movement (in cm)
    constexpr double ROBOT_EE_APPROACH_OFFSET = 5.5;                     // Approach distance for the end effector (in cm)
    constexpr double ROBOT_EE_APPROACH_HEIGHT = 5.0;                     // Height of the end effector above the ground (in cm)
    constexpr double ROBOT_EE_ATTACH_DISTANCE = 2.0;                     // Distance to attach the end effector (in cm)

    // Primitive models
    constexpr std::string_view PRIMITIVE_SPHERE_MODEL = "resources/primitives/sphere.obj";
    constexpr utils::Transform PRIMITIVE_SPHERE_LOCAL_TRANSFORM =
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f};

    constexpr std::string_view PRIMITIVE_CUBE_MODEL = "resources/primitives/cube.obj";
    constexpr utils::Transform PRIMITIVE_CUBE_LOCAL_TRANSFORM =
        {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 90.0f};

    constexpr int MAX_PRIMITIVES = 2;        // Maximum number of primitives in the simulator
    constexpr double PRIMITIVE_RADIUS = 2.5; // Radius of the sphere primitive (in cm)
}

#endif
