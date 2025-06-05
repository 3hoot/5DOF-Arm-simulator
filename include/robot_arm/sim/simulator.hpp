#ifndef ROBOT_ARM_SIMULATOR_HPP
#define ROBOT_ARM_SIMULATOR_HPP

#include <vector>
#include <string_view>
#include <raylib.h>

#include "../core/robot.hpp"
#include "utils.hpp"

// All angles are in degrees [deg]
// All lengths are in centimeters [cm]

namespace robot_arm::sim
{

    // class Primitive
    // {
    // };

    class Simulator
    {
    public:
        Simulator(core::Robot &robot, bool enable_debug,
                  int window_width, int window_height, const std::string_view window_title,
                  float camera_fovy, Vector3 camera_position, Vector3 camera_target, Vector3 camera_up,
                  int camera_projection);
        Simulator(core::Robot &robot, bool use_config);
        ~Simulator();

        // Runs the main loop of the simulator
        void run();

    private:
        // Robot instance to simulate
        core::Robot &robot_;

        // Flag to enable or disable debug mode
        bool enable_debug_;

        // Raylib parameters
        const int window_width_;
        const int window_height_;
        const std::string_view window_title_;
        Camera3D camera_;

        // Robot to model pairing (Joint index to model index mapping)
        std::vector<utils::FusionModel> robot_models_;
        size_t selected_joint_index_ = 0;

        // Updates the transformation matrices of the robot models based on the robot's current state
        void updateRobotModels();

        // Primitive models for the robot arm
        // 0 for cube primitive
        // 1 for sphere primitive
        // std::vector<FusionModel> primitive_models_;
    };
}

#endif
