#ifndef ROBOT_ARM_CORE_CONFIG_HPP
#define ROBOT_ARM_CORE_CONFIG_HPP

#include <vector>
#include <numbers>
#include <Eigen/Dense>
#include <functional>
#include <utility>

#include "joint.hpp"

// All angles are in radians [rad]
// All lengths are in centimeters [cm]

namespace robot_arm::core::config
{
    // Joint parameters for the robot arm
    const std::vector<DHParameters> JOINT_PARAMETERS = {
        {0.0, 0.0, 2.0, 0.0},                                      // Joint 0 - Base
        {0.0, 0.0, 12.5, 0.0},                                     // Joint 1
        {6.9645, std::numbers::pi / 2, 0.0, std::numbers::pi / 2}, // Joint 2
        {16.4645, 0.0, 0.0, -std::numbers::pi / 2},                // Joint 3
        {16.4645, 0.0, 0.0, std::numbers::pi / 2},                 // Joint 4
        {0.0, std::numbers::pi / 2, 0.0, 0.0},                     // Joint 5
        {0.0, 0.0, 5.5, 0.0},                                      // Joint 6 - End effector
    };
    const std::vector<JointType> JOINT_TYPES = {
        JointType::Static,   // Joint 0 - Base
        JointType::Revolute, // Joint 1
        JointType::Revolute, // Joint 2
        JointType::Revolute, // Joint 3
        JointType::Revolute, // Joint 4
        JointType::Revolute, // Joint 5
        JointType::Static,   // Joint 6 - End effector
    };
    const std::vector<std::pair<double, double>> JOINT_SETTINGS_RANGE = {
        {0.0, 0.0},                                            // Joint 0 - Base
        {-std::numbers::pi, std::numbers::pi},                 // Joint 1
        {-std::numbers::pi / 2, std::numbers::pi / 2},         // Joint 2
        {-std::numbers::pi / 4, std::numbers::pi},             // Joint 3
        {-3 * std::numbers::pi / 5, 3 * std::numbers::pi / 5}, // Joint 4
        {-std::numbers::pi, std::numbers::pi},                 // Joint 5
        {0.0, 0.0},                                            // Joint 6 - End effector
    };

    // Robot parameters
    const std::vector<Joint> ROBOT_JOINTS = []()
    {
        std::vector<Joint> joints;
        for (size_t i = 0; i < JOINT_PARAMETERS.size(); ++i)
            joints.emplace_back(JOINT_PARAMETERS[i], JOINT_TYPES[i], JOINT_SETTINGS_RANGE[i]);

        return joints;
    }();
}

#endif
