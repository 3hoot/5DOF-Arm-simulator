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
        // a, alpha, d, theta
        {0.0, 0.0, 30, 0.0},
        {0, std::numbers::pi / 2, 16, 0},
        {20, 0.0, 0.0, 0},
        {15, 0.0, -8.0, 0},
        {6.0, 0.0, 0.0, 0.0}};
    const std::vector<JointType> JOINT_TYPES = {
        JointType::Revolute, // Positional Joint 1      First 3 joints are positional joints
        JointType::Revolute, // Positional Joint 2      last 3 are orientational joints (spherical wrist)
        JointType::Revolute, // Positional Joint 3
        JointType::Revolute, // Orientational Joint 1
        JointType::Static    // Object Joint (End Effector)
    };
    const std::vector<std::pair<double, double>> JOINT_SETTINGS_RANGE = {
        {-2 * std::numbers::pi, 2 * std::numbers::pi},
        {-std::numbers::pi / 4, 5 * std::numbers::pi / 4},
        {-std::numbers::pi / 2, std::numbers::pi / 2},
        {-std::numbers::pi / 2, std::numbers::pi / 2},
        {0.0, 0.0}};

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
