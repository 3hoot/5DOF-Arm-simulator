#ifndef ROBOT_ARM_CORE_UTILS_HPP
#define ROBOT_ARM_CORE_UTILS_HPP

#include <Eigen/Dense>

#include "joint.hpp"

// All angles are in radians [rad]
// All lengths are in centimeters [cm]

namespace robot_arm::core::utils
{
    // Checks for collision between two joints using the start and end offsets, and the collision radius
    // Needs to know the position of the joints in the world frame
    bool jointIntersect(const Joint &joint1, const Eigen::Vector3d &pos1,
                        const Joint &joint2, const Eigen::Vector3d &pos2);

    // Calculates the transformation matrix for a joint based on its parameters
    Eigen::Matrix4d calculateTransformationMatrix(const DHParameters &parameters);
}

#endif
