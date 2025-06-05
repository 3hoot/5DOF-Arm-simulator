#ifndef ROBOT_ARM_SIM_UTILS_HPP
#define ROBOT_ARM_SIM_UTILS_HPP

#include <Eigen/Dense>
#include <raylib.h>
#include <string_view>

// All angles are in degrees [deg]
// All lengths are in centimeters [cm]

namespace robot_arm::sim::utils
{
    struct Transform
    {
        Vector3 position = {0.0f, 0.0f, 0.0f};      // Position of the object in the world
        Vector3 rotation_axis = {0.0f, 0.0f, 1.0f}; // Rotation of the object in the world
        float rotation_angle = 0.0f;                // Angle of rotation around the axis in degrees

        // Overload the multiplication operator to apply the transformation to a Vector3
        Transform operator*(const Transform &other) const;
    };

    struct FusionModel
    {
        Model model;                        // Raylib model representing the object
        Transform local_transform;          // Local transformation of the object in its own space
        Transform global_transform;         // Transformation of the object in the world
        Vector3 scale = {1.0f, 1.0f, 1.0f}; // Scale of the object in the world
    };

    // Converts an Eigen matrix to a Raylib Transform
    Transform toTransform(const Eigen::Matrix4d &matrix,
                          double symmetry_treshold = 0.001,
                          double identity_treshold = 0.001);

    // Loads a model from a file and applies the specified transformation
    FusionModel loadModel(std::string_view file_path, const Transform &local_transform);
}

#endif
