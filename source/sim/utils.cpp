#include <Eigen/Dense>
#include <utility>
#include <cmath>
#include <numbers>
#include <string_view>
#include <raylib.h>

#include "robot_arm/sim/utils.hpp"

namespace robot_arm::sim::utils
{
    Transform toTransform(const Eigen::Matrix4d &matrix,
                          const Vector3 &scale,
                          double symmetry_treshold,
                          double identity_treshold)
    {
        // Rotate the matrix by -90 degrees around the X-axis to match Raylib's Y-up
        Eigen::Matrix4d rotation_4x4 = Eigen::Matrix4d::Identity();
        rotation_4x4.block<3, 3>(0, 0) = Eigen::AngleAxisd(-std::numbers::pi / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix4d adjusted_matrix = rotation_4x4 * matrix; // no clue how this works, but it does

        Transform transform;

        // Extract position
        transform.position = {static_cast<float>(adjusted_matrix(0, 3)),
                              static_cast<float>(adjusted_matrix(1, 3)),
                              static_cast<float>(adjusted_matrix(2, 3))};

        // Extract rotation
        Eigen::Matrix3d rot = adjusted_matrix.block<3, 3>(0, 0);
        if (rot.isIdentity(identity_treshold))
        {
            transform.rotation_angle = 0.0f;
            transform.rotation_axis = {0.0f, 0.0f, 1.0f};
            return transform;
        }

        Eigen::AngleAxisd aa(rot);
        float angle_deg = static_cast<float>(aa.angle() * (180.0 / std::numbers::pi));
        Eigen::Vector3d axis = aa.axis().normalized();

        transform.rotation_angle = angle_deg;
        transform.rotation_axis = {
            static_cast<float>(axis.x()),
            static_cast<float>(axis.y()),
            static_cast<float>(axis.z())};

        return transform;
    }

    FusionModel loadModel(std::string_view file_path, const Transform &local_transform)
    {
        FusionModel fusion_model;
        fusion_model.model = LoadModel(file_path.data());
        fusion_model.local_transform = local_transform;
        fusion_model.global_transform = {0.0f, 0.0f, 0.0f};
        fusion_model.scale = {1.0f, 1.0f, 1.0f};
        return fusion_model;
    }
}
