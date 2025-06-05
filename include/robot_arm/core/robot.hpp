#ifndef ROBOT_ARM_CORE_ROBOT_HPP
#define ROBOT_ARM_CORE_ROBOT_HPP

#include <vector>
#include <numbers>
#include <Eigen/Dense>

#include "joint.hpp"

// All angles are in radians [rad]
// All lengths are in centimeters [cm]

namespace robot_arm::core
{
    class Robot
    {
    public:
        Robot(const std::vector<Joint> &joints);

        // Getters
        const std::vector<Eigen::Matrix4d> &getTransformationMatrices() const { return transformation_matrices_; }
        const std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> &getJointOffsets() const { return joint_offsets_; }
        const std::vector<double> &getJointSettings() const { return joint_settings_; };

        // Setters
        bool addEffector(const Joint &joint);
        bool removeEffector();
        bool setJointSetting(size_t joint_index, double setting);

    private:
        std::vector<Joint> joints_ = {}; // List of joints in the robot arm
        std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> joint_offsets_ = {};

        std::vector<Eigen::Matrix4d> transformation_matrices_ = {}; // Global transformation matrices
                                                                    // for each joint
        std::vector<double> joint_settings_ = {};                   // Current settings for each joint

        bool effector_attached_ = false; // Flag if an end effector is attached

        // Helper function to update the transformation matrices after setting a joint's setting
        // Updates the transformation matrices for all joints from the specified joint index
        void updateTransformationMatrices(size_t joint_index);
        void updateJointOffsets(size_t joint_index);
    };
}

#endif
