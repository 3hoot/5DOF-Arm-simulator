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
        const std::vector<double> &getJointSettings() const { return joint_settings_; };
        double getApproachDistance() const;

        // Setters
        bool addEffector(const Joint &joint);
        bool removeEffector();
        bool setJointSetting(size_t joint_index, double setting);

        // IK methods
        // Solves the inverse kinematics for the robot arm to reach the target transformation matrix
        // Solves analytically for the first 3 joints and rotates the last 2 joints to align the end effector
        // If unable to reach the target, returns settings to be used to reach the closest position
        std::vector<double> solveIK(const Eigen::Vector3d &target_position,
                                    const Eigen::Vector3d &approach_vector,
                                    const double approach_angle);

    private:
        std::vector<Joint> joints_ = {};                            // List of joints in the robot arm
        std::vector<Eigen::Matrix4d> transformation_matrices_ = {}; // Global transformation matrices
                                                                    // for each joint
        std::vector<double> joint_settings_ = {};                   // Current settings for each joint
        double ground_clearance_;                                   // Minimum clearance above the ground plane

        bool effector_attached_ = false; // Flag if an end effector is attached

        // Helper function to update the transformation matrices after setting a joint's setting
        // Updates the transformation matrices for all joints from the specified joint index
        void updateTransformationMatrices(size_t joint_index);
    };
}

#endif
