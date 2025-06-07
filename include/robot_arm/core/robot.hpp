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

        // Setters
        bool addEffector(const Joint &joint);
        bool removeEffector();
        bool setJointSetting(size_t joint_index, double setting);

        // IK methods
        // This one is hardcoded for THIS particular robot arm, but can be generalized later
        Eigen::Matrix<double, 6, 3> computeJacobian3() const;
        Eigen::Vector3d computeJointChanges3(const Eigen::Vector3d &pos_err, double lambda) const;
        // Computes the twist vector for the robot arm to reach a target transform
        Eigen::Vector<double, 6> computePositionTwist(const Eigen::Matrix4d &target_transform) const;

        // Solves the inverse kinematics for the robot arm to reach a target pose
        bool solveIK(const Eigen::Matrix4d &target_pose,
                     int max_iters, double tolerance);

    private:
        std::vector<Joint> joints_ = {};                            // List of joints in the robot arm
        std::vector<Eigen::Matrix4d> transformation_matrices_ = {}; // Global transformation matrices
                                                                    // for each joint
        std::vector<double> joint_settings_ = {};                   // Current settings for each joint

        bool effector_attached_ = false; // Flag if an end effector is attached

        // Helper function to update the transformation matrices after setting a joint's setting
        // Updates the transformation matrices for all joints from the specified joint index
        void updateTransformationMatrices(size_t joint_index);
    };
}

#endif
