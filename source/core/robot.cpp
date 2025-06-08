#include <vector>
#include <numbers>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

#include "robot_arm/core/robot.hpp"
#include "robot_arm/core/joint.hpp"
#include "robot_arm/core/utils.hpp"

namespace robot_arm::core
{
    // Robot class implementation
    Robot::Robot(const std::vector<Joint> &joints)
        : joints_(joints)
    {
        // Initialize the transformation matrices for each joint
        transformation_matrices_.resize(joints.size());
        updateTransformationMatrices(0);

        // Initialize the joint offsets
        joint_settings_.reserve(joints.size());
        for (const auto &joint : joints_)
            joint_settings_.push_back(joint.getSetting());
    }

    void Robot::updateTransformationMatrices(size_t joint_index)
    {
        // Update the transformation matrices for all joints from the specified joint index
        for (size_t i = joint_index; i < joints_.size(); ++i)
        {
            const Joint &joint = joints_[i];
            Eigen::Matrix4d prev_matrix = (i == 0 ? Eigen::Matrix4d::Identity() : transformation_matrices_[i - 1]);
            transformation_matrices_[i] = prev_matrix * joint.getTransformationMatrix();
        }
    }

    bool Robot::addEffector(const Joint &joint)
    {
        if (effector_attached_)
            return false; // Cannot add an effector if one is already attached

        joints_.push_back(joint);
        Eigen::Matrix4d prev_matrix =
            (joints_.size() > 1 ? transformation_matrices_.back() : Eigen::Matrix4d::Identity());
        transformation_matrices_.push_back(prev_matrix * joint.getTransformationMatrix());
        joint_settings_.push_back(joint.getSetting());
        effector_attached_ = true;
        return true; // Successfully added the effector
    }

    bool Robot::removeEffector()
    {
        if (!effector_attached_)
            return false; // Cannot remove an effector if none is attached

        joints_.pop_back();
        transformation_matrices_.pop_back();
        joint_settings_.pop_back();
        effector_attached_ = false;
        return true; // Successfully removed the effector
    }

    bool Robot::setJointSetting(size_t joint_index, double setting)
    {
        if (joint_index < 0 || joint_index >= static_cast<int>(joints_.size()))
            return false; // Invalid joint index

        Joint &joint = joints_[joint_index];
        double prev_setting = joint.getSetting();

        // Set the joint's setting
        if (!joint.setSetting(setting))
            return false; // Setting failed, e.g. for static joints

        // Update the transformation matrices from the specified joint index
        updateTransformationMatrices(joint_index);

        // Check if any of the joint after the specified joint has a collision with the ground plane
        for (size_t i = joint_index + 1; i < joints_.size(); ++i)
        {
            const double MIN_GROUND_CLEARANCE = 5.0; // Minimum clearance above the ground plane
            const Eigen::Matrix4d &matrix = transformation_matrices_[i];
            // Check if the joint's position is below the ground plane (z < 0)
            if (matrix(2, 3) < MIN_GROUND_CLEARANCE)
            {
                // Reset the joint setting to the previous value
                joint.setSetting(prev_setting);
                updateTransformationMatrices(joint_index);
                return false; // Collision detected, revert the setting
            }
        }

        joint_settings_[joint_index] = setting;
        return true; // Successfully set the joint setting without collisions
    }

    // The fixed IK solver: only first 3 joints move, only position is considered
    bool Robot::solveIK(const Eigen::Matrix4d &target_pose)
    {
        return false; // Not implemented yet
    }
}
