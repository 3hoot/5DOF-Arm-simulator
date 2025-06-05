#include <vector>
#include <numbers>
#include <Eigen/Dense>

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

        joint_offsets_.resize(joints.size());
        updateJointOffsets(0);

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

    void Robot::updateJointOffsets(size_t joint_index)
    {
        // Update the joint offsets for all joints from the specified joint index
        for (size_t i = joint_index; i < joints_.size(); ++i)
        {
            const Joint &joint = joints_[i];
            Eigen::Matrix4d transformation_matrix = transformation_matrices_[i];

            Eigen::Vector3d offset_start = joint.getOffsets().first;
            Eigen::Matrix4d offset_start_matrix = Eigen::Matrix4d::Identity();
            offset_start_matrix.block<3, 1>(0, 3) = offset_start;
            offset_start_matrix = transformation_matrix * offset_start_matrix;

            Eigen::Vector3d offset_end = joint.getOffsets().second;
            Eigen::Matrix4d offset_end_matrix = Eigen::Matrix4d::Identity();
            offset_end_matrix.block<3, 1>(0, 3) = offset_end;
            offset_end_matrix = transformation_matrix * offset_end_matrix;

            joint_offsets_[i] = {offset_start_matrix, offset_end_matrix};
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
        joint_settings_[joint_index] = setting;
        updateTransformationMatrices(joint_index);
        updateJointOffsets(joint_index);

        // Check for collisions
        for (size_t i = 0; i < joints_.size(); ++i)
        {
            if (i == joint_index || i == joint_index - 1 || i == joint_index + 1)
                continue; // Skip the current joint and its immediate neighbors

            Eigen::Vector3d pos1a = joint_offsets_[joint_index].first.block<3, 1>(0, 3);
            Eigen::Vector3d pos1b = joint_offsets_[joint_index].second.block<3, 1>(0, 3);
            Eigen::Vector3d pos2a = joint_offsets_[i].first.block<3, 1>(0, 3);
            Eigen::Vector3d pos2b = joint_offsets_[i].second.block<3, 1>(0, 3);

            if (utils::jointIntersect(pos1a, pos1b, joint.getCollisionRadius(),
                                      pos2a, pos2b, joints_[i].getCollisionRadius()))
            {
                // If a collision is detected, revert the joint's setting
                joint.setSetting(prev_setting);
                joint_settings_[joint_index] = prev_setting;
                updateTransformationMatrices(joint_index);
                updateJointOffsets(joint_index);

                return false; // Collision detected, setting not applied
            }
        }
        return true; // Successfully set the joint setting without collisions
    }

}
