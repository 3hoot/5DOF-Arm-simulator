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

    // Compute a 6x3 Jacobian for the first 3 joints only
    Eigen::Matrix<double, 6, 3> Robot::computeJacobian3() const
    {
        using Mat4 = Eigen::Matrix4d;
        using Mat3 = Eigen::Matrix3d;
        using Vec3 = Eigen::Vector3d;

        std::vector<Mat4> matrices = {
            Mat4::Identity(), // Base
            transformation_matrices_[1],
            transformation_matrices_[2],
            transformation_matrices_[3],
            transformation_matrices_.back() // End effector
        };

        Eigen::Matrix<double, 6, 3> jacobian;
        const Vec3 &last_P = matrices.back().block<3, 1>(0, 3);

        for (size_t i = 0; i < 3; ++i)
        {
            const Mat4 &T = matrices[i];
            const Vec3 &P = T.block<3, 1>(0, 3);
            const Mat3 &R = T.block<3, 3>(0, 0);
            Vec3 z_axis = R * Vec3(0, 0, 1); // Joint axis in world frame

            jacobian.block<3, 1>(0, i) = z_axis.cross(last_P - P); // linear
            jacobian.block<3, 1>(3, i) = z_axis;                   // angular
        }

        return jacobian;
    }

    Eigen::Vector<double, 6> Robot::computePositionTwist(const Eigen::Matrix4d &target_transform) const
    {
        // Position error in world frame
        Eigen::Vector3d pos_err = target_transform.block<3, 1>(0, 3) - transformation_matrices_.back().block<3, 1>(0, 3);

        // Set orientation error to zero for position-only IK
        return (Eigen::Vector<double, 6>() << pos_err, Eigen::Vector3d::Zero()).finished();
    }

    // Compute joint changes for the first 3 joints, position-only
    Eigen::Vector3d Robot::computeJointChanges3(const Eigen::Vector3d &pos_err, double lambda) const
    {
        Eigen::Matrix<double, 6, 3> jacobian = computeJacobian3();
        Eigen::Matrix<double, 3, 3> J_pos = jacobian.block<3, 3>(0, 0);

        // Damped least squares pseudo-inverse
        Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d JtJ = J_pos.transpose() * J_pos;
        Eigen::Matrix3d damped = JtJ + lambda * lambda * identity;
        Eigen::Matrix3d J_pinv = damped.inverse() * J_pos.transpose();

        return J_pinv * pos_err;
    }

    // The fixed IK solver: only first 3 joints move, only position is considered
    bool Robot::solveIK(const Eigen::Matrix4d &target_pose, int max_iters, double tolerance)
    {
        for (int iter = 0; iter < max_iters; ++iter)
        {
            Eigen::Matrix4d current_pose = transformation_matrices_.back();
            Eigen::Vector3d pos_err = target_pose.block<3, 1>(0, 3) - current_pose.block<3, 1>(0, 3);
            if (pos_err.norm() < tolerance)
                return true;

            double lambda = (pos_err.norm() < 0.1) ? 1.0 : 0.5;
            Eigen::Vector3d joint_changes = computeJointChanges3(pos_err, lambda);

            double alpha = std::clamp(pos_err.norm(), 0.01, 0.2);
            double max_step = 0.2;

            for (size_t i = 0; i < 3; ++i)
            {
                size_t joint_index = i + 1;
                double joint_min = joints_[joint_index].getSettingRange().first;
                double joint_max = joints_[joint_index].getSettingRange().second;
                double current = joint_settings_[joint_index];
                double center = 0.5 * (joint_min + joint_max);
                double bias = 0.01 * (center - current);
                double delta = std::clamp(alpha * (joint_changes[i] + bias), -max_step, max_step);

                double tentative = current + delta;
                tentative = std::clamp(tentative, joint_min, joint_max);
                delta = tentative - current;

                setJointSetting(joint_index, current + delta);
            }
            updateTransformationMatrices(0);
        }
        // Final check
        Eigen::Matrix4d final_pose = transformation_matrices_.back();
        Eigen::Vector3d final_pos = final_pose.block<3, 1>(0, 3);
        Eigen::Vector3d target_pos = target_pose.block<3, 1>(0, 3);
        return (final_pos - target_pos).norm() < tolerance;
    }
}
