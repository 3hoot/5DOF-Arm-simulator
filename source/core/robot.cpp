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

        // Set the ground clearance to a default value
        ground_clearance_ = 1.0; // Default minimum clearance above the ground plane
    }

    double Robot::getApproachDistance() const
    {
        // Calculate the approach distance based on the last joint's position
        if (joints_.empty())
            return 0.0; // No joints, no approach distance

        const Joint &last_joint = joints_.back();
        return last_joint.getDHParameters().d; // Add the d parameter of the last joint
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
            const Eigen::Matrix4d &matrix = transformation_matrices_[i];
            // Check if the joint's position is below the ground plane (z < 0)
            if (matrix(2, 3) < ground_clearance_)
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

    std::vector<double> Robot::solveIK(const Eigen::Vector3d &target_position,
                                       const Eigen::Vector3d &approach_vector,
                                       const double approach_angle)
    {
        // Solve the inverse kinematics for the robot arm to reach the target transformation matrix
        // This is a simplified version, assuming the first 3 joints are revolute and
        std::vector<double> settings = joint_settings_; // Start with the current joint settings

        const double offset_1 = joints_[0].getDHParameters().d +
                                joints_[1].getDHParameters().d; // Offset for the first joint (z-axis)
        const double offset_2 = joints_[2].getDHParameters().a; // Offset for the first joint (x-axis)

        // Constructing a triangle to solve for the second and third joints
        const double a = joints_[3].getDHParameters().a; // Length of the second link
        const double b = joints_[4].getDHParameters().a; // Length of the third link
        const double d = std::sqrt(std::pow(target_position.x(), 2) +
                                   std::pow(target_position.y(), 2)) -
                         offset_2;
        const double h = target_position.z() - offset_1;
        const double c = std::sqrt(std::pow(d, 2) + std::pow(h, 2));

        // Set the first joint to point towards the target position
        settings[1] = atan2(target_position.y(), target_position.x());

        // Clamp for acos
        double arg2 = (std::pow(b, 2) + std::pow(c, 2) - std::pow(a, 2)) / (2 * b * c);
        arg2 = std::clamp(arg2, -1.0, 1.0);
        settings[2] = -(std::numbers::pi / 2 - std::atan2(h, d) - std::acos(arg2));

        double arg3 = (std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2)) / (2 * a * b);
        arg3 = std::clamp(arg3, -1.0, 1.0);
        settings[3] = -(std::numbers::pi / 2 - std::acos(arg3));

        // Create dummy robot arm to calculate the end effector position
        std::vector<Joint> joints_copy = joints_;
        Robot dummy_robot(joints_copy);
        for (size_t i = 0; i < settings.size(); ++i)
            dummy_robot.setJointSetting(i, settings[i]);

        // Joint 4 alignment
        Eigen::Vector3d joint4_to_ee = (dummy_robot.getTransformationMatrices().back().block<3, 1>(0, 3) -
                                        dummy_robot.getTransformationMatrices()[4].block<3, 1>(0, 3))
                                           .normalized();
        Eigen::Vector3d approach_vector_normalized = approach_vector.normalized();
        // Get joint 4's rotation axis in world coordinates
        Eigen::Vector3d joint4_axis = dummy_robot.getTransformationMatrices()[4].block<3, 1>(0, 2).normalized();

        // Project both vectors onto the plane normal to joint4_axis
        Eigen::Vector3d joint4_to_ee_proj = (joint4_to_ee - joint4_to_ee.dot(joint4_axis) * joint4_axis).normalized();
        Eigen::Vector3d approach_proj = (approach_vector_normalized - approach_vector_normalized.dot(joint4_axis) * joint4_axis).normalized();

        double angle_to_approach = std::acos(joint4_to_ee_proj.dot(approach_proj));
        // Use atan2 for correct sign:
        double signed_angle = std::atan2(
            joint4_axis.dot(joint4_to_ee_proj.cross(approach_proj)),
            joint4_to_ee_proj.dot(approach_proj));
        settings[4] = settings[4] + signed_angle;

        // Joint 5 alignment
        settings[5] = approach_angle;

        return settings; // Return the calculated joint settings
    }
}
