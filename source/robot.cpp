#include <spdlog/spdlog.h>
#include <Eigen/Dense>
#include <utility>

#include "utils.hpp"
#include "config.hpp"
#include "robot.hpp"

Robot::Robot(const Eigen::Matrix<double, 6, 4> &DH_params,
             const std::array<std::pair<double, double>, 5> &joint_limits)
    : m_DH_params(DH_params),
      m_joint_limits(joint_limits),
      m_joint_relative_transforms{}
{
    spdlog::info("Robot initialized with (modified) DH parameters");

    // Initialize joint transformation matrices
    // Last joint is the end-effector
    for (size_t i = 0; i < static_cast<size_t>(m_DH_params.rows()); ++i)
    {
        // Calculate the joint transformation matrix for each joint
        double a = m_DH_params(i, 0);
        double alpha = m_DH_params(i, 1);
        double d = m_DH_params(i, 2);
        double theta = m_DH_params(i, 3);

        m_theta_offsets[i] = theta; // Store the initial theta offset
        m_joint_relative_transforms[i] = utils::getTransformationMatrix(a, alpha, d, theta);
    }
}

const Eigen::Matrix4d &Robot::setJointAngle(size_t joint_index, double angle_rad)
{
    checkValidJointIndex(joint_index);
    checkValidJointAngle(joint_index, angle_rad);

    // Set the joint angle
    m_joint_angles[joint_index] = angle_rad;

    // Update the joint transformation matrix for this joint
    double a = m_DH_params(joint_index, 0);
    double alpha = m_DH_params(joint_index, 1);
    double d = m_DH_params(joint_index, 2);
    double theta = m_DH_params(joint_index, 3) = angle_rad + m_theta_offsets[joint_index]; // Add the angle to the theta offset

    m_joint_relative_transforms[joint_index] = utils::getTransformationMatrix(a, alpha, d, theta);
    return m_joint_relative_transforms[joint_index];
}

// Getters
// -----------------------------------------------------------------------------

const Eigen::Matrix4d &Robot::getRelativeJointTransform(size_t joint_index) const
{
    checkValidJointIndex(joint_index);
    return m_joint_relative_transforms[joint_index];
}

const Eigen::Matrix4d Robot::getAbsoluteJointTransform(size_t joint_index) const
{
    checkValidJointIndex(joint_index);

    Eigen::Matrix4d absolute_transform = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i <= joint_index; ++i)
        absolute_transform *= m_joint_relative_transforms[i];

    return absolute_transform;
}

const Eigen::Vector3d Robot::getJointPosition(size_t joint_index) const
{
    checkValidJointIndex(joint_index);
    return Robot::getAbsoluteJointTransform(joint_index).block<3, 1>(0, 3);
}

const Eigen::Matrix3d Robot::getJointRotation(size_t joint_index) const
{
    checkValidJointIndex(joint_index);
    return Robot::getAbsoluteJointTransform(joint_index).block<3, 3>(0, 0);
}

int Robot::getJointCount() const
{
    return static_cast<int>(m_joint_relative_transforms.size());
}

double Robot::getJointAngle(size_t joint_index) const
{
    checkValidJointIndex(joint_index);
    return m_joint_angles[joint_index];
}

// -----------------------------------------------------------------------------

// Debugging methods
// -----------------------------------------------------------------------------

void Robot::modifyDHParams(int row, int col, double value)
{
    if (row < 0 || row >= m_DH_params.rows() || col < 0 || col >= m_DH_params.cols())
    {
        spdlog::error("Invalid DH parameter index: ({}, {})", row, col);
        throw std::out_of_range("Invalid DH parameter index");
    }
    m_DH_params(row, col) = value;

    // Reinitialize the joint transformation matrices
    for (size_t i = 0; i < static_cast<size_t>(m_DH_params.rows()); ++i)
    {
        double a = m_DH_params(i, 0);
        double alpha = m_DH_params(i, 1);
        double d = m_DH_params(i, 2);
        double theta = m_DH_params(i, 3);

        m_joint_relative_transforms[i] = utils::getTransformationMatrix(a, alpha, d, theta);
    }

    spdlog::info("Modified DH parameter at ({}, {}) to {}", row, col, value);
}

// -----------------------------------------------------------------------------

// Sanity checks
// -----------------------------------------------------------------------------

void Robot::checkValidJointAngle(size_t joint_index, double angle_rad) const
{
    if (!(angle_rad >= m_joint_limits[joint_index].first && angle_rad <= m_joint_limits[joint_index].second))
    {
        spdlog::error("Joint angle {} is out of limits for joint index {}. Valid range: [{}, {}]", angle_rad, joint_index, m_joint_limits[joint_index].first, m_joint_limits[joint_index].second);
        throw std::out_of_range("Invalid joint angle");
    }
}

void Robot::checkValidJointIndex(size_t joint_index) const
{
    if (!(joint_index < m_joint_relative_transforms.size()))
    {
        spdlog::error("Joint index {} is out of range. Valid range: [0, {}]", joint_index, m_joint_relative_transforms.size() - 1);
        throw std::out_of_range("Invalid joint index");
    }
}

// -----------------------------------------------------------------------------
