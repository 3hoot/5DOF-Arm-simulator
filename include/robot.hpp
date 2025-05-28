#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Dense>
#include <utility>

class Robot
{
public:
    Robot(const Eigen::Matrix<double, 6, 4> &DH_params,
          const std::array<std::pair<double, double>, 5> &joint_limits);

    const Eigen::Matrix4d &setJointAngle(size_t joint_index, double angle_rad);
    const Eigen::Matrix4d &getRelativeJointTransform(size_t joint_index) const;
    const Eigen::Matrix4d getAbsoluteJointTransform(size_t joint_index) const;

    // Returns the absolute position of the selected joint [x, y, z]
    const Eigen::Vector3d getJointPosition(size_t joint_index) const;
    const Eigen::Matrix3d getJointRotation(size_t joint_index) const;
    int getJointCount() const;
    double getJointAngle(size_t joint_index) const;

    // Debugging methods
    void modifyDHParams(int row, int col, double value);

private:
    Eigen::Matrix<double, 6, 4> m_DH_params = {};                          // Denavit-Hartenberg parameters
    std::array<Eigen::Matrix4d, 6> m_joint_relative_transforms = {};       // Joint transformation matrices
    std::array<std::pair<double, double>, 5> m_joint_limits = {};          // Joint limits for each joint (min, max)
    std::array<double, 6> m_joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Joint angles in radians
    std::array<double, 6> m_theta_offsets;                                 // Theta offsets for each joint

    void Robot::checkValidJointIndex(size_t joint_index) const;                   // Throws an exception if the joint index is invalid
    void Robot::checkValidJointAngle(size_t joint_index, double angle_rad) const; // Throws an exception if the joint angle is invalid
};

#endif
