#ifndef ROBOT_ARM_CORE_JOINT_HPP
#define ROBOT_ARM_CORE_JOINT_HPP

// All angles are in radians [rad]
// All lengths are in centimeters [cm]

#include <utility>
#include <Eigen/Dense>

namespace robot_arm::core
{
    struct DHParameters
    {
        double a = 0.0;
        double alpha = 0.0;
        double d = 0.0;
        double theta = 0.0;
    };

    enum class JointType
    {
        Static,
        Revolute
    };

    class Joint
    {
    public:
        Joint(const DHParameters &dh_params, const JointType type,
              std::pair<Eigen::Vector3d, Eigen::Vector3d> joint_offsets = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
              double collision_radius = 0.0,
              std::pair<double, double> setting_range = {0.0, 0.0});

        // Getters
        // Returns the original DH parameters of the joint, NOT the current state
        const DHParameters &getDHParameters() const { return dh_params_; }
        // Returns the transformation matrix of the joint, actual state
        const Eigen::Matrix4d &getTransformationMatrix() const { return transformation_matrix_; }
        JointType getType() const { return type_; }

        std::pair<Eigen::Vector3d, Eigen::Vector3d> getOffsets() const { return joint_offsets_; }
        double getCollisionRadius() const { return collision_radius_; }

        std::pair<double, double> getSettingRange() const { return setting_range_; }
        double getSetting() const { return setting_; }

        // Setters
        bool setSetting(double setting);

    private:
        DHParameters dh_params_;
        Eigen::Matrix4d transformation_matrix_;
        JointType type_;

        std::pair<Eigen::Vector3d, Eigen::Vector3d> joint_offsets_;
        double collision_radius_ = 0.0;

        std::pair<double, double> setting_range_;
        double setting_ = 0.0;
    };
}

#endif
