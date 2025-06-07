#include <Eigen/Dense>
#include <utility>

#include "robot_arm/core/joint.hpp"
#include "robot_arm/core/utils.hpp"

namespace robot_arm::core
{
    // Joint class implementation
    Joint::Joint(const DHParameters &dh_params, const JointType type,
                 std::pair<double, double> setting_range)
        : dh_params_(dh_params), type_(type),
          setting_range_(setting_range)
    {
        // Initialize the transformation matrix using the DH parameters
        transformation_matrix_ = utils::calculateTransformationMatrix(dh_params_);
    }

    bool Joint::setSetting(double setting)
    {
        if (type_ == JointType::Static)
            return false; // Static joints cannot have their settings changed

        if (setting < setting_range_.first || setting > setting_range_.second)
            return false; // Setting out of range

        // Prepare a new set of DH parameters based on the current setting
        DHParameters new_params = dh_params_;

        switch (type_)
        {
        case JointType::Revolute:

            new_params.theta += setting; // Update the theta parameter for revolute joints
            transformation_matrix_ = utils::calculateTransformationMatrix(new_params);
            setting_ = setting; // Update the current setting
            return true;        // Successfully set the new setting

        default:
            return false; // Unsupported joint type
        }
    }
}
