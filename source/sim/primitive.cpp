#include <Eigen/Dense>
#include <raylib.h>
#include <iostream>

#include "robot_arm/sim/primitive.hpp"

namespace robot_arm::sim
{
    Eigen::Vector3d Primitive::getApproachVector(double approach_height, double approach_offset) const
    {
        switch (type_)
        {
        case PrimitiveType::Sphere:
        {
            const Eigen::Vector3d &position = transformation_matrix_.block<3, 1>(0, 3);
            const Eigen::Vector3d target = Eigen::Vector3d(position.x(), position.y(), approach_height + position.z());
            const Eigen::Vector3d approach_vector = (target - position).normalized() * (radius_ + approach_offset);
            return -approach_vector;
        }
        case PrimitiveType::Cube:
        {
            return Eigen::Vector3d(0.0, 0.0, -(radius_ + approach_offset)); // For cubes, the approach vector is always downwards
        }
        default:
        {
            return Eigen::Vector3d(0.0, 0.0, 0.0); // Default case, should not happen
        }
        }
    }

    void Primitive::setTransformationMatrix(const Eigen::Matrix4d &transformation_matrix)
    {
        transformation_matrix_ = transformation_matrix;
    }
}
