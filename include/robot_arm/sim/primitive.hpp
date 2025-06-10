#ifndef ROBOT_ARM_SIM_PRIMITIVE_HPP
#define ROBOT_ARM_SIM_PRIMITIVE_HPP

#include <Eigen/Dense>
#include <raylib.h>

#include "utils.hpp"

// All angles are in radians [rad]
// All lengths are in centimeters [cm]

namespace robot_arm::sim
{
    enum class PrimitiveType
    {
        Sphere, // Sphere primitive
        Cube,   // Cube primitive
    };

    // For now only spheres are supported as primitives
    class Primitive
    {
    public:
        Primitive(const Eigen::Matrix4d &transformation_matrix, double radius, Color color, PrimitiveType type)
            : transformation_matrix_(transformation_matrix), radius_(radius), color_(color), type_(type) {}

        // Getters
        const Eigen::Matrix4d &getTransformationMatrix() const { return transformation_matrix_; }
        PrimitiveType getType() const { return type_; }
        double getRadius() const { return radius_; }
        const Color &getColor() const { return color_; }
        Eigen::Vector3d getApproachVector(double approach_height, double approach_offset) const; // approach height is mesured
                                                                                                 // from the position of the
                                                                                                 // primitive

        // Sets the transformation matrix for the primitive
        void setTransformationMatrix(const Eigen::Matrix4d &transformation_matrix);
        void setColor(const Color &color) { color_ = color; }

    private:
        Eigen::Matrix4d transformation_matrix_; // Transformation matrix for the primitive,
                                                // relative to the world frame
        PrimitiveType type_;
        double radius_;
        Color color_;
    };
}

#endif
