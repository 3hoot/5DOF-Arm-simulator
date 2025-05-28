#ifndef UTILS_HPP
#define UTILS_HPP

#include <raylib.h>
#include <Eigen/Dense>

namespace utils
{
    // Translator functions---------------------------------------------------------
    Vector3 toRotation(const Eigen::Matrix3d &rotationMatrix);
    Vector3 toPosition(const Eigen::Vector3d &vec);
    // -----------------------------------------------------------------------------

    Eigen::Matrix4d getTransformationMatrix(double a, double alpha, double d, double theta);
}

#endif
