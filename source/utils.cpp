#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <raylib.h>
#include <Eigen/Dense>

#include "utils.hpp"

namespace utils
{
    // Converts an Eigen rotation matrix to a Raylib Vector3 representation
    Vector3 toRotation(const Eigen::Matrix3d &rotationMatrix)
    {
        // Fix for Z-up (robotics) to Y-up (Raylib): rotate -90° about X
        Eigen::Matrix3d fix = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d raylibRot = fix * rotationMatrix;

        // Extract Euler angles from the fixed rotation matrix
        double sy = sqrt(raylibRot(0, 0) * raylibRot(0, 0) + raylibRot(1, 0) * raylibRot(1, 0));
        bool singular = sy < 1e-6;

        double x, y, z;
        if (!singular)
        {
            x = atan2(raylibRot(2, 1), raylibRot(2, 2));
            y = atan2(-raylibRot(2, 0), sy);
            z = atan2(raylibRot(1, 0), raylibRot(0, 0));
        }
        else
        {
            x = atan2(-raylibRot(1, 2), raylibRot(1, 1));
            y = atan2(-raylibRot(2, 0), sy);
            z = 0;
        }

        return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }

    Vector3 toPosition(const Eigen::Vector3d &vec)
    {
        // Switching y and z components to match Raylib's coordinate system
        return {static_cast<float>(vec.x()), static_cast<float>(vec.z()), static_cast<float>(vec.y())};
    }

    Eigen::Matrix4d getTransformationMatrix(double a, double alpha, double d, double theta)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        // RotX(alpha)
        Eigen::Matrix4d Rx = Eigen::Matrix4d::Identity();
        Rx(1, 1) = cos(alpha);
        Rx(1, 2) = -sin(alpha);
        Rx(2, 1) = sin(alpha);
        Rx(2, 2) = cos(alpha);

        // TransX(a)
        Eigen::Matrix4d Tx = Eigen::Matrix4d::Identity();
        Tx(0, 3) = a;

        // RotZ(theta)
        Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
        Rz(0, 0) = cos(theta);
        Rz(0, 1) = -sin(theta);
        Rz(1, 0) = sin(theta);
        Rz(1, 1) = cos(theta);

        // TransZ(d)
        Eigen::Matrix4d Tz = Eigen::Matrix4d::Identity();
        Tz(2, 3) = d;

        T = Rx * Tx * Rz * Tz;
        return T;
    }
}
