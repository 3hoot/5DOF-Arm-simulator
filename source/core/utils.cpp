#include <Eigen/Dense>
#include <utility>
#include <cmath>

#include "robot_arm/core/utils.hpp"
#include "robot_arm/core/joint.hpp"

namespace robot_arm::core::utils
{
    bool jointIntersect(const Joint &joint1, const Eigen::Vector3d &pos1,
                        const Joint &joint2, const Eigen::Vector3d &pos2)
    {
        // Implementation of the intersection check between two joints
        // as specified in the article "Robust Computation of Distance Between Line Segments"
        // by David Eberly for Geometric Tools in Redmond WA 98052
        // link: https://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf

        // Get the start and end points of the lines representing the joints
        Eigen::Vector3d A0 = pos1 + joint1.getOffsets().first;
        Eigen::Vector3d A1 = pos1 + joint1.getOffsets().second;
        Eigen::Vector3d B0 = pos2 + joint2.getOffsets().first;
        Eigen::Vector3d B1 = pos2 + joint2.getOffsets().second;

        double a = (A1 - A0).dot(A1 - A0);
        double b = (A1 - A0).dot(B1 - B0);
        double c = (B1 - B0).dot(B1 - B0);
        double d = (A1 - A0).dot(A0 - B0);
        double e = (B1 - B0).dot(A0 - B0);
        double f = (A0 - B0).dot(A0 - B0);

        // Lambda function to calculate the distance squared between the two lines
        auto R = [&](double s, double t)
        { return a * s * s + 2 * b * s * t + c * t * t - d * s - e * t + f; };

        // Proper algorithm to find the closest points on the two line segments
        double det = a * c - b * b, s, t;
        if (det > 0) // nonparallel lines
        {
            double bte = b * e, ctd = c * d;
            if (bte <= ctd)
            {
                if (e <= 0)
                {
                    s = (-d >= a ? 1 : (-d > 0 ? -d / a : 0));
                    t = 0;
                }
                else if (e < c)
                {
                    s = 0;
                    t = e / c;
                }
                else
                {
                    s = (b - d >= a ? 1 : (b - d > 0 ? (b - d) / a : 0));
                    t = 1;
                }
            }
            else
            {
                s = bte - ctd;
                if (a >= det)
                {
                    if (b + e <= 0)
                    {
                        s = (-d <= 0 ? 0 : (-d < a ? -d / a : 1));
                        t = 0;
                    }
                    else if (b + e < c)
                    {
                        s = 1;
                        t = (b + e) / c;
                    }
                    else
                    {
                        s = (b - d <= 0 ? 0 : (b - d < a ? (b - d) / a : 1));
                        t = 1;
                    }
                }
                else
                {
                    double ate = a * e, btd = b * d;
                    if (ate <= btd)
                    {
                        s = (-d <= 0 ? 0 : (-d >= a ? 1 : -d / a));
                        t = 0;
                    }
                    else
                    {
                        t = ate - btd;
                        if (t >= det)
                        {
                            s = (b - d <= 0 ? 0 : (b - d >= a ? 1 : (b - d) / a));
                            t = 1;
                        }
                        else
                        {
                            s /= det;
                            t /= det;
                        }
                    }
                }
            }
        }
        else // parallel lines
        {
            if (e <= 0)
            {
                s = (-d <= 0 ? 0 : (-d >= a ? 1 : -d / a));
                t = 0;
            }
            else if (e >= c)
            {
                s = (b - d <= 0 ? 0 : (b - d >= a ? 1 : (b - d) / a));
                t = 1;
            }
            else
            {
                s = 0;
                t = e / c;
            }
        }

        double dist = std::sqrt(R(s, t));
        return dist < (joint1.getCollisionRadius() + joint2.getCollisionRadius());
    }

    Eigen::Matrix4d calculateTransformationMatrix(const DHParameters &parameters)
    {
        // An aliases for the parameters, sin and cos to improve readability
        const DHParameters &p = parameters;
        using std::sin, std::cos;

        // Calculate the transformation matrix based on the Modified Denavit-Hartenberg parameters
        Eigen::Matrix4d transformation_matrix;
        transformation_matrix << cos(p.theta), -sin(p.theta), 0, p.a,
            sin(p.theta) * cos(p.alpha), cos(p.theta) * cos(p.alpha), -sin(p.alpha), -p.d * sin(p.alpha),
            sin(p.theta) * sin(p.alpha), cos(p.theta) * sin(p.alpha), cos(p.alpha), p.d * cos(p.alpha),
            0, 0, 0, 1;

        return transformation_matrix;
    }
}
