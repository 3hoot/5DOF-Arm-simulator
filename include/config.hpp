#ifndef CONFIG_HPP
#define CONFIG_HPP

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <Eigen/Dense>

class Config
{
public:
    // Delete copy/move constructor and assignment operator to enforce singleton pattern
    Config(const Config &) = delete;
    Config &operator=(const Config &) = delete;
    Config(const Config &&) = delete;
    Config &operator=(const Config &&) = delete;

    static Config &instance()
    {
        static Config instance;
        return instance;
    }

    //// Configuration values
    // Raylib
    static constexpr int WINDOW_WIDTH = 1280;
    static constexpr int WINDOW_HEIGHT = 720;
    static constexpr char *WINDOW_TITLE = "Arm Simulator";
    static constexpr int FPS = 60;
    static constexpr float CAMERA_FOV = 45;

    // Robot
    static constexpr double LINK_1 = 2.0;
    static constexpr double LINK_2 = 3.0;
    static constexpr double LINK_3 = 8.0;
    static constexpr double LINK_4 = 5.0;
    static constexpr double LINK_5 = 1.0;

    typedef Eigen::Matrix<double, 6, 4> DHParams; // Type alias for Denavit-Hartenberg parameters
    const DHParams DH_PARAMS = (Eigen::Matrix<double, 6, 4>() << 0.0, 0.0, LINK_1, 0.0,
                                LINK_2, M_PI / 2, 0.0, M_PI / 2,
                                LINK_3, 0.0, 0.0, -M_PI / 2,
                                LINK_4, 0.0, 0.0, M_PI / 2,
                                0.0, M_PI / 2, 0.0, 0.0,
                                0.0, 0.0, LINK_5, 0.0)
                                   .finished(); // [a, alpha, d, theta]

    typedef std::array<std::pair<double, double>, 5> JointLimits; // Type alias for joint limits
    static constexpr JointLimits JOINT_LIMITS = {{{0.0, 2 * M_PI},
                                                  {-M_PI / 2, M_PI / 2},
                                                  {-M_PI / 4, 5 * M_PI / 4},
                                                  {-M_PI / 2, M_PI / 2},
                                                  {0.0, 2 * M_PI}}}; // Joint limits in radians [min, max]

private:
    Config() = default;
};

#endif
