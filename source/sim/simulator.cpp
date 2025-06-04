#include <vector>
#include <Eigen/Dense>
#include <raylib.h>
#include <string_view>
#include <numbers>
#include <unordered_map>
#include <functional>

#include "robot_arm/core/robot.hpp"
#include "robot_arm/sim/config.hpp"
#include "robot_arm/sim/simulator.hpp"
#include "robot_arm/sim/utils.hpp"

namespace robot_arm::sim
{
    Simulator::Simulator(core::Robot &robot, bool enable_debug,
                         int window_width, int window_height, const std::string_view window_title)
        : robot_(robot), enable_debug_(enable_debug),
          window_width_(window_width), window_height_(window_height), window_title_(window_title)
    {
        // Initialize Raylib camera
        // Hardcoded for now, but can be made configurable later
        camera_.position = {0.0f, 20.0f, 20.0f};
        camera_.target = {0.0f, 0.0f, 0.0f};
        camera_.up = {0.0f, 1.0f, 0.0f};
        camera_.fovy = 45.0f;
        camera_.projection = CAMERA_PERSPECTIVE;

        // Initialize Raylib window
        InitWindow(window_width_, window_height_, window_title_.data());
        SetTargetFPS(60);
    }

    Simulator::Simulator(core::Robot &robot, bool use_config)
        : Simulator(robot, config::ENABLE_DEBUG,
                    config::SCREEN_WIDTH, config::SCREEN_HEIGHT, config::WINDOW_TITLE) {}

    Simulator::~Simulator()
    {
        CloseWindow(); // Close Raylib window
    }

    void Simulator::run()
    {
        while (!WindowShouldClose())
        {
            if (IsCursorHidden())
                UpdateCamera(&camera_, CAMERA_FIRST_PERSON);

            // Toggle camera controls
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
            {
                if (IsCursorHidden())
                    EnableCursor();
                else
                    DisableCursor();
            }

            // Begin drawing
            BeginDrawing();
            ClearBackground(RAYWHITE);

            // Begin 3D mode
            BeginMode3D(camera_);
            DrawGrid(50, 5.0f);

            // Draw robot joints and end effector
            for (const auto &joint_matrix : robot_.getTransformationMatrices())
            {
                utils::Transform joint_transform = utils::toTransform(joint_matrix);

                Color joint_color = BLUE;
                if (joint_matrix == robot_.getTransformationMatrices().back())
                    joint_color = RED; // End effector is red
                if (joint_matrix == robot_.getTransformationMatrices().front())
                    joint_color = GREEN; // Base joint is green

                DrawCube(joint_transform.position, 0.5f, 0.5f, 0.5f, joint_color);
            }

            EndMode3D();

            // Draw debug information if enabled
            if (enable_debug_)
            {
                DrawText("Debug Mode Enabled", 10, 10, 20, DARKGRAY);
            }

            EndDrawing();
        }
    }
} // namespace robot_arm
