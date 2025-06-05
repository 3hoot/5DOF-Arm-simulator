#include <vector>
#include <Eigen/Dense>
#include <raylib.h>
#include <string_view>
#include <numbers>
#include <unordered_map>
#include <functional>
#include <iostream>

#include "robot_arm/core/robot.hpp"
#include "robot_arm/sim/config.hpp"
#include "robot_arm/sim/simulator.hpp"
#include "robot_arm/sim/utils.hpp"

namespace robot_arm::sim
{
    Simulator::Simulator(core::Robot &robot, bool enable_debug,
                         int window_width, int window_height, const std::string_view window_title,
                         float camera_fovy, Vector3 camera_position, Vector3 camera_target,
                         Vector3 camera_up, int camera_projection)
        : robot_(robot), enable_debug_(enable_debug),
          window_width_(window_width), window_height_(window_height), window_title_(window_title)
    {
        // Initialize Raylib camera
        camera_.position = camera_position;
        camera_.target = camera_target;
        camera_.up = camera_up;
        camera_.fovy = camera_fovy;
        camera_.projection = camera_projection;

        // Initialize Raylib window
        InitWindow(window_width_, window_height_, window_title_.data());
        SetTargetFPS(60);

        // Loading robot models
        robot_models_.resize(robot_.getTransformationMatrices().size());
        for (auto &robot_model : robot_models_)
        {
            size_t index = &robot_model - &robot_models_[0];
            robot_model.model = LoadModel(config::ROBOT_MODELS[index].data());
            robot_model.global_transform = utils::toTransform(robot_.getTransformationMatrices()[index]);
            robot_model.local_transform = config::ROBOT_MODELS_LOCAL_TRANSFORMS[index];

            float scaling_factor = 0.1f; // Scale factor
            robot_model.scale = {scaling_factor, scaling_factor, scaling_factor};
        }
    }

    Simulator::Simulator(core::Robot &robot, bool use_config)
        : Simulator(robot, config::ENABLE_DEBUG,
                    config::SCREEN_WIDTH, config::SCREEN_HEIGHT, config::WINDOW_TITLE,
                    config::CAMERA_FOVY, config::CAMERA_POSITION, config::CAMERA_TARGET,
                    config::CAMERA_UP, config::CAMERA_PROJECTION) {}

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

            // Move joints
            if (IsCursorHidden())
            {
                size_t &sji = selected_joint_index_;
                size_t joint_count = robot_.getTransformationMatrices().size();
                double joint_increment = std::numbers::pi / 90.0; // Increment in radians (5 degrees)
                double joint_setting = robot_.getJointSettings()[sji];

                if (IsKeyPressed(KEY_I))
                    sji = (sji + 1) % joint_count;
                if (IsKeyPressed(KEY_K))
                    sji = (sji - 1 + joint_count) % joint_count;
                if (IsKeyDown(KEY_J))
                    robot_.setJointSetting(sji, joint_setting + joint_increment);
                if (IsKeyDown(KEY_L))
                    robot_.setJointSetting(sji, joint_setting - joint_increment);

                if (IsKeyDown(KEY_Z))
                    camera_.position.y -= 0.1f; // Move camera down
                if (IsKeyDown(KEY_X))
                    camera_.position.y += 0.1f; // Move camera up
            }

            updateRobotModels(); // Update robot models based on the current state of the robot

            // Begin drawing
            BeginDrawing();
            ClearBackground(RAYWHITE);

            // Begin 3D mode
            BeginMode3D(camera_);
            DrawGrid(50, 5.0f);

            // Draw
            // for (size_t i = 0; i < 2; ++i)
            // {
            //     auto &robot_model = robot_models_[i];
            //     utils::Transform transform = robot_model.local_transform * robot_model.global_transform;
            //     DrawModelEx(robot_model.model, transform.position, transform.rotation_axis,
            //                 transform.rotation_angle, robot_model.scale, GRAY);
            //     DrawModelWiresEx(robot_model.model, transform.position, transform.rotation_axis,
            //                      transform.rotation_angle, robot_model.scale, DARKGRAY);
            // }

            // Draw robot joints and end effector
            Color joint_color = {0, 0, 255, 255}; // Default joint color (blue)
            for (size_t i = 0; i < robot_.getTransformationMatrices().size(); ++i)
            {
                const auto &joint_matrix = robot_.getTransformationMatrices()[i];
                size_t joint_index = &joint_matrix - &robot_.getTransformationMatrices()[0];
                utils::Transform joint_transform = utils::toTransform(joint_matrix);
                DrawCube(joint_transform.position, 0.5f, 0.5f, 0.5f, joint_color);

                utils::Transform start_offset = utils::toTransform(robot_.getJointOffsets()[joint_index].first);
                DrawCubeWires(start_offset.position, 0.3f, 0.3f, 0.3f, joint_color);

                utils::Transform end_offset = utils::toTransform(robot_.getJointOffsets()[joint_index].second);
                DrawCubeWires(end_offset.position, 0.3f, 0.3f, 0.3f, joint_color);

                // Update joint color based on the selected joint index
                joint_color.r += 20;
                joint_color.g += 40;
                joint_color.b -= 60;
                if (joint_color.r > 255)
                    joint_color.r = 0;
                if (joint_color.g > 255)
                    joint_color.g = 0;
                if (joint_color.b < 0)
                    joint_color.b = 255;
            }

            EndMode3D();

            // Draw debug information if enabled
            if (enable_debug_)
            {
                DrawText("Debug Mode Enabled", 10, 10, 20, DARKGRAY);
                std::string joint_info = "Selected Joint: " + std::to_string(selected_joint_index_) +
                                         " | Setting: " + std::to_string(robot_.getJointSettings()[selected_joint_index_]);
                DrawText(joint_info.c_str(), 10, 40, 20, DARKGRAY);
            }

            EndDrawing();
        }
    }

    void Simulator::updateRobotModels()
    {
        for (size_t i = 0; i < robot_models_.size(); ++i)
        {
            const auto &joint_matrix = robot_.getTransformationMatrices()[i];
            robot_models_[i].global_transform = utils::toTransform(joint_matrix);
        }
    }
} // namespace robot_arm
