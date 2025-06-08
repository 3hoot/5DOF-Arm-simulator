#include <vector>
#include <Eigen/Dense>
#include <raylib.h>
#include <raymath.h>
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
                         Vector3 camera_up, int camera_projection,
                         float object_radius, float object_mounting_distance)
        : robot_(robot), enable_debug_(enable_debug),
          window_width_(window_width), window_height_(window_height), window_title_(window_title),
          object_radius_(object_radius), object_mounting_distance_(object_mounting_distance)
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

            float scaling_factor = 1.0f; // Scale factor
            robot_model.scale = {scaling_factor, scaling_factor, scaling_factor};
        }

        // Initialize spatial cursor position
        cursor3D_ = camera_.position;
    }

    Simulator::Simulator(core::Robot &robot, bool use_config)
        : Simulator(robot, config::ENABLE_DEBUG,
                    config::SCREEN_WIDTH, config::SCREEN_HEIGHT, config::WINDOW_TITLE,
                    config::CAMERA_FOVY, config::CAMERA_POSITION, config::CAMERA_TARGET,
                    config::CAMERA_UP, config::CAMERA_PROJECTION,
                    config::OBJECT_RADIUS, config::OBJECT_MOUNTING_DISTANCE) {}

    Simulator::~Simulator()
    {
        CloseWindow(); // Close Raylib window
    }

    void Simulator::run()
    {
        while (!WindowShouldClose())
        {
            // End effector position and orientation
            utils::Transform ee_transform = utils::toTransform(robot_.getTransformationMatrices().back());

            // Distance from end effector to object
            float distance_to_object = Vector3Distance(ee_transform.position, cursor3D_);

            // Toggle camera controls
            if (IsCursorHidden())
                UpdateCamera(&camera_, CAMERA_FIRST_PERSON);
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
            {
                if (IsCursorHidden())
                    EnableCursor();
                else
                    DisableCursor();
            }

            if (IsCursorHidden())
            {
                size_t &sji = selected_joint_index_;
                size_t joint_count = robot_.getTransformationMatrices().size();
                double joint_increment = std::numbers::pi / 90.0; // Increment in radians (5 degrees)
                double joint_setting = robot_.getJointSettings()[sji];

                // Move joints
                if (IsKeyPressed(KEY_I))
                    sji = (sji + 1) % joint_count;
                if (IsKeyPressed(KEY_K))
                    sji = (sji - 1 + joint_count) % joint_count;
                if (IsKeyDown(KEY_J))
                    robot_.setJointSetting(sji, joint_setting + joint_increment);
                if (IsKeyDown(KEY_L))
                    robot_.setJointSetting(sji, joint_setting - joint_increment);

                // Move camera
                if (IsKeyDown(KEY_Z))
                {
                    camera_.position.y -= 0.1f; // Move camera down
                    camera_.target.y -= 0.1f;   // Adjust camera target accordingly
                }
                if (IsKeyDown(KEY_X))
                {
                    camera_.position.y += 0.1f; // Move camera up
                    camera_.target.y += 0.1f;   // Adjust camera target accordingly
                }

                // Spawn object at cursor position
                if (IsKeyPressed(KEY_SPACE))
                {
                    if (!object_spawned_)
                    {
                        object_position_ = cursor3D_;
                        object_spawned_ = true; // Spawn the object
                    }
                    else
                    {
                        object_spawned_ = false; // Remove the object
                    }
                }

                // Attach object to end effector
                if (IsKeyPressed(KEY_T) && object_spawned_ && distance_to_object <= object_mounting_distance_)
                {
                    // Attach the object to the end effector
                    object_attached_ = true;
                    object_position_ = Vector3Add(
                        ee_transform.position,
                        Vector3Scale(ee_transform.rotation_axis, object_mounting_distance_));
                }

                // Detach object from end effector
                if (IsKeyPressed(KEY_G) && object_attached_)
                {
                    object_attached_ = false; // Detach the object
                }
            }
            else
            {
                // Spatial cursor controls
                if (IsKeyDown(KEY_W))
                    cursor3D_.z -= 0.1f; // Move cursor forward
                if (IsKeyDown(KEY_S))
                    cursor3D_.z += 0.1f; // Move cursor backward
                if (IsKeyDown(KEY_A))
                    cursor3D_.x -= 0.1f; // Move cursor left
                if (IsKeyDown(KEY_D))
                    cursor3D_.x += 0.1f; // Move cursor right
                if (IsKeyDown(KEY_Z))
                    cursor3D_.y -= 0.1f; // Move cursor down
                if (IsKeyDown(KEY_X))
                    cursor3D_.y += 0.1f; // Move cursor up
            }
            updateRobotModels(); // Update robot models based on the current state of the robot

            // Begin drawing
            BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera_);
            DrawGrid(50, 5.0f);

            // Draw primitive object if spawned
            if (object_spawned_)
            {
                DrawSphere(object_position_, object_radius_, RED);
                DrawSphereWires(object_position_, object_radius_, 16, 16, MAROON);
            }

            // Draw robot
            for (size_t i = 0; i < robot_.getTransformationMatrices().size(); ++i)
            {
                auto &robot_model = robot_models_[i];
                utils::Transform transform = robot_model.local_transform * robot_model.global_transform;
                DrawModelEx(robot_model.model, transform.position, transform.rotation_axis,
                            transform.rotation_angle, robot_model.scale, GRAY);
                DrawModelWiresEx(robot_model.model, transform.position, transform.rotation_axis,
                                 transform.rotation_angle, robot_model.scale, DARKGRAY);
            }

            if (enable_debug_)
            {
                // Draw spatial cursor
                DrawSphere(cursor3D_, 0.5f, RED);
                DrawSphere(ee_transform.position, 0.5f, GREEN);

                // Draw line connecting end effector to the spatial cursor position
                DrawLine3D(ee_transform.position, cursor3D_, BLUE);
            }

            EndMode3D();

            if (enable_debug_)
            {
                DrawText("Debug Mode Enabled", 10, 10, 20, DARKGRAY);

                // Robot joint information
                std::string joint_info = "Selected Joint: " + std::to_string(selected_joint_index_) +
                                         " | Setting: " + std::to_string(robot_.getJointSettings()[selected_joint_index_]);
                DrawText(joint_info.c_str(), 10, 40, 20, DARKGRAY);

                std::string end_info = "End Effector Position: x: " +
                                       std::to_string(ee_transform.position.x) +
                                       " y: " + std::to_string(ee_transform.position.y) +
                                       " z: " + std::to_string(ee_transform.position.z);
                DrawText(end_info.c_str(), 10, 70, 20, DARKGRAY);

                // Camera information
                std::string camera_info = "Camera Position: x: " +
                                          std::to_string(camera_.position.x) +
                                          " y: " + std::to_string(camera_.position.y) +
                                          " z: " + std::to_string(camera_.position.z);
                DrawText(camera_info.c_str(), 10, 100, 20, DARKGRAY);

                // Spatial cursor information
                std::string cursor_info = "Cursor Position: x: " +
                                          std::to_string(cursor3D_.x) +
                                          " y: " + std::to_string(cursor3D_.y) +
                                          " z: " + std::to_string(cursor3D_.z);
                DrawText(cursor_info.c_str(), 10, 130, 20, DARKGRAY);
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
