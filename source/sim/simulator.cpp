#include <vector>
#include <Eigen/Dense>
#include <raylib.h>
#include <raymath.h>
#include <string_view>
#include <numbers>
#include <functional>
#include <iostream>
#include <iomanip>

#include "robot_arm/core/robot.hpp"
#include "robot_arm/sim/config.hpp"
#include "robot_arm/sim/simulator.hpp"
#include "robot_arm/sim/utils.hpp"
#include "robot_arm/sim/primitive.hpp"

namespace robot_arm::sim
{
    Simulator::Simulator(core::Robot &robot)
        : robot_(robot), selected_joint_index_(0)
    {
        // Initialize Raylib camera
        camera_.position = config::CAMERA_POSITION;
        camera_.target = config::CAMERA_TARGET;
        camera_.up = config::CAMERA_UP;
        camera_.fovy = config::CAMERA_FOVY;
        camera_.projection = config::CAMERA_PROJECTION;

        // Initialize Raylib window
        InitWindow(config::SCREEN_WIDTH, config::SCREEN_HEIGHT, config::WINDOW_TITLE.data());
        SetTargetFPS(config::TARGET_FPS);

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

        // Loading primitives
        max_primitives_ = config::MAX_PRIMITIVES;
        primitives_.reserve(max_primitives_);

        float sphere_scaling_factor = 0.1f; // Scale factor for sphere primitives
        sphere_primitive_model_.model = LoadModel(config::PRIMITIVE_SPHERE_MODEL.data());
        sphere_primitive_model_.local_transform = config::PRIMITIVE_SPHERE_LOCAL_TRANSFORM;
        sphere_primitive_model_.scale = {sphere_scaling_factor, sphere_scaling_factor, sphere_scaling_factor};

        float cube_scaling_factor = 0.1f; // Scale factor for cube primitives
        cube_primitive_model_.model = LoadModel(config::PRIMITIVE_CUBE_MODEL.data());
        cube_primitive_model_.local_transform = config::PRIMITIVE_CUBE_LOCAL_TRANSFORM;
        cube_primitive_model_.scale = {cube_scaling_factor, cube_scaling_factor, cube_scaling_factor};

        // Initialize the simulator mode
        mode_ = config::SIMULATOR_DEFAULT_MODE;

        // Initialize target robot settings with default joint settings
        target_robot_settings_ = robot_.getJointSettings();
        target_position_ = robot_.getTransformationMatrices()[4].block<3, 1>(0, 3);
    }

    Simulator::~Simulator()
    {
        CloseWindow(); // Close Raylib window
    }

    void Simulator::run()
    {
        while (!WindowShouldClose())
        {
            // Smoothly interpolate the robot settings towards the target settings
            if (robotIsMoving(config::ROBOT_SETTINGS_DIFFERENCE_THRESHOLD) &&
                mode_ != SimulatorMode::IndividualJointControl)
                updateRobotSettings(config::ROBOT_SETTINGS_SMOOTHING_FACTOR,
                                    config::ROBOT_SETTINGS_DIFFERENCE_THRESHOLD);

            // Update all models based on their current transformation matrices
            updateRobotModels();
            updatePrimitiveModels();

            // Clamp selected primitive index to valid range
            if (!primitives_.empty())
                selected_primitive_index_ = std::clamp(selected_primitive_index_, 0, static_cast<int>(primitives_.size() - 1));
            else
                selected_primitive_index_ = 0; // Reset to 0 if no primitives are available

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

            // Main input handling
            if (IsCursorHidden())
            {
                // Spawn primitive cube with key Y
                if (IsKeyPressed(KEY_Y) && primitives_.size() < max_primitives_)
                {
                    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
                    Eigen::Vector3d ee_proj = robot_.getTransformationMatrices().back().block<3, 1>(0, 3);
                    ee_proj[2] = config::PRIMITIVE_RADIUS; // Project end effector position onto the ground

                    transformation_matrix.block<3, 1>(0, 3) = ee_proj;
                    addPrimitive(transformation_matrix, PrimitiveType::Cube, config::PRIMITIVE_RADIUS);
                }

                // Spawn primitive sphere with key U
                if (IsKeyPressed(KEY_U) && primitives_.size() < max_primitives_)
                {
                    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
                    Eigen::Vector3d ee_proj = robot_.getTransformationMatrices().back().block<3, 1>(0, 3);
                    ee_proj[2] = config::PRIMITIVE_RADIUS; // Project end effector position onto the ground

                    transformation_matrix.block<3, 1>(0, 3) = ee_proj;
                    addPrimitive(transformation_matrix, PrimitiveType::Sphere, config::PRIMITIVE_RADIUS);
                }

                // Primitive selection and removal
                if (IsKeyPressed(KEY_J) && !primitives_.empty() && !holding_primitive_) // Next primitive
                    selected_primitive_index_ = (selected_primitive_index_ + 1) % primitives_.size();
                if (IsKeyPressed(KEY_H) && !primitives_.empty() && !holding_primitive_) // Previous primitive
                    selected_primitive_index_ = (selected_primitive_index_ - 1 + primitives_.size()) % static_cast<int>(primitives_.size());

                if (IsKeyPressed(KEY_I)) // Remove selected primitive
                {
                    if (!primitives_.empty())
                    {
                        removePrimitive(selected_primitive_index_);
                        selected_primitive_index_ = std::min(selected_primitive_index_, static_cast<int>(primitives_.size() - 1));
                    }
                }

                // Primitive targeting
                if (IsKeyPressed(KEY_K))
                {
                    if (!primitives_.empty())
                    {
                        // Get the transformation matrix of the selected primitive
                        const Primitive &selected_primitive = primitives_[selected_primitive_index_];
                        const Eigen::Vector3d approach_vector = selected_primitive.getApproachVector(
                            config::ROBOT_EE_APPROACH_HEIGHT, config::ROBOT_EE_APPROACH_OFFSET);
                        const Eigen::Matrix4d &selected_primitive_matrix = selected_primitive.getTransformationMatrix();

                        // Set the target position to the center of the selected primitive

                        target_position_ = selected_primitive_matrix.block<3, 1>(0, 3) -
                                           approach_vector; // Adjust the target position based on the approach vector
                        target_robot_settings_ = robot_.solveIK(
                            target_position_, approach_vector, std::atan2(approach_vector[1], approach_vector[0]));
                    }
                }

                // Primitive picking and placing
                if (IsKeyPressed(KEY_L))
                {
                    if (!primitives_.empty())
                    {
                        Primitive &selected_primitive = primitives_[selected_primitive_index_];
                        double distance_to_primitive = (robot_.getTransformationMatrices().back().block<3, 1>(0, 3) -
                                                        selected_primitive.getTransformationMatrix().block<3, 1>(0, 3))
                                                           .norm();

                        // Check if the robot can pick up the primitive
                        if (!holding_primitive_ && distance_to_primitive <= selected_primitive.getRadius() + config::ROBOT_EE_ATTACH_DISTANCE)
                            holding_primitive_ = true; // Pick up the primitive
                        else if (holding_primitive_)
                            holding_primitive_ = false; // Release the primitive
                    }
                }

                // Camera controls
                if (IsKeyDown(KEY_Z))
                {
                    camera_.position.y -= config::SIMULATOR_CAMERA_SPEED; // Move camera down
                    camera_.target.y -= config::SIMULATOR_CAMERA_SPEED;   // Adjust camera target accordingly
                }
                if (IsKeyDown(KEY_X))
                {
                    camera_.position.y += config::SIMULATOR_CAMERA_SPEED; // Move camera up
                    camera_.target.y += config::SIMULATOR_CAMERA_SPEED;   // Adjust camera target accordingly
                }

                // Switch between control modes
                switchMode(KEY_TAB);

                size_t min_joint = 1;
                size_t max_joint = robot_.getTransformationMatrices().size() - 2;

                switch (mode_)
                {
                case SimulatorMode::IndividualJointControl:

                    // Keys 4 and 6 on the numpad to switch joints (skip joint 0 and last joint)
                    if (IsKeyPressed(KEY_KP_6)) // Next joint
                    {
                        selected_joint_index_++;
                        if (selected_joint_index_ > max_joint)
                            selected_joint_index_ = min_joint;
                    }
                    if (IsKeyPressed(KEY_KP_4)) // Previous joint
                    {
                        if (selected_joint_index_ <= min_joint)
                            selected_joint_index_ = max_joint;
                        else
                            selected_joint_index_--;
                    }

                    // Keys 8 and 2 on the numpad to increase/decrease joint settings
                    if (IsKeyDown(KEY_KP_8)) // Increase joint setting
                        robot_.setJointSetting(selected_joint_index_, robot_.getJointSettings()[selected_joint_index_] + config::ROBOT_SETTING_INCREMENT);
                    if (IsKeyDown(KEY_KP_2)) // Decrease joint setting
                        robot_.setJointSetting(selected_joint_index_, robot_.getJointSettings()[selected_joint_index_] - config::ROBOT_SETTING_INCREMENT);

                    break;
                case SimulatorMode::EndEffectorControl:

                    // Get the position input from the user
                    getPositionInput(target_position_, config::ROBOT_EE_MOVEMENT_INCREMENT);

                    // Calculate the approach angle based on the end effector position
                    double approach_angle = std::atan2(target_position_[1], target_position_[0]);

                    // Solve IK for the end effector target
                    target_robot_settings_ = robot_.solveIK(target_position_, Eigen::Vector3d(0.0, 0.0, -1.0), approach_angle);

                    break;
                }
            }

            // Begin drawing
            BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera_);
            DrawGrid(50, 5.0f);

            // Draw end effector projection on the ground
            if (mode_ == SimulatorMode::EndEffectorControl)
            {
                Eigen::Vector3d ee_position = robot_.getTransformationMatrices().back().block<3, 1>(0, 3);
                Vector3 ee_proj = {static_cast<float>(ee_position[0]),
                                   0.0f,
                                   static_cast<float>(-ee_position[1])};
                DrawSphere(ee_proj, 0.5f, RED);
                // DrawLine3D(end_effector_position, {end_effector_position.x, 0.0f, end_effector_position.z}, RED);
            }

            // Draw the robot
            drawRobot();
            // Draw primitives
            drawPrimitives();

            EndMode3D();

            // Draw information text
            drawInformation();
            // Draw help text
            if (config::ENABLE_HELP)
                drawHelp();
            // Draw debug information
            if (config::ENABLE_DEBUG)
                drawDebug();

            EndDrawing();
        }
    }

    void Simulator::drawRobot()
    {
        // Draw each robot model with its global transformation
        for (size_t i = 0; i < robot_models_.size(); ++i)
        {
            const auto &robot_model = robot_models_[i];
            utils::Transform transform = robot_model.local_transform * robot_model.global_transform;
            DrawModelEx(robot_model.model, transform.position, transform.rotation_axis,
                        transform.rotation_angle, robot_model.scale, GRAY);

            if (config::ENABLE_DEBUG && config::ENABLE_WIRES)
                DrawModelWiresEx(robot_model.model, transform.position, transform.rotation_axis,
                                 transform.rotation_angle, robot_model.scale, DARKGRAY);
        }
    }

    void Simulator::drawPrimitives()
    {
        // Draw each primitive with its transformation
        for (const auto &primitive : primitives_)
        {
            utils::Transform transform = utils::toTransform(primitive.getTransformationMatrix());
            utils::FusionModel &primitive_model =
                (primitive.getType() == PrimitiveType::Sphere) ? sphere_primitive_model_ : cube_primitive_model_;

            DrawModelEx(primitive_model.model, transform.position, transform.rotation_axis,
                        transform.rotation_angle, primitive_model.scale, primitive.getColor());

            if (config::ENABLE_DEBUG && config::ENABLE_WIRES)
                DrawModelWiresEx(primitive_model.model, transform.position, transform.rotation_axis,
                                 transform.rotation_angle, primitive_model.scale, DARKGRAY);
        }
    }

    void Simulator::drawInformation() const
    {
        // Draw background for information text
        DrawRectangle(0, 0, config::SCREEN_WIDTH, 40, Fade(BLACK, 0.75f));

        // Title and version information
        DrawText("Robot Arm Simulator v1.0 by Mikolaj Kurylo 193743", 10, 10, 20, WHITE);

        // Draw end effector position on the right side of the screen
        Eigen::Vector3d end_effector_position = robot_.getTransformationMatrices().back().block<3, 1>(0, 3);
        std::string end_effector_info = "End Effector Position: x: " + std::to_string(end_effector_position[0]) +
                                        " y: " + std::to_string(end_effector_position[1]) +
                                        " z: " + std::to_string(end_effector_position[2]);
        DrawText(end_effector_info.c_str(), config::SCREEN_WIDTH - 630, 10, 20, GRAY);
    }

    void Simulator::drawHelp() const
    {
        // Draw controls information at the right side of the screen
        // Draw semi-transparent background for help section
        const int help_height = 540;
        const int help_width = 400;
        const int help_x = config::SCREEN_WIDTH - help_width - 20;
        const int help_y = config::SCREEN_HEIGHT - help_height - 20;
        DrawRectangle(help_x, help_y, help_width, help_height, Fade(BLACK, 0.85f));

        int line = 0;
        const int line_height = 22;
        const int text_x = help_x + 20;
        const int text_y = help_y + 15;

        DrawText("Controls", text_x, text_y + line_height * line++, 22, YELLOW);

        line++; // Spacer

        DrawText("Camera:", text_x, text_y + line_height * line++, 20, LIGHTGRAY);
        DrawText("  WASD / Arrow Keys: Move", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  Z / X: Move Up / Down", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  Right Click: Toggle Cursor", text_x + 20, text_y + line_height * line++, 18, WHITE);

        line++; // Spacer

        DrawText("Modes:", text_x, text_y + line_height * line++, 20, LIGHTGRAY);
        DrawText("  Tab: Switch Control Mode", text_x + 20, text_y + line_height * line++, 18, WHITE);

        line++; // Spacer

        DrawText("Joint/EE Control (Numpad):", text_x, text_y + line_height * line++, 20, LIGHTGRAY);
        DrawText("  4 / 6: Select Joint", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  8 / 2: Increase / Decrease", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  7 / 9: Move Up / Down (EE)", text_x + 20, text_y + line_height * line++, 18, WHITE);

        line++; // Spacer

        DrawText("Primitives:", text_x, text_y + line_height * line++, 20, LIGHTGRAY);
        DrawText("  Y: Spawn Cube", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  U: Spawn Sphere", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  J / H: Next / Previous", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  I: Remove Selected", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  K: Target Selected", text_x + 20, text_y + line_height * line++, 18, WHITE);
        DrawText("  L: Pick / Place", text_x + 20, text_y + line_height * line++, 18, WHITE);
    }

    void Simulator::drawDebug() const
    {

        // Draw semi-transparent background for debug section
        const int debug_height = 400;
        const int debug_width = 400;
        const int debug_x = config::SCREEN_WIDTH - debug_width - 20;
        const int debug_y = 60;
        DrawRectangle(debug_x, debug_y, debug_width, debug_height, Fade(BLACK, 0.85f));

        int line = 0;
        const int line_height = 22;
        const int text_x = debug_x + 20;
        const int text_y = debug_y + 15;

        DrawText("Debug Info", text_x, text_y + line_height * line++, 22, YELLOW);

        line++; // Spacer

        // Simulation mode
        std::string mode_info;
        switch (mode_)
        {
        case SimulatorMode::IndividualJointControl:
            mode_info = "Mode: Individual Joint Control";
            break;
        case SimulatorMode::EndEffectorControl:
            mode_info = "Mode: End Effector Control";
            break;
        }
        DrawText(mode_info.c_str(), text_x, text_y + line_height * line++, 20, LIGHTGRAY);

        line++; // Spacer

        // Joint settings
        DrawText("Joint Settings:", text_x, text_y + line_height * line++, 20, LIGHTGRAY);
        for (size_t i = 1; i < robot_.getJointSettings().size() - 1; ++i)
        {
            std::string joint_info = "  Joint " + std::to_string(i) + ": " +
                                     std::to_string(robot_.getJointSettings()[i]);
            DrawText(joint_info.c_str(), text_x + 20, text_y + line_height * line++, 18, WHITE);
        }
        std::string selected_joint_info = "Selected Joint: " + std::to_string(selected_joint_index_);
        DrawText(selected_joint_info.c_str(), text_x, text_y + line_height * line++, 18, GREEN);

        line++; // Spacer

        // Primitive info
        DrawText("Primitives:", text_x, text_y + line_height * line++, 20, LIGHTGRAY);
        std::string primitive_info = "  Count: " + std::to_string(primitives_.size());
        DrawText(primitive_info.c_str(), text_x + 20, text_y + line_height * line++, 18, WHITE);
        std::string selected_primitive_info = "  Selected: " + std::to_string(selected_primitive_index_);
        DrawText(selected_primitive_info.c_str(), text_x + 20, text_y + line_height * line++, 18, WHITE);

        if (selected_primitive_index_ < primitives_.size())
        {
            const auto &primitive = primitives_[selected_primitive_index_];
            std::string primitive_details;
            switch (primitive.getType())
            {
            case PrimitiveType::Sphere:
                primitive_details = "    Sphere | Radius: " + std::to_string(primitive.getRadius());
                break;
            case PrimitiveType::Cube:
                primitive_details = "    Cube | Side: " + std::to_string(primitive.getRadius() * 2.0);
                break;
            }
            DrawText(primitive_details.c_str(), text_x + 40, text_y + line_height * line++, 18, WHITE);

            std::ostringstream primitive_position_stream;
            primitive_position_stream << std::fixed << std::setprecision(2)
                                      << "    Pos: x:" << primitive.getTransformationMatrix()(0, 3)
                                      << " y:" << primitive.getTransformationMatrix()(1, 3)
                                      << " z:" << primitive.getTransformationMatrix()(2, 3);
            std::string primitive_position_info = primitive_position_stream.str();
            DrawText(primitive_position_info.c_str(), text_x + 40, text_y + line_height * line++, 18, WHITE);
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

    void Simulator::updateRobotSettings(double smoothing_factor, double setting_difference_threshold)
    {
        // Update the robot settings based on the current target settings
        for (size_t i = 0; i < robot_.getJointSettings().size(); ++i)
        {
            double current_setting = robot_.getJointSettings()[i];
            double target_setting = target_robot_settings_[i];
            if (std::abs(current_setting - target_setting) > setting_difference_threshold)
            {
                current_setting += (target_setting - current_setting) * smoothing_factor; // Smooth interpolation
                robot_.setJointSetting(i, current_setting);
            }
        }
    }

    bool Simulator::robotIsMoving(double setting_difference_threshold) const
    {
        // Check if any of the settings are within the threshold of the target settings
        for (size_t i = 0; i < robot_.getJointSettings().size(); ++i)
        {
            if (std::abs(robot_.getJointSettings()[i] - target_robot_settings_[i]) > setting_difference_threshold)
                return true;
        }
        return false;
    }

    void Simulator::switchMode(int key)
    {
        if (IsKeyPressed(key) && IsCursorHidden())
        {
            // Toggle between individual joint control and end effector control
            switch (mode_)
            {
            case SimulatorMode::IndividualJointControl:
                mode_ = SimulatorMode::EndEffectorControl;

                // Reset target settings to current joint settings
                target_robot_settings_ = robot_.getJointSettings();
                target_position_ = robot_.getTransformationMatrices()[4].block<3, 1>(0, 3);

                break;
            case SimulatorMode::EndEffectorControl:
                mode_ = SimulatorMode::IndividualJointControl;
                break;
            default:
                mode_ = SimulatorMode::IndividualJointControl;
                break;
            }
        }
    }

    void Simulator::getPositionInput(Eigen::Vector3d &position, const double movement_increment)
    {
        // Keys 8, 2, 4, and 6 on the numpad to move  in the x and y plane
        if (IsKeyDown(KEY_KP_8))
            position[0] += movement_increment;
        if (IsKeyDown(KEY_KP_2))
            position[0] -= movement_increment;
        if (IsKeyDown(KEY_KP_4))
            position[1] += movement_increment;
        if (IsKeyDown(KEY_KP_6))
            position[1] -= movement_increment;

        // Keys 7 and 9 on the numpad to move up and down
        if (IsKeyDown(KEY_KP_7))
            position[2] += movement_increment;
        if (IsKeyDown(KEY_KP_9))
            position[2] -= movement_increment;
    }

    void Simulator::addPrimitive(const Eigen::Matrix4d &transformation_matrix, const PrimitiveType type, const double radius)
    {
        // Check if there is space to add a new primitive
        if (primitives_.size() >= max_primitives_)
        {
            std::cerr << "Maximum number of primitives reached. Cannot add more." << std::endl;
            return;
        }

        // Create a new primitive and add it to the list
        Primitive new_primitive(transformation_matrix, radius, RED, type);
        primitives_.push_back(new_primitive);
    }

    void Simulator::removePrimitive(size_t index)
    {
        if (index < 0 || index >= max_primitives_)
        {
            std::cerr << "Invalid primitive index: " << index << std::endl;
            return; // Invalid index, do nothing
        }

        // Remove the primitive at the specified index
        primitives_.erase(primitives_.begin() + index);
        if (selected_primitive_index_ >= static_cast<int>(primitives_.size()))
            selected_primitive_index_ = 0; // Reset selected index if it exceeds the current size
    }

    void Simulator::updatePrimitiveModels()
    {
        // Find picked up primitive if any and update its transformation matrix
        if (holding_primitive_ && selected_primitive_index_ < primitives_.size())
        {
            // Get the transformation matrix of the selected primitive
            const Primitive &selected_primitive = primitives_[selected_primitive_index_];
            Eigen::Matrix4d transformation_matrix = robot_.getTransformationMatrices().back();

            // Offset the transformation matrix to the radius of the primitive
            Eigen::Vector3d offset = -selected_primitive.getApproachVector(
                config::ROBOT_EE_APPROACH_HEIGHT, 0.0);
            Eigen::Matrix4d offset_matrix = Eigen::Matrix4d::Identity();
            offset_matrix.block<3, 1>(0, 3) = offset;

            // Apply the offset to the transformation matrix
            transformation_matrix = transformation_matrix * offset_matrix;

            // Set the updated transformation matrix back to the primitive
            primitives_[selected_primitive_index_].setTransformationMatrix(transformation_matrix);
        }

        if (!primitives_.empty())
        {
            // Reset the color of all primitives
            for (auto &primitive : primitives_)
            {
                primitive.setColor(RED); // Default color for unselected primitives
            }
            primitives_[selected_primitive_index_].setColor(GREEN); // Highlight the selected primitive
        }

        // Update the transformation matrices of the primitive models based on their current state
        for (size_t i = 0; i < primitives_.size(); ++i)
        {
            const auto &primitive = primitives_[i];
            switch (primitive.getType())
            {
            case PrimitiveType::Sphere:
                sphere_primitive_model_.global_transform = utils::toTransform(primitive.getTransformationMatrix());
                break;

            case PrimitiveType::Cube:
                cube_primitive_model_.global_transform = utils::toTransform(primitive.getTransformationMatrix());
                break;
            }
        }
    }
} // namespace robot_arm
