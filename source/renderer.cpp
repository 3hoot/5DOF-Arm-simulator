#include <spdlog/spdlog.h>
#include <raylib.h>
#include <Eigen/Dense>
#include <rlgl.h>

#include "config.hpp"
#include "renderer.hpp"
#include "robot.hpp"
#include "utils.hpp"

Renderer &Renderer::instance(int width, int height, const char *title, int FPS, float camera_fov)
{
    static Renderer instance;
    spdlog::info("Initializing Renderer with width: {}, height: {}, title: {}, FPS: {}", width, height, title, FPS);

    // Initialize the window
    InitWindow(width, height, title);
    SetTargetFPS(FPS); // Set the target frames per second
    spdlog::info("Initialized renderer with window size: {}x{}", width, height);

    // Set up the camera
    m_camera.position = {5.0f, 5.0f, 5.0f};   // Set camera position
    m_camera.target = {0.0f, 0.0f, 0.0f};     // Set camera target
    m_camera.up = {0.0f, 1.0f, 0.0f};         // Set camera up vector
    m_camera.fovy = camera_fov;               // Set camera field of view
    m_camera.projection = CAMERA_PERSPECTIVE; // Set camera projection type
    spdlog::info("Initialized camera with position: ({}, {}, {})", m_camera.position.x, m_camera.position.y, m_camera.position.z);

    return instance;
}

Renderer &Renderer::instance()
{
    Config &config = Config::instance();
    return instance(config.WINDOW_WIDTH, config.WINDOW_HEIGHT, config.WINDOW_TITLE);
}

Camera Renderer::m_camera; // Initialize the static camera member

const bool Renderer::isOpen() const
{
    return !WindowShouldClose();
}

void Renderer::beginDrawing() const
{
    if (isOpen())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE); // Clear the background with a white color
        BeginMode3D(m_camera);     // Begin 3D mode with the camera
        DrawGrid(10, 1.0f);        // Draw a grid in the 3D space
    }
}

void Renderer::endDrawing() const
{
    if (isOpen())
    {
        EndMode3D();  // End 3D mode
        EndDrawing(); // Finish drawing
    }
}

void Renderer::drawJoint(int joint_index, const Robot &robot) const
{
    if (!isOpen())
        return;

    // Placeholder for actual joint model drawing
    // For now draws a cube at the joint position
    const Vector3 joint_position = utils::toPosition(robot.getJointPosition(joint_index));

    // Use RED for the end-effector joint, BLUE for others
    Color joint_color = (joint_index == robot.getJointCount() - 1) ? RED : BLUE;
    DrawCube(joint_position, 0.5f, 0.5f, 0.5f, joint_color); // Draw a cube at the joint position
}

// TODO: Rework updateCamera
void Renderer::updateCamera(int mode)
{
    if (isOpen())
    {
        UpdateCamera(&m_camera, mode); // Update the camera based on the mode
    }
}

Renderer::~Renderer()
{
    CloseWindow();
    spdlog::info("Renderer resources unloaded and window closed.");
}
