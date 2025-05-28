#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <raylib.h>
#include <iostream>
#include <algorithm>

#include "config.hpp"
#include "renderer.hpp"
#include "robot.hpp"
#include "simulation.hpp"

Simulation::Simulation(Config &config, Renderer &renderer)
    : m_config(config), m_renderer(renderer),
      m_robot(config.DH_PARAMS, config.JOINT_LIMITS)
{
    auto m_main_logger = spdlog::stdout_color_mt("main_logger");
    spdlog::info("Successfully initialized the Arm simulator logger.");

    spdlog::info("Robot initialized with (modified) Denavit-Hartenberg parameters.");

    spdlog::info("Simulation initialized with configuration: "
                 "Window size: {}x{}, Title: {}, FPS: {}, Camera FOV: {}",
                 m_config.WINDOW_WIDTH, m_config.WINDOW_HEIGHT,
                 m_config.WINDOW_TITLE, m_config.FPS, m_config.CAMERA_FOV);
}

void Simulation::run()
{
    // Main simulation loop
    size_t selected_joint = 0;   // Default selected joint for input
    double selected_angle = 0.0; // Default angle for input
    while (m_renderer.isOpen())
    {
        auto setAngle = [](size_t joint_index, double angle_rad, Robot &robot)
        {
            try
            {
                robot.setJointAngle(joint_index, angle_rad);
            }
            catch (const std::exception &e)
            {
                spdlog::error("Error setting joint angle: {}", e.what());
            }
        };

        // Input handling ----------------------------------------------------------------
        // TODO: rework later to use a more sophisticated input handling system

        // Keyboard input handling
        if (IsKeyPressed(KEY_ESCAPE))
            break;
        if (IsKeyPressed(KEY_L)) // Modify link lengths parameters
        {
            EnableCursor(); // Ensure cursor is enabled for input
            int link;
            double value;
            std::cout << "Enter link index (1-5) and new length value: ";
            std::cin >> link >> value;

            try
            {
                switch (link)
                {
                case 1:
                    m_robot.modifyDHParams(0, 2, value); // Link 1
                    break;
                case 2:
                    m_robot.modifyDHParams(1, 0, value); // Link 2
                    break;
                case 3:
                    m_robot.modifyDHParams(2, 0, value); // Link 3
                    break;
                case 4:
                    m_robot.modifyDHParams(3, 0, value); // Link 4
                    break;
                case 5:
                    m_robot.modifyDHParams(5, 2, value); // Link 5
                }
            }
            catch (const std::exception &e)
            {
                spdlog::error("Error modifying link lengths: {}", e.what());
            }
        }
        if (IsKeyPressed(KEY_R)) // Reset robot to initial state
        {
            m_robot = Robot(m_config.DH_PARAMS, m_config.JOINT_LIMITS);
            spdlog::info("Robot reset to initial state.");
        }
        if (IsKeyPressed(KEY_KP_8)) // Switch joint selection
        {
            // Loop through joints, excluding the end-effector
            selected_joint = (selected_joint + 1) % (m_robot.getJointCount() - 1);
            selected_angle = m_robot.getJointAngle(selected_joint); // Reset angle to current joint angle
            spdlog::info("Selected joint: {}", selected_joint);
        }
        if (IsKeyPressed(KEY_KP_2)) // Switch joint selection
        {
            // Wrap around the joints, excluding the end-effector
            selected_joint = (selected_joint - 1 + m_robot.getJointCount() - 1) % (m_robot.getJointCount() - 1);
            selected_angle = m_robot.getJointAngle(selected_joint); // Reset angle to current joint angle
            spdlog::info("Selected joint: {}", selected_joint);
        }
        if (IsKeyDown(KEY_KP_6)) // Increase joint angle
        {
            // Increment by 2.5 degrees and clamp to max limit
            selected_angle = std::min(selected_angle + M_PI / 72, m_config.JOINT_LIMITS[selected_joint].second);
            spdlog::info("Selected joint {} angle increased to: {:.2f} radians", selected_joint, selected_angle);

            setAngle(selected_joint, selected_angle, m_robot);
        }
        if (IsKeyDown(KEY_KP_4)) // Decrease joint angle
        {
            // Decrement by 2.5 degrees and clamp to min limit
            selected_angle = std::max(selected_angle - M_PI / 72, m_config.JOINT_LIMITS[selected_joint].first);
            spdlog::info("Selected joint {} angle decreased to: {:.2f} radians", selected_joint, selected_angle);

            setAngle(selected_joint, selected_angle, m_robot);
        }

        // Mouse input handling
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
            IsCursorHidden() ? EnableCursor() : DisableCursor();
        if (IsCursorHidden())
            m_renderer.updateCamera(CAMERA_FIRST_PERSON);

        // -------------------------------------------------------------------------------

        // Drawing -----------------------------------------------------------------------
        m_renderer.beginDrawing();

        for (int i = 0; i < m_robot.getJointCount(); ++i)
            m_renderer.drawJoint(i, m_robot);

        m_renderer.endDrawing();
        // -------------------------------------------------------------------------------
    }

    spdlog::info("Closed the Arm simulator. Exiting...");
}
