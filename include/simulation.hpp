#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <raylib.h>

#include "config.hpp"
#include "renderer.hpp"
#include "robot.hpp"
#include "simulation.hpp"

class Simulation
{
public:
    Simulation(Config &config, Renderer &renderer);
    void run();

private:
    Config &m_config;                              // Reference to the configuration for the simulation
    Renderer &m_renderer;                          // Reference to the renderer for drawing
    Robot m_robot;                                 // Robot instance for the simulation
    std::shared_ptr<spdlog::logger> m_main_logger; // Logger for the main simulation
};

#endif
