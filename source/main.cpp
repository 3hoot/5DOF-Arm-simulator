#include "config.hpp"
#include "renderer.hpp"
#include "simulation.hpp"

int main()
{
    Config &config = Config::instance();
    Renderer &renderer = Renderer::instance(config.WINDOW_WIDTH, config.WINDOW_HEIGHT,
                                            config.WINDOW_TITLE, config.FPS, config.CAMERA_FOV);
    Simulation simulation(config, renderer);
    simulation.run();

    return 0;
}
