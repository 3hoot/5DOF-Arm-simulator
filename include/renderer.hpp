#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <raylib.h>

#include "robot.hpp"

// ------ Renderer singleton class for the project ------
class Renderer
{
public:
    // Delete copy/move constructor and assignment operator to enforce singleton pattern
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;
    Renderer(Renderer &&) = delete;
    Renderer &operator=(Renderer &&) = delete;

    // ------ Rendering methods ------
    static Renderer &instance();
    static Renderer &instance(int width, int height, const char *title, int FPS = 60,
                              float camera_fov = 45.0f);
    void beginDrawing() const;
    void endDrawing() const;
    void drawJoint(int joint_index, const Robot &robot) const;
    const bool isOpen() const;
    void Renderer::updateCamera(int mode);

    static Camera m_camera; // Camera for the 3D scene

private:
    // Private constructor to prevent instantiation
    Renderer() = default;
    ~Renderer();
};

#endif
