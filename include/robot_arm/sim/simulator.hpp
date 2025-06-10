#ifndef ROBOT_ARM_SIMULATOR_HPP
#define ROBOT_ARM_SIMULATOR_HPP

#include <vector>
#include <string_view>
#include <raylib.h>

#include "../core/robot.hpp"
#include "utils.hpp"
#include "primitive.hpp"

// All angles are in degrees [deg]
// All lengths are in centimeters [cm]

namespace robot_arm::sim
{
    enum class SimulatorMode
    {
        IndividualJointControl, // Control individual joints
        EndEffectorControl,     // Control the end effector directly
    };

    class Simulator
    {
    public:
        Simulator(core::Robot &robot);
        ~Simulator();

        // Runs the main loop of the simulator
        void run();

    private:
        // Raylib camera and configuration
        Camera3D camera_;
        SimulatorMode mode_;
        void switchMode(int key);

        // Drawing methods
        void drawRobot();
        void drawPrimitives();

        void drawInformation() const;
        void drawHelp() const;
        void drawDebug() const;

        // Robot instance to simulate
        core::Robot &robot_;
        std::vector<double> target_robot_settings_; // Target settings for the robot's joints
        Eigen::Vector3d target_position_;           // Target position for the end effector

        // Robot to model pairing (Joint index to model index mapping)
        std::vector<utils::FusionModel> robot_models_;
        size_t selected_joint_index_ = 0;

        // Updates the transformation matrices of the robot models based on the robot's current state
        void updateRobotModels();
        // Updates the robot settings based on the current target settings
        void updateRobotSettings(double smoothing_factor, double setting_difference_threshold);
        // Checks if any of the settings are within the threshold of the target settings
        bool robotIsMoving(double setting_difference_threshold) const;

        // Handles input for moving the given position in the simulator
        void getPositionInput(Eigen::Vector3d &position, const double movement_increment);

        // Primitives in the simulator
        std::vector<Primitive> primitives_;
        utils::FusionModel sphere_primitive_model_; // Primitive models for rendering primitive objects
        utils::FusionModel cube_primitive_model_;

        bool holding_primitive_ = false;   // Flag to indicate if a primitive is being held
        int max_primitives_;               // Maximum number of primitives allowed in the simulator
        int selected_primitive_index_ = 0; // Index of the currently selected primitive

        void addPrimitive(const Eigen::Matrix4d &transformation_matrix, const PrimitiveType type, const double radius);
        void removePrimitive(size_t index);
        void updatePrimitiveModels();
    };
}

#endif
