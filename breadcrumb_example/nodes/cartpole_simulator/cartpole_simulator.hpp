#pragma once

#include <breadcrumb_example/cartpole_simulator_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace breadcrumb_example::cartpole_simulator {

struct Context : CartpoleSimulatorContext<Context> {
    // Physics state
    double cart_position = 0.0;         // meters
    double cart_velocity = 0.0;         // m/s
    double pole_angle = 0.1;            // radians (start slightly off vertical)
    double pole_angular_velocity = 0.0; // rad/s

    // Control input
    double applied_force = 0.0; // Newtons

    // Physics parameters (hardcoded)
    static constexpr double CART_MASS = 1.0;   // kg
    static constexpr double POLE_MASS = 0.1;   // kg
    static constexpr double POLE_LENGTH = 1.0; // meters (full length)
    static constexpr double GRAVITY = 9.81;    // m/s^2
    static constexpr double FRICTION = 0.1;    // damping coefficient
    static constexpr double DT = 0.02;         // timestep (50 Hz)

    // Helper method to reset state
    void reset_state() {
        cart_position = 0.0;
        cart_velocity = 0.0;
        pole_angle = 0.1; // Small initial angle
        pole_angular_velocity = 0.0;
        applied_force = 0.0;
    }
};

void init(std::shared_ptr<Context> ctx);

// IMPORTANT - this _must_ match the node name. Cake expects the node to be defined at pkg_name::node_name::NodeName
using CartpoleSimulator = CartpoleSimulatorBase<Context, init>;

} // namespace breadcrumb_example::cartpole_simulator
