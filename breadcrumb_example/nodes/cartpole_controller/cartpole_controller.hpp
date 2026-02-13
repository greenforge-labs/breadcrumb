#pragma once

#include <breadcrumb_example/cartpole_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace breadcrumb_example::cartpole_controller {

struct Context : CartpoleControllerContext<Context> {
    // State variables (from joint_states)
    double cart_position = 0.0;
    double cart_velocity = 0.0;
    double pole_angle = 0.0;
    double pole_angular_velocity = 0.0;

    // Controller state
    bool enabled = true;         // Start enabled by default
    bool state_received = false; // Flag to check if we've received state yet
    bool emergency_stopped = false;

    // Position tracking setpoint
    double setpoint_position = 0.0; // Target cart position (default: origin)
    double goal_tolerance = 0.0;    // Tolerance for action completion
};

void init(std::shared_ptr<Context> ctx);

// IMPORTANT - this _must_ match the node name. Cake expects the node to be defined at pkg_name::node_name::NodeName
using CartpoleController = CartpoleControllerBase<Context, init>;

} // namespace breadcrumb_example::cartpole_controller
