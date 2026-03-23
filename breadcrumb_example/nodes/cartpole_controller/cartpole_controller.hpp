#pragma once

#include <breadcrumb_example/cartpole_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace breadcrumb_example::cartpole_controller {

struct Session : CartpoleControllerSession<Session> {
    using CartpoleControllerSession::CartpoleControllerSession;

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

CallbackReturn on_configure(std::shared_ptr<Session> sn);

// IMPORTANT - this _must_ match the node name. Cake expects the node to be defined at pkg_name::node_name::NodeName
using CartpoleController = CartpoleControllerBase<Session, on_configure>;

} // namespace breadcrumb_example::cartpole_controller
