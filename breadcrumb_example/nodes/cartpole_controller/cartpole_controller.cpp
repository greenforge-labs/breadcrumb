#include "cartpole_controller.hpp"
#include <breadcrumb_example_interfaces/action/track_position.hpp>
#include <cake/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace breadcrumb_example::cartpole_controller {

// Helper function to print current feedback gains
void print_gains(std::shared_ptr<Session> sn) {
    RCLCPP_INFO(
        sn->node.get_logger(),
        "  Feedback Gains: K = [%.2f, %.2f, %.2f, %.2f]",
        sn->params.k1,
        sn->params.k2,
        sn->params.k3,
        sn->params.k4
    );
}

// Joint state callback - extract state from joint_states message
void joint_state_callback(std::shared_ptr<Session> sn, sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    // Look for cart_to_world and pole_to_cart joints
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "cart_to_world") {
            sn->cart_position = msg->position[i];
            if (i < msg->velocity.size()) {
                sn->cart_velocity = msg->velocity[i];
            }
        } else if (msg->name[i] == "pole_to_cart") {
            sn->pole_angle = msg->position[i];
            if (i < msg->velocity.size()) {
                sn->pole_angular_velocity = msg->velocity[i];
            }
        }
    }
    sn->state_received = true;
}

// Enable/disable service handler
void enable_handler(
    std::shared_ptr<Session> sn,
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response
) {
    sn->enabled = request->data;
    response->success = true;
    if (sn->enabled) {
        sn->emergency_stopped = false;
        response->message = "Controller enabled";
        RCLCPP_INFO(sn->node.get_logger(), "Controller enabled, emergency stop cleared");
    } else {
        response->message = "Controller disabled";
        RCLCPP_INFO(sn->node.get_logger(), "Controller disabled");
    }
}

// Emergency stop callback - triggered by any safety source
void emergency_stop_callback(std::shared_ptr<Session> sn, std_msgs::msg::Bool::ConstSharedPtr msg) {
    if (msg->data) {
        sn->emergency_stopped = true;
        sn->enabled = false;
        RCLCPP_WARN(sn->node.get_logger(), "Emergency stop triggered!");
    }
}

// Action goal tracking - check and update in control loop
void update_action_tracking(std::shared_ptr<Session> sn) {
    using TrackPosition = breadcrumb_example_interfaces::action::TrackPosition;

    auto active_goal = sn->actions.track_position->get_active_goal();

    if (active_goal) {
        // Update setpoint if we have a new goal
        if (sn->setpoint_position != active_goal->target_position ||
            sn->goal_tolerance != active_goal->goal_tolerance) {

            sn->setpoint_position = active_goal->target_position;
            sn->goal_tolerance = active_goal->goal_tolerance;

            RCLCPP_INFO(
                sn->node.get_logger(),
                "New position goal: %.2f m (tolerance: %.3f m)",
                sn->setpoint_position,
                sn->goal_tolerance
            );
        }

        // Compute distance to goal
        double distance_to_goal = std::abs(sn->cart_position - sn->setpoint_position);

        // Send feedback
        auto feedback = std::make_shared<TrackPosition::Feedback>();
        feedback->current_position = sn->cart_position;
        feedback->distance_to_goal = distance_to_goal;
        sn->actions.track_position->publish_feedback(feedback);

        // Check if goal is reached
        if (distance_to_goal < sn->goal_tolerance) {
            auto result = std::make_shared<TrackPosition::Result>();
            result->success = true;
            result->final_position = sn->cart_position;
            sn->actions.track_position->succeed(result);

            RCLCPP_INFO(sn->node.get_logger(), "Position goal reached! Final: %.2f m", sn->cart_position);
        }
    }
}

// State feedback control timer callback
void control_timer_callback(std::shared_ptr<Session> sn) {
    // Update parameters if the are out of date
    if (sn->param_listener->is_old(sn->params)) {
        sn->params = sn->param_listener->get_params();
        print_gains(sn);
    }

    // Update action tracking (check for goals, send feedback, complete if reached)
    update_action_tracking(sn);

    // Publish status message
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "{\"enabled\":" + std::string(sn->enabled ? "true" : "false") +
                      ",\"emergency_stopped\":" + std::string(sn->emergency_stopped ? "true" : "false") + "}";
    sn->publishers.status->publish(status_msg);

    // Don't publish if controller is disabled or we haven't received state yet
    if (!sn->enabled || !sn->state_received) {
        return;
    }

    // Full state feedback with setpoint tracking: u = -K * (x - x_desired)
    // State vector: x = [cart_position, cart_velocity, pole_angle, pole_angular_velocity]
    // Setpoint: [setpoint_position, 0, 0, 0] (track position, keep pole upright)
    // Control law: u = -(k1*(x - x_setpoint) + k2*xdot + k3*theta + k4*thetadot)
    const double k1 = sn->params.k1;
    const double k2 = sn->params.k2;
    const double k3 = sn->params.k3;
    const double k4 = sn->params.k4;

    // Compute position error for setpoint tracking
    const double position_error = sn->cart_position - sn->setpoint_position;

    const double control_force =
        -(k1 * position_error + k2 * sn->cart_velocity + k3 * sn->pole_angle + k4 * sn->pole_angular_velocity);

    // Publish control force
    auto force_msg = std_msgs::msg::Float64();
    force_msg.data = control_force;
    sn->publishers.force->publish(force_msg);
}

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    // Set up joint state subscriber
    sn->subscribers.joint_states->set_callback(joint_state_callback);

    // Set up enable/disable service
    sn->services.enable->set_request_handler(enable_handler);

    // Set up position tracking action server options
    cake::SingleGoalActionServerOptions<breadcrumb_example_interfaces::action::TrackPosition> action_options;
    action_options.new_goals_replace_current_goal = true; // Allow updating the setpoint
    sn->actions.track_position->set_options(action_options);

    // Set up emergency stop subscribers (for_each_param creates one per safety source)
    for (auto &[source, sub] : sn->subscribers.emergency_stops) {
        sub->set_callback(emergency_stop_callback);
    }

    // Create control timer (50 Hz to match simulator)
    cake::create_timer(sn, std::chrono::milliseconds(20), control_timer_callback);

    RCLCPP_INFO(sn->node.get_logger(), "State Feedback Cartpole Controller initialized");
    print_gains(sn);
    RCLCPP_INFO(sn->node.get_logger(), "  Controller state: %s", sn->enabled ? "ENABLED" : "DISABLED");

    return CallbackReturn::SUCCESS;
}

} // namespace breadcrumb_example::cartpole_controller
