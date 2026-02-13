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
void print_gains(std::shared_ptr<Context> ctx) {
    RCLCPP_INFO(
        ctx->node->get_logger(),
        "  Feedback Gains: K = [%.2f, %.2f, %.2f, %.2f]",
        ctx->params.k1,
        ctx->params.k2,
        ctx->params.k3,
        ctx->params.k4
    );
}

// Joint state callback - extract state from joint_states message
void joint_state_callback(std::shared_ptr<Context> ctx, sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    // Look for cart_to_world and pole_to_cart joints
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "cart_to_world") {
            ctx->cart_position = msg->position[i];
            if (i < msg->velocity.size()) {
                ctx->cart_velocity = msg->velocity[i];
            }
        } else if (msg->name[i] == "pole_to_cart") {
            ctx->pole_angle = msg->position[i];
            if (i < msg->velocity.size()) {
                ctx->pole_angular_velocity = msg->velocity[i];
            }
        }
    }
    ctx->state_received = true;
}

// Enable/disable service handler
void enable_handler(
    std::shared_ptr<Context> ctx,
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response
) {
    ctx->enabled = request->data;
    response->success = true;
    if (ctx->enabled) {
        ctx->emergency_stopped = false;
        response->message = "Controller enabled";
        RCLCPP_INFO(ctx->node->get_logger(), "Controller enabled, emergency stop cleared");
    } else {
        response->message = "Controller disabled";
        RCLCPP_INFO(ctx->node->get_logger(), "Controller disabled");
    }
}

// Emergency stop callback - triggered by any safety source
void emergency_stop_callback(std::shared_ptr<Context> ctx, std_msgs::msg::Bool::ConstSharedPtr msg) {
    if (msg->data) {
        ctx->emergency_stopped = true;
        ctx->enabled = false;
        RCLCPP_WARN(ctx->node->get_logger(), "Emergency stop triggered!");
    }
}

// Action goal tracking - check and update in control loop
void update_action_tracking(std::shared_ptr<Context> ctx) {
    using TrackPosition = breadcrumb_example_interfaces::action::TrackPosition;

    auto active_goal = ctx->actions.track_position->get_active_goal();

    if (active_goal) {
        // Update setpoint if we have a new goal
        if (ctx->setpoint_position != active_goal->target_position ||
            ctx->goal_tolerance != active_goal->goal_tolerance) {

            ctx->setpoint_position = active_goal->target_position;
            ctx->goal_tolerance = active_goal->goal_tolerance;

            RCLCPP_INFO(
                ctx->node->get_logger(),
                "New position goal: %.2f m (tolerance: %.3f m)",
                ctx->setpoint_position,
                ctx->goal_tolerance
            );
        }

        // Compute distance to goal
        double distance_to_goal = std::abs(ctx->cart_position - ctx->setpoint_position);

        // Send feedback
        auto feedback = std::make_shared<TrackPosition::Feedback>();
        feedback->current_position = ctx->cart_position;
        feedback->distance_to_goal = distance_to_goal;
        ctx->actions.track_position->publish_feedback(feedback);

        // Check if goal is reached
        if (distance_to_goal < ctx->goal_tolerance) {
            auto result = std::make_shared<TrackPosition::Result>();
            result->success = true;
            result->final_position = ctx->cart_position;
            ctx->actions.track_position->succeed(result);

            RCLCPP_INFO(ctx->node->get_logger(), "Position goal reached! Final: %.2f m", ctx->cart_position);
        }
    }
}

// State feedback control timer callback
void control_timer_callback(std::shared_ptr<Context> ctx) {
    // Update parameters if the are out of date
    if (ctx->param_listener->is_old(ctx->params)) {
        ctx->params = ctx->param_listener->get_params();
        print_gains(ctx);
    }

    // Update action tracking (check for goals, send feedback, complete if reached)
    update_action_tracking(ctx);

    // Publish status message
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "{\"enabled\":" + std::string(ctx->enabled ? "true" : "false") +
                      ",\"emergency_stopped\":" + std::string(ctx->emergency_stopped ? "true" : "false") + "}";
    ctx->publishers.status->publish(status_msg);

    // Don't publish if controller is disabled or we haven't received state yet
    if (!ctx->enabled || !ctx->state_received) {
        return;
    }

    // Full state feedback with setpoint tracking: u = -K * (x - x_desired)
    // State vector: x = [cart_position, cart_velocity, pole_angle, pole_angular_velocity]
    // Setpoint: [setpoint_position, 0, 0, 0] (track position, keep pole upright)
    // Control law: u = -(k1*(x - x_setpoint) + k2*xdot + k3*theta + k4*thetadot)
    const double k1 = ctx->params.k1;
    const double k2 = ctx->params.k2;
    const double k3 = ctx->params.k3;
    const double k4 = ctx->params.k4;

    // Compute position error for setpoint tracking
    const double position_error = ctx->cart_position - ctx->setpoint_position;

    const double control_force =
        -(k1 * position_error + k2 * ctx->cart_velocity + k3 * ctx->pole_angle + k4 * ctx->pole_angular_velocity);

    // Publish control force
    auto force_msg = std_msgs::msg::Float64();
    force_msg.data = control_force;
    ctx->publishers.force->publish(force_msg);
}

void init(std::shared_ptr<Context> ctx) {
    // Set up joint state subscriber
    ctx->subscribers.joint_states->set_callback(joint_state_callback);

    // Set up enable/disable service
    ctx->services.enable->set_request_handler(enable_handler);

    // Set up position tracking action server options
    cake::SingleGoalActionServerOptions<breadcrumb_example_interfaces::action::TrackPosition> action_options;
    action_options.new_goals_replace_current_goal = true; // Allow updating the setpoint
    ctx->actions.track_position->set_options(action_options);

    // Set up emergency stop subscribers (for_each_param creates one per safety source)
    for (auto &[source, sub] : ctx->subscribers.emergency_stops) {
        sub->set_callback(emergency_stop_callback);
    }

    // Create control timer (50 Hz to match simulator)
    cake::create_timer(ctx, std::chrono::milliseconds(20), control_timer_callback);

    RCLCPP_INFO(ctx->node->get_logger(), "State Feedback Cartpole Controller initialized");
    print_gains(ctx);
    RCLCPP_INFO(ctx->node->get_logger(), "  Controller state: %s", ctx->enabled ? "ENABLED" : "DISABLED");
}

} // namespace breadcrumb_example::cartpole_controller
