#include "cartpole_controller.hpp"
#include <cake/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
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
        response->message = "Controller enabled";
        RCLCPP_INFO(ctx->node->get_logger(), "Controller enabled");
    } else {
        response->message = "Controller disabled";
        RCLCPP_INFO(ctx->node->get_logger(), "Controller disabled");
    }
}

// State feedback control timer callback
void control_timer_callback(std::shared_ptr<Context> ctx) {
    // Update parameters if the are out of date
    if (ctx->param_listener->is_old(ctx->params)) {
        ctx->params = ctx->param_listener->get_params();
        print_gains(ctx);
    }

    // Don't publish if controller is disabled or we haven't received state yet
    if (!ctx->enabled || !ctx->state_received) {
        return;
    }

    // Full state feedback: u = -K * x
    // State vector: x = [cart_position, cart_velocity, pole_angle, pole_angular_velocity]
    // Control law: u = -(k1*x1 + k2*x2 + k3*x3 + k4*x4)
    const double k1 = ctx->params.k1;
    const double k2 = ctx->params.k2;
    const double k3 = ctx->params.k3;
    const double k4 = ctx->params.k4;

    const double control_force =
        -(k1 * ctx->cart_position + k2 * ctx->cart_velocity + k3 * ctx->pole_angle + k4 * ctx->pole_angular_velocity);

    // Publish control force
    auto msg = std_msgs::msg::Float64();
    msg.data = control_force;
    ctx->publishers.force->publish(msg);
}

void init(std::shared_ptr<Context> ctx) {
    // Set up joint state subscriber
    ctx->subscribers.joint_states->set_callback(joint_state_callback);

    // Set up enable/disable service
    ctx->services.enable->set_request_handler(enable_handler);

    // Create control timer (50 Hz to match simulator)
    cake::create_timer(ctx, std::chrono::milliseconds(20), control_timer_callback);

    RCLCPP_INFO(ctx->node->get_logger(), "State Feedback Cartpole Controller initialized");
    print_gains(ctx);
    RCLCPP_INFO(ctx->node->get_logger(), "  Controller state: %s", ctx->enabled ? "ENABLED" : "DISABLED");
}

} // namespace breadcrumb_example::cartpole_controller
