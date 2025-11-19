#include "cartpole_simulator.hpp"
#include <cake/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace breadcrumb_example::cartpole_simulator {

// Force subscriber callback
void force_callback(std::shared_ptr<Context> ctx, std_msgs::msg::Float64::ConstSharedPtr msg) {
    ctx->applied_force = msg->data;
}

// Reset service handler
void reset_handler(
    std::shared_ptr<Context> ctx,
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response
) {
    ctx->reset_state();
    response->success = true;
    response->message = "Cartpole reset to initial conditions";
    RCLCPP_INFO(ctx->node->get_logger(), "Cartpole reset");
}

// Physics simulation using classic cartpole equations
void update_physics(std::shared_ptr<Context> ctx) {
    const double &F = ctx->applied_force;
    const double &x_dot = ctx->cart_velocity;
    const double &theta = ctx->pole_angle;
    const double &theta_dot = ctx->pole_angular_velocity;

    const double mc = Context::CART_MASS;
    const double mp = Context::POLE_MASS;
    const double L = Context::POLE_LENGTH / 2.0; // Half-length to center of mass
    const double g = Context::GRAVITY;
    const double b = Context::FRICTION;
    const double dt = Context::DT;

    // Compute total mass and pole mass times length
    const double total_mass = mc + mp;
    const double pole_mass_length = mp * L;

    // Temporary variables for the dynamics
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double theta_dot_sq = theta_dot * theta_dot;

    // Cartpole dynamics (from standard equations of motion)
    // x_ddot = (F + pole_mass_length * theta_dot^2 * sin(theta) - mp * g * sin(theta) * cos(theta) - b * x_dot)
    //          / (total_mass - mp * cos^2(theta))
    const double temp = (F + pole_mass_length * theta_dot_sq * sin_theta - b * x_dot) / total_mass;
    const double theta_ddot =
        (g * sin_theta - cos_theta * temp) / (L * (4.0 / 3.0 - mp * cos_theta * cos_theta / total_mass));
    const double x_ddot = temp - pole_mass_length * theta_ddot * cos_theta / total_mass;

    // Euler integration
    ctx->cart_velocity += x_ddot * dt;
    ctx->cart_position += ctx->cart_velocity * dt;
    ctx->pole_angular_velocity += theta_ddot * dt;
    ctx->pole_angle += ctx->pole_angular_velocity * dt;

    // Enforce cart position hard stops (matches URDF joint limits)
    const double limit = Context::CART_POSITION_LIMIT;
    if (ctx->cart_position > limit) {
        ctx->cart_position = limit;
        ctx->cart_velocity = std::min(0.0, ctx->cart_velocity); // Only allow negative velocity
    } else if (ctx->cart_position < -limit) {
        ctx->cart_position = -limit;
        ctx->cart_velocity = std::max(0.0, ctx->cart_velocity); // Only allow positive velocity
    }

    // Normalize pole angle to [-pi, pi]
    while (ctx->pole_angle > M_PI)
        ctx->pole_angle -= 2.0 * M_PI;
    while (ctx->pole_angle < -M_PI)
        ctx->pole_angle += 2.0 * M_PI;
}

// Publish joint states
void publish_state(std::shared_ptr<Context> ctx) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = ctx->node->now();

    msg.name = {"cart_to_world", "pole_to_cart"};
    msg.position = {ctx->cart_position, ctx->pole_angle};
    msg.velocity = {ctx->cart_velocity, ctx->pole_angular_velocity};

    ctx->publishers.joint_states->publish(msg);
}

// Timer callback for simulation loop
void simulation_timer_callback(std::shared_ptr<Context> ctx) {
    update_physics(ctx);
    publish_state(ctx);
}

void init(std::shared_ptr<Context> ctx) {
    // Set up force subscriber
    ctx->subscribers.cartpole_force->set_callback(force_callback);

    // Set up reset service
    ctx->services.reset->set_request_handler(reset_handler);

    // Create simulation timer (50 Hz)
    cake::create_timer(ctx, std::chrono::milliseconds(20), simulation_timer_callback);

    RCLCPP_INFO(ctx->node->get_logger(), "Cartpole simulator initialized");
    RCLCPP_INFO(ctx->node->get_logger(), "  Cart mass: %.2f kg", Context::CART_MASS);
    RCLCPP_INFO(ctx->node->get_logger(), "  Pole mass: %.2f kg", Context::POLE_MASS);
    RCLCPP_INFO(ctx->node->get_logger(), "  Pole length: %.2f m", Context::POLE_LENGTH);
    RCLCPP_INFO(ctx->node->get_logger(), "  Simulation rate: %.1f Hz", 1.0 / Context::DT);
}

} // namespace breadcrumb_example::cartpole_simulator
