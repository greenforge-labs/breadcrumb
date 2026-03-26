#include "cartpole_simulator.hpp"
#include <jig/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace breadcrumb_example::cartpole_simulator {

// Force subscriber callback
void force_callback(std::shared_ptr<Session> sn, std_msgs::msg::Float64::ConstSharedPtr msg) {
    sn->applied_force = msg->data;
}

// Reset service handler
void reset_handler(
    std::shared_ptr<Session> sn,
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response
) {
    sn->reset_state();
    response->success = true;
    response->message = "Cartpole reset to initial conditions";
    RCLCPP_INFO(sn->node.get_logger(), "Cartpole reset");
}

// Physics simulation using classic cartpole equations
void update_physics(std::shared_ptr<Session> sn) {
    const double &F = sn->applied_force;
    const double &x_dot = sn->cart_velocity;
    const double &theta = sn->pole_angle;
    const double &theta_dot = sn->pole_angular_velocity;

    const double mc = Session::CART_MASS;
    const double mp = Session::POLE_MASS;
    const double L = Session::POLE_LENGTH / 2.0; // Half-length to center of mass
    const double g = Session::GRAVITY;
    const double b = Session::FRICTION;
    const double dt = Session::DT;

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
    sn->cart_velocity += x_ddot * dt;
    sn->cart_position += sn->cart_velocity * dt;
    sn->pole_angular_velocity += theta_ddot * dt;
    sn->pole_angle += sn->pole_angular_velocity * dt;

    // Normalize pole angle to [-pi, pi]
    while (sn->pole_angle > M_PI)
        sn->pole_angle -= 2.0 * M_PI;
    while (sn->pole_angle < -M_PI)
        sn->pole_angle += 2.0 * M_PI;
}

// Publish joint states
void publish_state(std::shared_ptr<Session> sn) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = sn->node.now();

    msg.name = {"cart_to_world", "pole_to_cart"};
    msg.position = {sn->cart_position, sn->pole_angle};
    msg.velocity = {sn->cart_velocity, sn->pole_angular_velocity};

    sn->publishers.joint_states->publish(msg);
}

// Timer callback for simulation loop
void simulation_timer_callback(std::shared_ptr<Session> sn) {
    update_physics(sn);
    publish_state(sn);
}

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    // Set up force subscriber
    sn->subscribers.requested_force->set_callback(force_callback);

    // Set up reset service
    sn->services.reset->set_request_handler(reset_handler);

    // Create simulation timer (50 Hz)
    jig::create_timer(sn, std::chrono::milliseconds(20), simulation_timer_callback);

    RCLCPP_INFO(sn->node.get_logger(), "Cartpole simulator initialized");
    RCLCPP_INFO(sn->node.get_logger(), "  Cart mass: %.2f kg", Session::CART_MASS);
    RCLCPP_INFO(sn->node.get_logger(), "  Pole mass: %.2f kg", Session::POLE_MASS);
    RCLCPP_INFO(sn->node.get_logger(), "  Pole length: %.2f m", Session::POLE_LENGTH);
    RCLCPP_INFO(sn->node.get_logger(), "  Simulation rate: %.1f Hz", 1.0 / Session::DT);

    return CallbackReturn::SUCCESS;
}

} // namespace breadcrumb_example::cartpole_simulator
