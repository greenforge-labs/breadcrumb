#include "controller.hpp"
#include <cake/timer.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;

namespace breadcrumb_example::controller {

void navigation_callback(std::shared_ptr<Context> ctx) {
    // Check for active navigation goal
    auto active_goal = ctx->actions.navigate_to_pose->get_active_goal();

    if (!active_goal) {
        // No active goal, reset progress
        ctx->navigation_progress = 0;

        // Stop movement
        geometry_msgs::msg::Twist stop_cmd;
        ctx->publishers.cmd_vel->publish(stop_cmd);
        return;
    }

    // We have an active goal - simulate navigation progress
    RCLCPP_INFO(
        ctx->node->get_logger(),
        "Navigating to pose (%.2f, %.2f, %.2f) - Progress: %d/10",
        active_goal->pose.pose.position.x,
        active_goal->pose.pose.position.y,
        active_goal->pose.pose.position.z,
        ctx->navigation_progress
    );

    // Increment progress
    ctx->navigation_progress++;

    // Publish velocity command (simulate movement)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = ctx->params.max_speed * 0.5;
    ctx->publishers.cmd_vel->publish(cmd);

    // Publish feedback
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    feedback->distance_remaining = 10.0 - ctx->navigation_progress;
    ctx->actions.navigate_to_pose->publish_feedback(feedback);

    // Check if navigation is complete
    if (ctx->navigation_progress >= 10) {
        // Navigation complete - succeed
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        result->error_code = 0; // Success
        ctx->actions.navigate_to_pose->succeed(result);

        RCLCPP_INFO(ctx->node->get_logger(), "Navigation completed successfully!");

        // Stop movement
        geometry_msgs::msg::Twist stop_cmd;
        ctx->publishers.cmd_vel->publish(stop_cmd);

        // Reset progress
        ctx->navigation_progress = 0;
    }
}

void init(std::shared_ptr<Context> ctx) {
    RCLCPP_INFO(ctx->node->get_logger(), "Initializing controller");
    RCLCPP_INFO(ctx->node->get_logger(), "Max speed: %.2f m/s", ctx->params.max_speed);

    // Configure action server to replace current goals with new ones
    ctx->actions.navigate_to_pose->set_options({.new_goals_replace_current_goal = true});

    // Create timer to periodically check for and process navigation goals
    cake::create_timer(ctx, 500ms, navigation_callback);

    RCLCPP_INFO(ctx->node->get_logger(), "Controller ready");
}

} // namespace breadcrumb_example::controller
