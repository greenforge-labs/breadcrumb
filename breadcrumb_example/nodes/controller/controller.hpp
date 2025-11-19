#pragma once

#include <memory>
#include <breadcrumb_example/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace breadcrumb_example::controller {

// Extend the generated context with custom state
struct Context : ControllerContext<Context> {
    rclcpp::TimerBase::SharedPtr timer;
    int navigation_progress = 0;
};

// Forward declare init function
void init(std::shared_ptr<Context> ctx);

// Define the node class using the generated base
using Controller = ControllerBase<Context, init>;

} // namespace breadcrumb_example::controller
