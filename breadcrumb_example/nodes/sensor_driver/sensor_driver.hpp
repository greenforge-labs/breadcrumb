#pragma once

#include <breadcrumb_example/sensor_driver_interface.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace breadcrumb_example::sensor_driver {

// Extend the generated context with custom state
struct Context : SensorDriverContext<Context> {
    double simulated_temperature = 20.0;
};

// Forward declare init function
void init(std::shared_ptr<Context> ctx);

// Define the node class using the generated base
using SensorDriver = SensorDriverBase<Context, init>;

} // namespace breadcrumb_example::sensor_driver
