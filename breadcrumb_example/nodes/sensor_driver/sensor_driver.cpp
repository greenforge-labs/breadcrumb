#include "sensor_driver.hpp"
#include <cmath>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>

#include <cake/timer.hpp>

namespace breadcrumb_example::sensor_driver {

void timer_callback(std::shared_ptr<Context> ctx) {
    // Simulate temperature readings with some variation
    ctx->simulated_temperature = 20.0 + 5.0 * std::sin(ctx->node->now().seconds() * 0.5);

    // Publish temperature data
    sensor_msgs::msg::Temperature temp_msg;
    temp_msg.header.stamp = ctx->node->now();
    temp_msg.header.frame_id = ctx->params.frame_id;
    temp_msg.temperature = ctx->simulated_temperature;
    temp_msg.variance = 0.1;

    ctx->publishers.sensor_data->publish(temp_msg);

    // Publish status
    std_msgs::msg::String status_msg;
    status_msg.data = "Sensor " + ctx->params.sensor_id + " operating normally";
    ctx->publishers.sensor_status->publish(status_msg);

    RCLCPP_DEBUG(ctx->node->get_logger(), "Published temperature: %.2f°C", ctx->simulated_temperature);
}

void init(std::shared_ptr<Context> ctx) {
    RCLCPP_INFO(ctx->node->get_logger(), "Initializing sensor driver: %s", ctx->params.sensor_id.c_str());
    RCLCPP_INFO(ctx->node->get_logger(), "Update rate: %.1f Hz", ctx->params.update_rate);

    // Create timer based on update_rate parameter
    auto period = std::chrono::duration<double>(1.0 / ctx->params.update_rate);
    cake::create_timer(ctx, period, timer_callback);
}

} // namespace breadcrumb_example::sensor_driver
