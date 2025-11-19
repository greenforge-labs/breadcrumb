#include "data_processor.hpp"
#include <example_interfaces/srv/trigger.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>

namespace breadcrumb_example::data_processor {

void sensor_data_callback(std::shared_ptr<Context> ctx, sensor_msgs::msg::Temperature::ConstSharedPtr msg) {
    if (!ctx->params.processing_enabled) {
        return;
    }

    // Update statistics
    ctx->sample_count++;
    ctx->sum_temp += msg->temperature;
    ctx->min_temp = std::min(ctx->min_temp, msg->temperature);
    ctx->max_temp = std::max(ctx->max_temp, msg->temperature);

    // Keep last 100 samples
    ctx->temperature_history.push_back(msg->temperature);
    if (ctx->temperature_history.size() > 100) {
        ctx->temperature_history.erase(ctx->temperature_history.begin());
    }

    // Publish processed data (copy with additional processing flag)
    sensor_msgs::msg::Temperature processed_msg = *msg;
    ctx->publishers.processed_data->publish(processed_msg);

    // Check threshold and publish alert if needed
    if (msg->temperature > ctx->params.threshold) {
        std_msgs::msg::String alert_msg;
        alert_msg.data = "Temperature " + std::to_string(msg->temperature) + "°C exceeds threshold " +
                         std::to_string(ctx->params.threshold) + "°C";
        ctx->publishers.alerts->publish(alert_msg);

        RCLCPP_WARN(ctx->node->get_logger(), "%s", alert_msg.data.c_str());
    }

    RCLCPP_DEBUG(
        ctx->node->get_logger(), "Processed temperature: %.2f°C (samples: %zu)", msg->temperature, ctx->sample_count
    );
}

void get_statistics_handler(
    std::shared_ptr<Context> ctx,
    example_interfaces::srv::Trigger::Request::SharedPtr request,
    example_interfaces::srv::Trigger::Response::SharedPtr response
) {

    std::ostringstream oss;
    oss << "Statistics: samples=" << ctx->sample_count;

    if (ctx->sample_count > 0) {
        double avg = ctx->sum_temp / ctx->sample_count;
        oss << ", min=" << ctx->min_temp << "°C"
            << ", max=" << ctx->max_temp << "°C"
            << ", avg=" << avg << "°C";
    }

    response->success = true;
    response->message = oss.str();

    RCLCPP_INFO(ctx->node->get_logger(), "Statistics requested: %s", response->message.c_str());
}

void init(std::shared_ptr<Context> ctx) {
    RCLCPP_INFO(ctx->node->get_logger(), "Initializing data processor");
    RCLCPP_INFO(
        ctx->node->get_logger(),
        "Processing enabled: %s, Threshold: %.1f°C",
        ctx->params.processing_enabled ? "yes" : "no",
        ctx->params.threshold
    );

    // Set callbacks
    ctx->subscribers.sensor_data->set_callback(sensor_data_callback);
    ctx->services.get_statistics->set_request_handler(get_statistics_handler);
}

} // namespace breadcrumb_example::data_processor
