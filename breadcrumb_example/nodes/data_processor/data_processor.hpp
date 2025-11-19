#pragma once

#include <memory>
#include <vector>
#include <breadcrumb_example/data_processor_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace breadcrumb_example::data_processor {

// Extend the generated context with custom state
struct Context : DataProcessorContext<Context> {
    std::vector<double> temperature_history;
    size_t sample_count = 0;
    double min_temp = std::numeric_limits<double>::max();
    double max_temp = std::numeric_limits<double>::lowest();
    double sum_temp = 0.0;
};

// Forward declare init function
void init(std::shared_ptr<Context> ctx);

// Define the node class using the generated base
using DataProcessor = DataProcessorBase<Context, init>;

} // namespace breadcrumb_example::data_processor
