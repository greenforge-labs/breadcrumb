from dataclasses import dataclass

from breadcrumb_example.monitor_node.interface import MonitorNodeContext, run

from sensor_msgs.msg import Temperature

from example_interfaces.srv import Trigger


@dataclass
class Context(MonitorNodeContext):
    """Extended context with custom state."""

    data_count: int = 0
    alert_count: int = 0
    last_temperature: float = 0.0


def processed_data_callback(ctx: Context, msg: Temperature):
    """Handle incoming processed temperature data."""
    ctx.data_count += 1
    ctx.last_temperature = msg.temperature

    if ctx.data_count % 10 == 0:
        ctx.logger.info(f"Received {ctx.data_count} temperature samples. " f"Latest: {msg.temperature:.2f}°C")


def alerts_callback(ctx: Context, msg):
    """Handle incoming alerts."""
    ctx.alert_count += 1
    ctx.logger.warn(f"ALERT #{ctx.alert_count}: {msg.data}")


def timer_callback(ctx: Context):
    """Periodically log status and request statistics."""
    ctx.logger.info(
        f"Monitor status - Data: {ctx.data_count}, "
        f"Alerts: {ctx.alert_count}, "
        f"Last temp: {ctx.last_temperature:.2f}°C"
    )

    # Request statistics from data processor
    if ctx.service_clients.get_statistics.service_is_ready():
        future = ctx.service_clients.get_statistics.call_async(Trigger.Request())
        future.add_done_callback(lambda f: statistics_response_callback(ctx, f))
    else:
        ctx.logger.debug("Statistics service not ready")


def statistics_response_callback(ctx: Context, future):
    """Handle response from statistics service."""
    try:
        response = future.result()
        if response.success:
            ctx.logger.info(f"Statistics: {response.message}")
        else:
            ctx.logger.warn(f"Statistics request failed: {response.message}")
    except Exception as e:
        ctx.logger.error(f"Service call failed: {e}")


def init(ctx: Context):
    """Initialize the monitor node."""
    ctx.logger.info("Initializing monitor node")
    ctx.logger.info(f"Log interval: {ctx.params.log_interval} seconds")

    # Set callbacks for subscribers
    ctx.subscribers.processed_data.set_callback(processed_data_callback)
    ctx.subscribers.alerts.set_callback(alerts_callback)

    # Create timer for periodic logging
    ctx.node.create_timer(ctx.params.log_interval, lambda: timer_callback(ctx))


if __name__ == "__main__":
    run(Context, init)
