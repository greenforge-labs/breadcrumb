#!/usr/bin/env python3
"""CartPole Web UI Node - Provides web dashboard for monitoring and controlling the cartpole system."""

from dataclasses import dataclass, field
import json
from pathlib import Path
from queue import Queue
from threading import Lock
import time

from ament_index_python.packages import get_package_share_directory
from breadcrumb_example.cartpole_ui.interface import CartpoleUiContext, run
from cake import create_thread
from flask import Flask, Response, jsonify, render_template, request
from rclpy.parameter import Parameter
from werkzeug.serving import run_simple

from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import JointState

from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import SetBool, Trigger

from breadcrumb_example_interfaces.action import TrackPosition


@dataclass
class Context(CartpoleUiContext):
    """Extended context with web UI state."""

    # Current cartpole state
    cart_position: float = 0.0
    cart_velocity: float = 0.0
    pole_angle: float = 0.0
    pole_angular_velocity: float = 0.0
    last_update_time: float = 0.0

    # Thread safety
    state_lock: Lock = field(default_factory=Lock)

    # SSE client queues
    web_clients: list = field(default_factory=list)

    # Action client state
    tracking_goal: bool = False

    # Rate limiting for web updates (only send updates every N seconds)
    last_web_update_time: float = 0.0
    web_update_rate: float = 0.05  # 20 Hz max (50ms between updates)


def joint_states_callback(ctx: Context, msg: JointState):
    """Callback for joint_states updates - with rate limiting."""
    # joint_states has: position[0] = cart, position[1] = pole, velocity[0] = cart_vel, velocity[1] = pole_vel

    current_time = time.time()

    with ctx.state_lock:
        if len(msg.position) >= 2 and len(msg.velocity) >= 2:
            ctx.cart_position = msg.position[0]
            ctx.pole_angle = msg.position[1]
            ctx.cart_velocity = msg.velocity[0]
            ctx.pole_angular_velocity = msg.velocity[1]
            ctx.last_update_time = current_time

        # Rate limit web updates - only send if enough time has passed
        if current_time - ctx.last_web_update_time < ctx.web_update_rate:
            return  # Skip this update

        ctx.last_web_update_time = current_time

    # Only push to web clients if we have clients and passed rate limit
    if not ctx.web_clients:
        return

    # Push update to all web clients
    state_data = {
        "cart_position": ctx.cart_position,
        "cart_velocity": ctx.cart_velocity,
        "pole_angle": ctx.pole_angle,
        "pole_angular_velocity": ctx.pole_angular_velocity,
        "timestamp": ctx.last_update_time,
        "tracking": ctx.tracking_goal,
    }

    # Clean up disconnected clients while pushing updates
    disconnected_clients = []
    for client_queue in ctx.web_clients:
        try:
            # Drop oldest item if queue is full to prevent memory buildup
            if client_queue.full():
                try:
                    client_queue.get_nowait()  # Remove oldest
                except:
                    pass
            client_queue.put_nowait(state_data)
        except Exception as e:
            # Queue broken or client disconnected
            disconnected_clients.append(client_queue)

    # Remove disconnected clients
    for client in disconnected_clients:
        if client in ctx.web_clients:
            ctx.web_clients.remove(client)


def create_app(ctx: Context) -> Flask:
    """Create Flask app with routes as closures over context."""
    app = Flask(
        __name__,
        template_folder=Path(get_package_share_directory("breadcrumb_example")) / "web_templates",
    )

    @app.route("/")
    def index():
        """Serve the dashboard HTML."""
        return render_template("dashboard.html")

    @app.route("/events")
    def sse():
        """Server-Sent Events endpoint for real-time updates."""

        def event_stream():
            from queue import Empty

            q = Queue(maxsize=5)  # Smaller queue to prevent memory buildup
            ctx.web_clients.append(q)
            ctx.logger.info(f"SSE client connected. Total clients: {len(ctx.web_clients)}")

            try:
                while True:
                    try:
                        # Use timeout to send keepalive and detect disconnections
                        data = q.get(timeout=5.0)
                        yield f"data: {json.dumps(data)}\n\n"
                    except Empty:
                        # Timeout - send keepalive comment to detect disconnections
                        yield ": keepalive\n\n"
            finally:
                # Clean up this client's queue (always runs on disconnect)
                if q in ctx.web_clients:
                    ctx.web_clients.remove(q)
                ctx.logger.info(f"SSE client disconnected. Total clients: {len(ctx.web_clients)}")

        return Response(event_stream(), mimetype="text/event-stream")

    @app.route("/api/enable", methods=["POST"])
    def enable_controller():
        """Enable or disable the controller."""
        try:
            data = request.get_json()
            enable = data.get("enable", False)

            # Call the enable service
            req = SetBool.Request()
            req.data = enable

            future = ctx.service_clients.enable_controller.call_async(req)
            # Note: In a production system, you'd wait for the future with a timeout
            # For simplicity, we're fire-and-forget here

            ctx.logger.info(f"Controller {'enabled' if enable else 'disabled'}")
            return jsonify({"success": True, "enabled": enable})
        except Exception as e:
            ctx.logger.error(f"Failed to enable/disable controller: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route("/api/reset", methods=["POST"])
    def reset_simulator():
        """Reset the simulator."""
        try:
            req = Trigger.Request()
            future = ctx.service_clients.reset_simulator.call_async(req)
            # Note: In a production system, you'd wait for the future with a timeout
            # For simplicity, we're fire-and-forget here

            ctx.logger.info("Simulator reset requested")
            return jsonify({"success": True})
        except Exception as e:
            ctx.logger.error(f"Failed to reset simulator: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route("/api/track_position", methods=["POST"])
    def track_position():
        """Send a position tracking goal to the controller."""
        try:
            data = request.get_json()
            target_position = float(data.get("target_position", 0.0))
            goal_tolerance = float(data.get("goal_tolerance", 0.05))

            # Create action goal
            goal_msg = TrackPosition.Goal()
            goal_msg.target_position = target_position
            goal_msg.goal_tolerance = goal_tolerance

            # Send goal
            ctx.tracking_goal = True
            send_goal_future = ctx.action_clients.track_position.send_goal_async(
                goal_msg, feedback_callback=track_position_feedback
            )
            send_goal_future.add_done_callback(track_position_response)

            ctx.logger.info(f"Tracking position goal sent: {target_position}")
            return jsonify({"success": True, "target": target_position})
        except Exception as e:
            ctx.logger.error(f"Failed to send tracking goal: {e}")
            ctx.tracking_goal = False
            return jsonify({"success": False, "error": str(e)}), 500

    def track_position_feedback(feedback_msg):
        """Handle action feedback."""
        feedback = feedback_msg.feedback
        ctx.logger.info(
            f"Position tracking feedback: current={feedback.current_position:.3f}, "
            f"distance={feedback.distance_to_goal:.3f}"
        )

    def track_position_response(future):
        """Handle action goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            ctx.logger.warn("Position tracking goal rejected")
            ctx.tracking_goal = False
            return

        ctx.logger.info("Position tracking goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(track_position_result)

    def track_position_result(future):
        """Handle action result."""
        result = future.result().result
        ctx.tracking_goal = False
        ctx.logger.info(
            f"Position tracking complete: success={result.success}, " f"final_position={result.final_position:.3f}"
        )

    @app.route("/api/get_params", methods=["GET"])
    def get_controller_params():
        """Get current controller parameters."""
        try:
            # Create parameter client for the controller node
            param_client = ctx.node.create_client(GetParameters, f"/{ctx.params.controller_node_name}/get_parameters")

            # Wait for service to be available
            if not param_client.wait_for_service(timeout_sec=2.0):
                ctx.logger.error("Parameter service not available")
                return jsonify({"success": False, "error": "Service not available"}), 503

            # Build GetParameters request
            req = GetParameters.Request()
            req.names = ["k1", "k2", "k3", "k4"]

            # Call service
            future = param_client.call_async(req)

            # Wait for the future to complete with polling
            timeout = 2.0
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    ctx.logger.error("Parameter get timed out")
                    return jsonify({"success": False, "error": "Request timed out"}), 504
                time.sleep(0.01)  # Small sleep to avoid busy-waiting

            response = future.result()

            # Parse the parameter values
            params = {}
            for i, name in enumerate(req.names):
                if i < len(response.values):
                    param_value = response.values[i]
                    if param_value.type == ParameterType.PARAMETER_DOUBLE:
                        params[name] = param_value.double_value
                    else:
                        ctx.logger.warn(f"Parameter {name} is not a double")
                        params[name] = 0.0
                else:
                    params[name] = 0.0

            ctx.logger.info(f"Retrieved parameters: {params}")
            return jsonify(
                {
                    "success": True,
                    "k1": params.get("k1", 0.0),
                    "k2": params.get("k2", 0.0),
                    "k3": params.get("k3", 0.0),
                    "k4": params.get("k4", 0.0),
                }
            )
        except Exception as e:
            ctx.logger.error(f"Failed to get parameters: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route("/api/set_params", methods=["POST"])
    def set_controller_params():
        """Update controller parameters dynamically."""
        try:
            data = request.get_json()

            # Build parameter list
            params = []
            if "k1" in data:
                params.append(Parameter("k1", Parameter.Type.DOUBLE, float(data["k1"])))
            if "k2" in data:
                params.append(Parameter("k2", Parameter.Type.DOUBLE, float(data["k2"])))
            if "k3" in data:
                params.append(Parameter("k3", Parameter.Type.DOUBLE, float(data["k3"])))
            if "k4" in data:
                params.append(Parameter("k4", Parameter.Type.DOUBLE, float(data["k4"])))

            if not params:
                return jsonify({"success": False, "error": "No parameters provided"}), 400

            # Create parameter client for the controller node
            param_client = ctx.node.create_client(SetParameters, f"/{ctx.params.controller_node_name}/set_parameters")

            # Build SetParameters request
            req = SetParameters.Request()
            req.parameters = [p.to_parameter_msg() for p in params]

            # Call service (fire and forget for simplicity)
            future = param_client.call_async(req)
            ctx.logger.info(f"Parameter update requested: {data}")

            return jsonify({"success": True, "params": data})
        except Exception as e:
            ctx.logger.error(f"Failed to set parameters: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    return app


def run_webserver(ctx: Context):
    """Run Flask webserver in separate thread (tracked by cake)."""
    ctx.logger.info(f"Starting webserver on port {ctx.params.webserver_port}")

    # Create and run app with routes as closures over context
    run_simple(
        "0.0.0.0",
        ctx.params.webserver_port,
        create_app(ctx),
        threaded=True,
        use_reloader=False,
        use_debugger=False,
        use_evalex=False,
    )


def init(ctx: Context):
    """Initialize the cartpole UI node."""
    # Set up ROS callbacks
    ctx.subscribers.joint_states.set_callback(joint_states_callback)

    # Start Flask webserver in separate thread using cake's thread utility
    create_thread(ctx, run_webserver)

    ctx.logger.info(f"CartPole UI initialized. Dashboard available at http://localhost:{ctx.params.webserver_port}")


if __name__ == "__main__":
    run(Context, init)
