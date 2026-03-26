#!/usr/bin/env python3
"""CartPole Web UI Node - Provides web dashboard for monitoring and controlling the cartpole system."""

from dataclasses import dataclass, field
import json
from pathlib import Path
from queue import Queue
import threading
from threading import Lock
import time

from ament_index_python.packages import get_package_share_directory
from breadcrumb_example.cartpole_ui.interface import CartpoleUiSession, run
from jig import TransitionCallbackReturn
from flask import Flask, Response, jsonify, render_template, request
from rclpy.parameter import Parameter
from werkzeug.serving import run_simple

from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import SetBool, Trigger

from breadcrumb_example_interfaces.action import TrackPosition


@dataclass
class Session(CartpoleUiSession):
    """Extended session with web UI state."""

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

    # Controller status from status topic
    controller_status: str = ""

    # Action client state
    tracking_goal: bool = False

    # Rate limiting for web updates (only send updates every N seconds)
    last_web_update_time: float = 0.0
    web_update_rate: float = 0.05  # 20 Hz max (50ms between updates)


def push_web_update(sn: Session):
    """Push current state to all SSE web clients, with rate limiting."""
    current_time = time.time()

    with sn.state_lock:
        if current_time - sn.last_web_update_time < sn.web_update_rate:
            return
        sn.last_web_update_time = current_time

    if not sn.web_clients:
        return

    state_data = {
        "cart_position": sn.cart_position,
        "cart_velocity": sn.cart_velocity,
        "pole_angle": sn.pole_angle,
        "pole_angular_velocity": sn.pole_angular_velocity,
        "timestamp": sn.last_update_time,
        "tracking": sn.tracking_goal,
        "controller_status": sn.controller_status,
    }

    disconnected_clients = []
    for client_queue in sn.web_clients:
        try:
            if client_queue.full():
                try:
                    client_queue.get_nowait()
                except:
                    pass
            client_queue.put_nowait(state_data)
        except Exception as e:
            disconnected_clients.append(client_queue)

    for client in disconnected_clients:
        if client in sn.web_clients:
            sn.web_clients.remove(client)


def joint_states_callback(sn: Session, msg: JointState):
    """Callback for joint_states updates."""
    current_time = time.time()

    with sn.state_lock:
        if len(msg.position) >= 2 and len(msg.velocity) >= 2:
            sn.cart_position = msg.position[0]
            sn.pole_angle = msg.position[1]
            sn.cart_velocity = msg.velocity[0]
            sn.pole_angular_velocity = msg.velocity[1]
            sn.last_update_time = current_time

    push_web_update(sn)


def controller_status_callback(sn: Session, msg: String):
    """Callback for controller status updates."""
    sn.controller_status = msg.data
    push_web_update(sn)


def create_app(sn: Session) -> Flask:
    """Create Flask app with routes as closures over session."""
    app = Flask(
        __name__,
        template_folder=Path(get_package_share_directory("breadcrumb_example")) / "web_templates",
    )

    @app.route("/")
    def index():
        """Serve the dashboard HTML."""
        return render_template("dashboard.html", stop_buttons=list(sn.publishers.emergency_stops.keys()))

    @app.route("/events")
    def sse():
        """Server-Sent Events endpoint for real-time updates."""

        def event_stream():
            from queue import Empty

            q = Queue(maxsize=5)  # Smaller queue to prevent memory buildup
            sn.web_clients.append(q)
            sn.logger.info(f"SSE client connected. Total clients: {len(sn.web_clients)}")

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
                if q in sn.web_clients:
                    sn.web_clients.remove(q)
                sn.logger.info(f"SSE client disconnected. Total clients: {len(sn.web_clients)}")

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

            future = sn.service_clients.enable_controller.call_async(req)
            # Note: In a production system, you'd wait for the future with a timeout
            # For simplicity, we're fire-and-forget here

            sn.logger.info(f"Controller {'enabled' if enable else 'disabled'}")
            return jsonify({"success": True, "enabled": enable})
        except Exception as e:
            sn.logger.error(f"Failed to enable/disable controller: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route("/api/reset", methods=["POST"])
    def reset_simulator():
        """Reset the simulator."""
        try:
            req = Trigger.Request()
            future = sn.service_clients.reset_simulator.call_async(req)
            # Note: In a production system, you'd wait for the future with a timeout
            # For simplicity, we're fire-and-forget here

            sn.logger.info("Simulator reset requested")
            return jsonify({"success": True})
        except Exception as e:
            sn.logger.error(f"Failed to reset simulator: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route("/api/emergency_stop", methods=["POST"])
    def emergency_stop():
        """Trigger an emergency stop for a specific source."""
        try:
            data = request.get_json()
            source = data.get("source", "")

            if source not in sn.publishers.emergency_stops:
                return jsonify({"success": False, "error": f"Unknown stop source: {source}"}), 400

            msg = Bool()
            msg.data = True
            sn.publishers.emergency_stops[source].publish(msg)

            sn.logger.info(f"Emergency stop triggered for source: {source}")
            return jsonify({"success": True, "source": source})
        except Exception as e:
            sn.logger.error(f"Failed to trigger emergency stop: {e}")
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
            sn.tracking_goal = True
            send_goal_future = sn.action_clients.track_position.send_goal_async(
                goal_msg, feedback_callback=track_position_feedback
            )
            send_goal_future.add_done_callback(track_position_response)

            sn.logger.info(f"Tracking position goal sent: {target_position}")
            return jsonify({"success": True, "target": target_position})
        except Exception as e:
            sn.logger.error(f"Failed to send tracking goal: {e}")
            sn.tracking_goal = False
            return jsonify({"success": False, "error": str(e)}), 500

    def track_position_feedback(feedback_msg):
        """Handle action feedback."""
        feedback = feedback_msg.feedback
        sn.logger.info(
            f"Position tracking feedback: current={feedback.current_position:.3f}, "
            f"distance={feedback.distance_to_goal:.3f}"
        )

    def track_position_response(future):
        """Handle action goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            sn.logger.warn("Position tracking goal rejected")
            sn.tracking_goal = False
            return

        sn.logger.info("Position tracking goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(track_position_result)

    def track_position_result(future):
        """Handle action result."""
        result = future.result().result
        sn.tracking_goal = False
        sn.logger.info(
            f"Position tracking complete: success={result.success}, " f"final_position={result.final_position:.3f}"
        )

    @app.route("/api/get_params", methods=["GET"])
    def get_controller_params():
        """Get current controller parameters."""
        try:
            # Create parameter client for the controller node
            param_client = sn.node.create_client(GetParameters, f"/{sn.params.controller_node_name}/get_parameters")

            # Wait for service to be available
            if not param_client.wait_for_service(timeout_sec=2.0):
                sn.logger.error("Parameter service not available")
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
                    sn.logger.error("Parameter get timed out")
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
                        sn.logger.warn(f"Parameter {name} is not a double")
                        params[name] = 0.0
                else:
                    params[name] = 0.0

            sn.logger.info(f"Retrieved parameters: {params}")
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
            sn.logger.error(f"Failed to get parameters: {e}")
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
            param_client = sn.node.create_client(SetParameters, f"/{sn.params.controller_node_name}/set_parameters")

            # Build SetParameters request
            req = SetParameters.Request()
            req.parameters = [p.to_parameter_msg() for p in params]

            # Call service (fire and forget for simplicity)
            future = param_client.call_async(req)
            sn.logger.info(f"Parameter update requested: {data}")

            return jsonify({"success": True, "params": data})
        except Exception as e:
            sn.logger.error(f"Failed to set parameters: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    return app


def run_webserver(sn: Session):
    """Run Flask webserver in separate thread."""
    sn.logger.info(f"Starting webserver on port {sn.params.webserver_port}")

    # Create and run app with routes as closures over session
    run_simple(
        "0.0.0.0",
        sn.params.webserver_port,
        create_app(sn),
        threaded=True,
        use_reloader=False,
        use_debugger=False,
        use_evalex=False,
    )


def on_configure(sn: Session) -> TransitionCallbackReturn:
    """Initialize the cartpole UI node."""
    # Set up ROS callbacks
    sn.subscribers.joint_states.set_callback(joint_states_callback)
    sn.subscribers.controller_status.set_callback(controller_status_callback)

    # Start Flask webserver in separate daemon thread
    threading.Thread(target=run_webserver, args=(sn,), daemon=True).start()

    sn.logger.info(f"CartPole UI initialized. Dashboard available at http://localhost:{sn.params.webserver_port}")

    return TransitionCallbackReturn.SUCCESS


if __name__ == "__main__":
    run(Session, on_configure)
