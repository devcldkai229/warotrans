"""ROS2 ↔ MQTT Robot Gateway: heartbeat/telemetry uplink + command downlink.

Threading:
- MQTT network thread (paho) chỉ parse JSON rồi đẩy vào inbox + trigger guard condition.
  Không gọi rclpy / Nav2 từ thread đó, không block.
- ROS executor thread: CommandRouter, MoveCommandHandler, Nav2 action futures, command tick.
- Wall-clock daemon threads: heartbeat + telemetry (fleet liveness). SingleThreadedExecutor
  can stall while waiting on ComputeRoute/DynamicEdges; ROS timers (even SYSTEM_TIME) then
  miss Host HeartbeatTimeoutSeconds. Wall threads keep MQTT uplink alive independently.
"""

from __future__ import annotations

import os
import queue
import threading
import time
from datetime import datetime, timezone
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from warotrans_fleet import navigation_state as nav_state
from warotrans_fleet.command_stack import build_command_stack
from warotrans_fleet.confirm_button import ButtonEdgeDetector, LgpioButtonInput
from warotrans_fleet.drive_assist import Footprint, Gaps, corridor_gaps, scan_to_base_points
from warotrans_fleet.interaction_contract import (
    InteractionContractError,
    parse_interaction_request,
    parse_interaction_result,
)
from warotrans_fleet.interaction_controller import InteractionController
from warotrans_fleet.localization_hysteresis import LocalizationHysteresis
from warotrans_fleet.mqtt_publisher import ReconnectingMqttPublisher, default_paho_factory
from warotrans_fleet.nav2_status_tracker import Nav2StatusTracker
from warotrans_fleet.navigator import NavigatorPort, UnavailableNavigator
from warotrans_fleet.payload_builder import build_heartbeat, build_telemetry
from warotrans_fleet.sequence import SequenceClock
from warotrans_fleet.teleop import TeleopController
from warotrans_fleet.transforms import quaternion_to_yaw


def _env_or(param_value: str, env_name: str) -> str:
    env = os.environ.get(env_name)
    return env if env is not None and env != "" else param_value


class _MqttCommandEvents:
    """CommandEventPublisher → MQTT QoS1 trên command_ack / command_result."""

    def __init__(self, node: "RobotGatewayNode") -> None:
        self._node = node

    def publish_ack(self, payload: dict[str, Any]) -> None:
        self._publish(self._node.topic("command_ack"), payload)

    def publish_result(self, payload: dict[str, Any]) -> None:
        self._publish(self._node.topic("command_result"), payload)

    def _publish(self, topic: str, payload: dict[str, Any]) -> None:
        if not self._node.mqtt.publish(topic, payload, qos=1):
            self._node.get_logger().error(
                f"[MqttPublishDropped] topic={topic} commandId={payload.get('commandId')} (outbound queue full)"
            )


class RobotGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_gateway")

        self.declare_parameter("robot_code", "RBT-001")
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_client_id", "warotrans-robot-gateway")
        self.declare_parameter("topic_prefix", "warotrans/v1")
        self.declare_parameter("map_version_code", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("heartbeat_hz", 1.0)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("max_transform_age_sec", 0.5)
        # Consecutive stale TF samples before publishing LOST (telemetry_hz=5 → ~0.6s).
        self.declare_parameter("localization_stale_limit", 3)
        # MOVE command
        self.declare_parameter("default_navigate_timeout_sec", 0.0)  # 0 = không timeout nếu command không gửi
        self.declare_parameter("goal_response_timeout_sec", 10.0)
        self.declare_parameter("cancel_confirm_timeout_sec", 10.0)
        self.declare_parameter("terminal_status_hold_sec", 3.0)
        self.declare_parameter("command_tick_hz", 5.0)
        self.declare_parameter("command_ledger_size", 500)
        # Robot has no battery sensor yet → telemetry sends null. Simulation only: a fixed value
        # (0..100) so backend CHECK.BATTERY_MIN can be exercised. < 0 = disabled (real robot).
        self.declare_parameter("simulated_battery_percent", -1.0)
        # Manual drive (MQTT `teleop` topic → /cmd_vel). Limits default to the real robot's provisional
        # motor limits (calibration.yaml); the simulation overrides them.
        self.declare_parameter("teleop_enabled", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("teleop_max_linear", 0.10)
        self.declare_parameter("teleop_max_angular", 0.45)
        self.declare_parameter("teleop_deadman_sec", 0.5)
        self.declare_parameter("teleop_max_message_age_sec", 0.0)  # 0 = off (robot clock may not be synced)
        # Drive Assist (teleop `assist`): lidar corridor as wide as the chassis (geometry.yaml 0.32 x 0.26 m).
        self.declare_parameter("assist_scan_topic", "/scan")
        self.declare_parameter("assist_half_length", 0.16)
        self.declare_parameter("assist_half_width", 0.13)
        self.declare_parameter("assist_stop_gap", 0.15)
        self.declare_parameter("assist_slow_gap", 0.6)
        self.declare_parameter("assist_max_scan_age_sec", 0.5)
        # HUMAN_CONFIRM button (not E-stop): BCM12 ↔ GND, active-low pull-up.
        self.declare_parameter("human_confirm_enabled", False)
        self.declare_parameter("human_confirm_gpiochip", 4)
        self.declare_parameter("human_confirm_gpio_pin", 12)
        self.declare_parameter("human_confirm_active_low", True)
        self.declare_parameter("human_confirm_debounce_ms", 50)
        self.declare_parameter("human_confirm_min_interval_ms", 300)
        self.declare_parameter("human_confirm_resend_sec", 5.0)
        # Route-grant enforcement (traffic coordinator). Off by default; enabled when
        # route_enforcement:=true OR route_graph path is non-empty.
        self.declare_parameter("route_enforcement", False)
        self.declare_parameter("route_graph", "")
        self.declare_parameter("route_graph_hash", "")
        self.declare_parameter("route_corridor_buffer_m", 0.35)
        self.declare_parameter("grant_loss_timeout_sec", 3.0)
        self.declare_parameter("adjust_edges_service", "route_server/DynamicEdgesScorer/adjust_edges")
        self.declare_parameter("compute_route_action", "compute_route")
        # Traffic mode: "legacy" (RouteEnforcer + NAVIGATE route grant, mặc định) hoặc "active"
        # (segment motion authority — TrafficClient). Chỉ là param; legacy giữ nguyên hành vi cũ.
        self.declare_parameter("traffic_mode", "legacy")
        self.declare_parameter("authority_loss_timeout_sec", 3.0)
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("enable_authority_gate", False)  # gate is a separate launch node
        self.declare_parameter("authority_state_path", "")  # "" = file tạm mặc định của AuthorityStore

        self._robot_code = _env_or(self._str_param("robot_code"), "WARO_ROBOT_CODE")
        self._mqtt_host = _env_or(self._str_param("mqtt_host"), "WARO_MQTT_HOST")
        mqtt_port_param = int(self.get_parameter("mqtt_port").get_parameter_value().integer_value)
        port_env = os.environ.get("WARO_MQTT_PORT")
        self._mqtt_port = int(port_env) if port_env else mqtt_port_param
        self._topic_prefix = self._str_param("topic_prefix").strip().rstrip("/")
        self._map_version_code = _env_or(self._str_param("map_version_code"), "WARO_MAP_VERSION_CODE") or None
        self._map_frame = self._str_param("map_frame")
        self._base_frame = self._str_param("base_frame")
        self._max_transform_age = self._float_param("max_transform_age_sec")
        simulated_battery = self._float_param("simulated_battery_percent")
        self._battery_percent = min(simulated_battery, 100.0) if simulated_battery >= 0.0 else None
        client_id = self._str_param("mqtt_client_id")

        default_timeout = self._float_param("default_navigate_timeout_sec")
        goal_response_timeout = self._float_param("goal_response_timeout_sec")
        cancel_confirm_timeout = self._float_param("cancel_confirm_timeout_sec")
        terminal_hold = self._float_param("terminal_status_hold_sec")
        tick_hz = self._float_param("command_tick_hz")
        ledger_size = int(self.get_parameter("command_ledger_size").get_parameter_value().integer_value)
        if default_timeout < 0 or goal_response_timeout <= 0 or cancel_confirm_timeout <= 0:
            raise ValueError("navigate timeouts must be > 0 (default_navigate_timeout_sec may be 0 = disabled)")
        if terminal_hold < 0 or tick_hz <= 0 or ledger_size < 1:
            raise ValueError("terminal_status_hold_sec >= 0, command_tick_hz > 0, command_ledger_size >= 1")

        self._seq = SequenceClock()
        self._nav_tracker = Nav2StatusTracker()
        stale_limit = int(self.get_parameter("localization_stale_limit").get_parameter_value().integer_value)
        if stale_limit < 1:
            raise ValueError("localization_stale_limit must be >= 1")
        self._loc_hysteresis = LocalizationHysteresis(stale_limit=stale_limit)
        self._last_localization = "UNKNOWN"
        self._last_pose_age_sec: Optional[float] = None
        self._lin_vel = 0.0
        self._ang_vel = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        odom_topic = self._str_param("odom_topic")
        amcl_topic = self._str_param("amcl_pose_topic")
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self._on_amcl_pose,
            qos_profile_sensor_data,
        )

        self._traffic_mode = self._str_param("traffic_mode").strip().lower()
        if self._traffic_mode not in ("legacy", "active"):
            raise ValueError(f"traffic_mode must be 'legacy' or 'active', got {self._traffic_mode!r}")
        self._authority_loss_timeout = self._float_param("authority_loss_timeout_sec")
        if self._authority_loss_timeout <= 0:
            raise ValueError("authority_loss_timeout_sec must be > 0")
        self._enable_authority_gate = bool(self.get_parameter("enable_authority_gate").get_parameter_value().bool_value)
        self._traffic_client = None

        navigator = self._setup_nav2()
        route_enforcer = self._setup_route_enforcer()

        self._teleop_enabled = bool(self.get_parameter("teleop_enabled").get_parameter_value().bool_value)
        human_env = os.environ.get("WARO_HUMAN_CONFIRM_ENABLED", "").strip().lower()
        if human_env in ("1", "true", "yes", "on"):
            self._human_confirm_enabled = True
        elif human_env in ("0", "false", "no", "off"):
            self._human_confirm_enabled = False
        else:
            self._human_confirm_enabled = bool(
                self.get_parameter("human_confirm_enabled").get_parameter_value().bool_value
            )
        pin_env = os.environ.get("WARO_HUMAN_CONFIRM_GPIO_PIN")
        self._human_confirm_pin = (
            int(pin_env)
            if pin_env
            else int(self.get_parameter("human_confirm_gpio_pin").get_parameter_value().integer_value)
        )

        subscribe_topics = [
            self.topic("command"),
            self.topic("interaction_request"),
            self.topic("interaction_result"),
        ] + ([self.topic("teleop")] if self._teleop_enabled else [])
        self.mqtt = ReconnectingMqttPublisher(
            host=self._mqtt_host,
            port=self._mqtt_port,
            client_id=f"{client_id}-{self._robot_code}",
            client_factory=default_paho_factory,
            subscribe_topic=subscribe_topics,
            on_message=self._on_mqtt_message,
        )

        if self._traffic_mode == "active":
            self._traffic_client = self._setup_traffic_client()
            # Boot sync: backend reconciles occupancy from what this robot still holds (queued until connected).
            self._publish_authority_state()

        self._commands = build_command_stack(
            robot_code=self._robot_code,
            publisher=_MqttCommandEvents(self),
            navigator=navigator,
            localization_provider=lambda: self._last_localization,
            log=self.get_logger(),
            map_frame=self._map_frame,
            robot_map_version_code=self._map_version_code,
            default_timeout_sec=default_timeout or None,
            goal_response_timeout_sec=goal_response_timeout,
            cancel_confirm_timeout_sec=cancel_confirm_timeout,
            terminal_status_hold_sec=terminal_hold,
            ledger_capacity=ledger_size,
            external_goal_active=lambda: self._nav_tracker.has_active_goal,
            route_enforcer=route_enforcer,
            traffic_mode=self._traffic_mode,
            traffic_client=self._traffic_client,
            expected_graph_hash=self._str_param("route_graph_hash").strip() or None,
        )

        self._teleop: Optional[TeleopController] = None
        if self._teleop_enabled:
            self._cmd_vel_pub = self.create_publisher(Twist, self._str_param("cmd_vel_topic"), 10)
            self._footprint = Footprint(
                half_length=self._float_param("assist_half_length"),
                half_width=self._float_param("assist_half_width"),
            )
            self._scan_max_age = self._float_param("assist_max_scan_age_sec")
            self._gaps: Optional[Gaps] = None
            self._gaps_at = 0.0
            self._laser_mount: dict[str, tuple[float, float, float]] = {}
            self.create_subscription(LaserScan, self._str_param("assist_scan_topic"), self._on_scan, qos_profile_sensor_data)
            self._teleop = TeleopController(
                robot_code=self._robot_code,
                publish_twist=self._publish_cmd_vel,
                monotonic=time.monotonic,
                navigation_busy=lambda: (
                    self._commands.move.active_command_id is not None or self._nav_tracker.has_active_goal
                ),
                max_linear=self._float_param("teleop_max_linear"),
                max_angular=self._float_param("teleop_max_angular"),
                deadman_sec=self._float_param("teleop_deadman_sec"),
                max_message_age_sec=self._float_param("teleop_max_message_age_sec"),
                log=self.get_logger(),
                gaps_provider=self._fresh_gaps,
                assist_stop_gap=self._float_param("assist_stop_gap"),
                assist_slow_gap=self._float_param("assist_slow_gap"),
            )
            self.create_timer(0.1, self._teleop.tick)

        self._interaction = InteractionController(
            robot_code=self._robot_code,
            boot_id=self._seq.boot_id,
            next_sequence=self._seq.next_interaction,
            publish_confirm=self._publish_interaction_confirm,
            log_info=lambda m: self.get_logger().info(m),
            log_warn=lambda m: self.get_logger().warning(m),
            resend_interval_sec=self._float_param("human_confirm_resend_sec"),
        )
        self._button_input: Optional[LgpioButtonInput] = None
        self._button_detector: Optional[ButtonEdgeDetector] = None
        if self._human_confirm_enabled:
            self._button_input = LgpioButtonInput(
                gpiochip=int(self.get_parameter("human_confirm_gpiochip").get_parameter_value().integer_value),
                pin=self._human_confirm_pin,
                log=self.get_logger(),
            )
            self._button_detector = ButtonEdgeDetector(
                active_low=bool(self.get_parameter("human_confirm_active_low").get_parameter_value().bool_value),
                debounce_ms=int(self.get_parameter("human_confirm_debounce_ms").get_parameter_value().integer_value),
                min_interval_ms=int(
                    self.get_parameter("human_confirm_min_interval_ms").get_parameter_value().integer_value
                ),
            )
            self.create_timer(0.01, self._on_button_timer)
            self.get_logger().info(
                f"HUMAN_CONFIRM_BUTTON enabled BCM={self._human_confirm_pin} "
                f"(available={self._button_input.available})"
            )
        else:
            self.get_logger().info("HUMAN_CONFIRM_BUTTON disabled (human_confirm_enabled=false)")

        # Inbox: MQTT thread → executor thread (commands, teleop, interaction, in arrival order).
        self._inbox: "queue.SimpleQueue[tuple[str, dict[str, Any]]]" = queue.SimpleQueue()
        self._inbox_signal = self.create_guard_condition(self._drain_inbox)
        self.create_timer(1.0 / tick_hz, self._commands.router.tick)

        self.mqtt.start()

        hb_hz = max(0.1, self._float_param("heartbeat_hz"))
        tel_hz = max(0.1, self._float_param("telemetry_hz"))
        # Wall-clock threads: independent of ROS executor stalls and /clock under use_sim_time.
        self._fleet_uplink_stop = threading.Event()
        self._heartbeat_thread = threading.Thread(
            target=self._fleet_uplink_loop,
            name="fleet-heartbeat",
            args=(1.0 / hb_hz, self._on_heartbeat_timer, "heartbeat"),
            daemon=True,
        )
        self._telemetry_thread = threading.Thread(
            target=self._fleet_uplink_loop,
            name="fleet-telemetry",
            args=(1.0 / tel_hz, self._on_telemetry_timer, "telemetry"),
            daemon=True,
        )
        self._heartbeat_thread.start()
        self._telemetry_thread.start()

        self.get_logger().info(
            f"Robot gateway started robot_code={self._robot_code} "
            f"mqtt={self._mqtt_host}:{self._mqtt_port} "
            f"frames={self._map_frame}->{self._base_frame} bootId={self._seq.boot_id} "
            f"subscribe={subscribe_topics} commands={list(self._commands.router.supported_types)} "
            f"mapVersion={self._map_version_code}"
        )

    # ------------------------------------------------------------------ setup helpers

    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    def _setup_nav2(self) -> NavigatorPort:
        action_name = self._str_param("navigate_action")
        try:
            from action_msgs.msg import GoalStatusArray

            from warotrans_fleet.nav2_navigator import Nav2ActionNavigator
        except ImportError as exc:
            self.get_logger().error(f"Nav2 unavailable ({exc}); NAVIGATE_TO_POSE will be rejected NAV2_UNAVAILABLE")
            return UnavailableNavigator()

        navigator = Nav2ActionNavigator(self, action_name)
        self.create_subscription(GoalStatusArray, f"{action_name}/_action/status", self._on_goal_status, 10)
        self.get_logger().info(f"Nav2 ActionClient on {action_name}")
        return navigator

    def _setup_route_enforcer(self):
        from warotrans_fleet.route_enforcer import RouteEnforcer, RouteEnforcerConfig

        enforcement = bool(self.get_parameter("route_enforcement").get_parameter_value().bool_value)
        graph_path = self._str_param("route_graph").strip()
        if not enforcement and not graph_path:
            return None

        from warotrans_fleet.nav2_route_client import try_create_nav2_route_client

        port = try_create_nav2_route_client(
            self,
            adjust_edges_service=self._str_param("adjust_edges_service"),
            compute_route_action=self._str_param("compute_route_action"),
        )
        if port is None:
            self.get_logger().warning(
                "route enforcement requested but nav2_msgs unavailable; NAVIGATE with route will fail"
            )

        config = RouteEnforcerConfig(
            enabled=enforcement or bool(graph_path),
            graph_path=graph_path,
            graph_hash=self._str_param("route_graph_hash"),
            corridor_buffer_m=self._float_param("route_corridor_buffer_m"),
            grant_loss_timeout_sec=self._float_param("grant_loss_timeout_sec"),
        )
        enforcer = RouteEnforcer(
            config=config,
            graph_port=port,
            pose_provider=self._route_pose_xy,
            mqtt_connected=lambda: self.mqtt.is_connected,
            log=lambda m: self.get_logger().info(m),
        )
        self.get_logger().info(
            f"Route enforcement ON graph={graph_path or '(none)'} "
            f"hash_param={self._str_param('route_graph_hash') or '(auto/sidecar)'} "
            f"corridor_buffer={config.corridor_buffer_m}m"
        )
        return enforcer

    def _setup_traffic_client(self):
        """Active Mode: TrafficClient + Nav2 FollowPath segment executor + SafetySupervisor."""
        from warotrans_fleet.authority_store import AuthorityStore
        from warotrans_fleet.nav2_route_client import try_create_nav2_route_client
        from warotrans_fleet.route_grant import load_graph_edges
        from warotrans_fleet.safety_supervisor import SafetySupervisor
        from warotrans_fleet.segment_executor import UnavailableSegmentExecutor
        from warotrans_fleet.traffic_client import TrafficClient

        graph_path = self._str_param("route_graph").strip()
        all_edge_ids: list[int] = []
        graph_edges = []
        if graph_path:
            graph_edges = load_graph_edges(graph_path)
            all_edge_ids = [e.edge_id for e in graph_edges]
        port = try_create_nav2_route_client(
            self,
            adjust_edges_service=self._str_param("adjust_edges_service"),
            compute_route_action=self._str_param("compute_route_action"),
            blocking_adjust=False,
        )
        try:
            from warotrans_fleet.nav2_segment_executor import Nav2SegmentExecutor

            executor = Nav2SegmentExecutor(
                node=self,
                route_port=port,
                follow_path_action=self._str_param("follow_path_action") or "follow_path",
                log=lambda m: self.get_logger().info(m),
            )
            if graph_edges:
                executor.set_edge_polylines(
                    {
                        e.edge_id: list(e.points)
                        if e.points
                        else [(e.x0, e.y0), (e.x1, e.y1)]
                        for e in graph_edges
                    }
                )
            self.get_logger().info(
                f"Active Mode: Nav2SegmentExecutor ready edges={len(graph_edges)} graph={graph_path or '(none)'}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Nav2SegmentExecutor unavailable ({exc!r}); using fail-safe rejector")
            executor = UnavailableSegmentExecutor()

        state_path = self._str_param("authority_state_path").strip() or None
        store = AuthorityStore(state_path)
        client = TrafficClient(
            store=store,
            route_port=port,
            executor=executor,
            progress_publisher=self._publish_traffic_progress,
            pose_provider=self._traffic_pose,
            all_edge_ids=all_edge_ids,
            log=lambda m: self.get_logger().info(m),
            localization_ok=lambda: self._last_localization == "LOCALIZED",
            frame_id=self._map_frame,
        )
        self._authority_store = store
        from warotrans_fleet.safety_supervisor import SafetyConfig

        pose_max_age = float(self._max_transform_age)
        scan_max_age = float(getattr(self, "_scan_max_age", pose_max_age) or pose_max_age)
        self._safety_supervisor = SafetySupervisor(
            config=SafetyConfig(
                authority_loss_timeout_sec=float(self._authority_loss_timeout),
                pose_max_age_sec=max(0.5, pose_max_age),
                scan_max_age_sec=max(0.5, scan_max_age),
                # Sim WSL often pauses briefly on TF/scan jitter; don't escalate to FAULT.
                pause_fault_timeout_sec=30.0,
            ),
            authority_provider=lambda: store.active,
            motion_expected=lambda: client.state == "EXECUTING",
            pose_provider=lambda: (p[:2] if (p := self._traffic_pose()) else None),
            pose_age_provider=self._safety_pose_age,
            scan_age_provider=self._safety_scan_age,
            localization_ok_provider=lambda: self._last_localization == "LOCALIZED",
            last_authority_at_provider=lambda: getattr(client, "last_authority_at", None),
            on_fault=lambda code, detail: client.fault(code, detail),
            on_authority_ok=self._publish_authority_ok,
        )
        self.create_timer(0.05, self._on_safety_supervisor_tick)
        self.get_logger().info(
            f"traffic_mode=active: TrafficClient+SafetySupervisor ready "
            f"authority_loss_timeout={self._authority_loss_timeout}s "
            f"enable_authority_gate={self._enable_authority_gate}"
        )
        return client

    def _publish_authority_ok(self, ok: bool) -> None:
        from std_msgs.msg import Bool

        pub = getattr(self, "_authority_ok_pub", None)
        if pub is None:
            self._authority_ok_pub = self.create_publisher(Bool, "/traffic/authority_ok", 10)
            pub = self._authority_ok_pub
        msg = Bool()
        msg.data = bool(ok)
        pub.publish(msg)

    def _on_safety_supervisor_tick(self) -> None:
        supervisor = getattr(self, "_safety_supervisor", None)
        if supervisor is None:
            return
        try:
            supervisor.tick()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"SafetySupervisor tick failed: {exc!r}")
            self._publish_authority_ok(False)
        client = self._traffic_client
        if client is not None and client.state == "FAULTED" and not client.synced:
            # Sau FAULT: báo authority_state để backend reconcile, rồi cho phép authority seq cao hơn.
            if self._publish_authority_state():
                client.mark_synced()

    def _publish_authority_state(self) -> bool:
        import uuid

        client = self._traffic_client
        if client is None:
            return False
        body = {
            "schemaVersion": 1,
            "messageId": str(uuid.uuid4()),
            "robotCode": self._robot_code,
            "bootId": self._seq.boot_id,
            **client.authority_state_report(),
            "sentAt": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
        }
        if not self.mqtt.publish(self.topic("authority_state"), body, qos=1):
            self.get_logger().warning("authority_state queue full; message dropped")
            return False
        self.get_logger().info(f"authority_state published state={body.get('state')} lastSeq={body.get('lastSeq')}")
        return True

    def _traffic_pose(self) -> Optional[tuple[float, float, float]]:
        pose, status = self._lookup_map_pose()
        if status != "LOCALIZED" or pose is None:
            return None
        return float(pose["x"]), float(pose["y"]), float(pose["yaw"])

    def _publish_traffic_progress(self, payload: dict[str, Any]) -> None:
        import uuid

        body = {
            "schemaVersion": 1,
            "messageId": str(uuid.uuid4()),
            "robotCode": self._robot_code,
            "bootId": self._seq.boot_id,
            **payload,
        }
        if not self.mqtt.publish(self.topic("traffic_progress"), body, qos=1):
            self.get_logger().warning("traffic_progress queue full; message dropped")

    def _route_pose_xy(self) -> Optional[tuple[float, float]]:
        pose, status = self._lookup_map_pose()
        if status != "LOCALIZED" or pose is None:
            return None
        return float(pose["x"]), float(pose["y"])

    def topic(self, leaf: str) -> str:
        return f"{self._topic_prefix}/robots/{self._robot_code}/{leaf}"

    def destroy_node(self) -> bool:
        stop = getattr(self, "_fleet_uplink_stop", None)
        if stop is not None:
            stop.set()
        if self._teleop is not None:
            self._teleop.stop()
        if self._button_input is not None:
            self._button_input.close()
        self.mqtt.stop()
        return super().destroy_node()

    def _fleet_uplink_loop(self, period_sec: float, tick, label: str) -> None:
        """Wall-clock uplink; survives SingleThreadedExecutor stalls during Nav2 waits."""
        period = max(0.05, float(period_sec))
        while not self._fleet_uplink_stop.wait(period):
            try:
                tick()
            except Exception as exc:  # noqa: BLE001 — keep loop alive
                try:
                    self.get_logger().warning(f"fleet {label} tick failed: {exc!r}")
                except Exception:  # noqa: BLE001
                    pass

    # ------------------------------------------------------------------ command / teleop / interaction downlink

    def _publish_interaction_confirm(self, payload: dict[str, Any]) -> None:
        if not self.mqtt.publish(self.topic("interaction_confirm"), payload, qos=1):
            self.get_logger().warning("interaction_confirm queue full; message dropped")

    def _on_button_timer(self) -> None:
        if self._button_input is None or self._button_detector is None:
            return
        level = self._button_input.read_level()
        if level is None:
            return
        t_ms = int(time.monotonic() * 1000)
        if self._button_detector.sample(level, t_ms):
            self._interaction.set_monotonic(time.monotonic())
            self._interaction.on_button_press()
        self._interaction.tick(time.monotonic())

    def _on_mqtt_message(self, topic: str, body: dict) -> None:
        # MQTT network thread: chỉ enqueue, không xử lý.
        if topic == self.topic("command"):
            self._inbox.put(("command", body))
        elif topic == self.topic("teleop") and self._teleop is not None:
            self._inbox.put(("teleop", body))
        elif topic == self.topic("interaction_request"):
            self._inbox.put(("interaction_request", body))
        elif topic == self.topic("interaction_result"):
            self._inbox.put(("interaction_result", body))
        else:
            return
        self._inbox_signal.trigger()

    def _drain_inbox(self) -> None:
        while True:
            try:
                kind, body = self._inbox.get_nowait()
            except queue.Empty:
                return
            if kind == "command":
                self._commands.router.handle_message(body)
            elif kind == "teleop" and self._teleop is not None:
                self._teleop.handle(body)
            elif kind == "interaction_request":
                self._handle_interaction_request(body)
            elif kind == "interaction_result":
                self._handle_interaction_result(body)

    def _handle_interaction_request(self, body: dict) -> None:
        try:
            req = parse_interaction_request(body, local_robot_code=self._robot_code)
        except InteractionContractError as exc:
            self.get_logger().warning(f"interaction_request rejected: {exc}")
            return
        self._interaction.handle_request(req)

    def _handle_interaction_result(self, body: dict) -> None:
        try:
            result = parse_interaction_result(body, local_robot_code=self._robot_code)
        except InteractionContractError as exc:
            self.get_logger().warning(f"interaction_result rejected: {exc}")
            return
        self._interaction.handle_result(result)

    def _on_scan(self, msg: LaserScan) -> None:
        frame = msg.header.frame_id
        mount = self._laser_mount.get(frame)
        if mount is None:
            # Static mount (geometry.yaml: laser yaw +pi/2): look it up once per frame id.
            try:
                tf = self._tf_buffer.lookup_transform(self._base_frame, frame, Time()).transform
            except TransformException:
                return
            q = tf.rotation
            mount = (float(tf.translation.x), float(tf.translation.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))
            self._laser_mount[frame] = mount
        points = scan_to_base_points(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, *mount
        )
        self._gaps = corridor_gaps(points, self._footprint)
        self._gaps_at = time.monotonic()

    def _fresh_gaps(self) -> Optional[Gaps]:
        if self._gaps is None or time.monotonic() - self._gaps_at > self._scan_max_age:
            return None  # assist cannot see → no linear motion
        return self._gaps

    def _publish_cmd_vel(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    # ------------------------------------------------------------------ ROS inputs

    def _on_odom(self, msg: Odometry) -> None:
        self._lin_vel = float(msg.twist.twist.linear.x)
        self._ang_vel = float(msg.twist.twist.angular.z)

    def _on_amcl_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        pass

    def _on_goal_status(self, msg) -> None:
        self._nav_tracker.update_from_status_list([int(s.status) for s in msg.status_list])

    # ------------------------------------------------------------------ uplink

    def _on_heartbeat_timer(self) -> None:
        seq = self._seq.next_heartbeat()
        payload = build_heartbeat(
            robot_code=self._robot_code,
            boot_id=self._seq.boot_id,
            sequence=seq,
        )
        self.mqtt.publish(self.topic("heartbeat"), payload)

    def _safety_pose_age(self) -> Optional[float]:
        """Refresh map→base TF then return age (seconds); None if transform missing."""
        self._traffic_pose()
        return self._last_pose_age_sec

    def _safety_scan_age(self) -> Optional[float]:
        """Age of last LaserScan used by drive-assist; 0 when scan not subscribed (Active FollowPath)."""
        gaps_at = getattr(self, "_gaps_at", None)
        if gaps_at is None or gaps_at <= 0.0:
            return 0.0
        return max(0.0, time.monotonic() - float(gaps_at))

    def _lookup_map_pose(self) -> tuple[Optional[dict[str, float]], str]:
        try:
            when = Time()
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                when,
                timeout=Duration(seconds=0.05),
            )
            stamp = transform.header.stamp
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
            self._last_pose_age_sec = float(age)
            # Negative age = clock jump back; treat as stale (not LOCALIZED).
            if age < 0.0 or age > self._max_transform_age:
                return self._loc_hysteresis.on_stale()

            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            pose = {"x": float(t.x), "y": float(t.y), "yaw": float(yaw)}
            return self._loc_hysteresis.on_good(pose)
        except TransformException:
            self._last_pose_age_sec = None
            return self._loc_hysteresis.on_stale()

    def _navigation_status(self) -> str:
        status = self._commands.move.navigation_status
        if status == nav_state.IDLE and self._nav_tracker.has_active_goal:
            # Goal không do fleet gửi (RViz / CLI) — vẫn phản ánh robot đang chạy.
            return nav_state.NAVIGATING
        return status

    def _on_telemetry_timer(self) -> None:
        pose, localization_status = self._lookup_map_pose()
        self._last_localization = localization_status
        if localization_status != "LOCALIZED":
            pose = None

        seq = self._seq.next_telemetry()
        payload = build_telemetry(
            robot_code=self._robot_code,
            boot_id=self._seq.boot_id,
            sequence=seq,
            map_version_code=self._map_version_code,
            pose=pose,
            battery_percent=self._battery_percent,
            navigation_status=self._navigation_status(),
            localization_status=localization_status,
            linear_velocity=self._lin_vel,
            angular_velocity=self._ang_vel,
            current_command_id=self._commands.move.active_command_id,
            error_code=None,
        )
        self.mqtt.publish(self.topic("telemetry"), payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotGatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # Jazzy tự shutdown context khi nhận SIGINT — shutdown lần hai sẽ raise RCLError.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
