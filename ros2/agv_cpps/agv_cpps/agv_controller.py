"""
agv_controller.py
────────────────────────────────────────────────────────────────────────
CPPS Physical Layer: AGV Line-Follower State Machine

Each AGV navigates a fixed line-network graph by following named waypoint
nodes in sequence.  Navigation is time-based dead-reckoning:
  - Each straight leg drives for  distance / LINEAR_SPEED  seconds
  - Each turn drives angular for  angle / ANGULAR_SPEED  seconds
  - No odometry dependency — works without sensor feedback

The route planner builds a node-path through the graph from the current
position to the target, ensuring the robot only travels along real line
segments (no cutting through walls or open floor).

IR sensor behaviour (simulated):
  The robot publishes a /agv_N/ir_sensor topic showing "ON_LINE" or
  "OFF_LINE" based on whether the estimated pose is within LINE_HALF_WIDTH
  of any registered line segment.  This drives the line-follower P-
  controller that applies a small corrective angular.z to keep the robot
  centred on the line.  Speed is reduced when correction is large.

State machine:
  IDLE → GOING_TO_DEPOT → LOADING → GOING_TO_SHOP → UNLOADING
       → RETURNING → IDLE
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String
from agv_interfaces.msg import AGVStatus, ReplenishmentOrder

# ── Tuning ────────────────────────────────────────────────────────────
LINEAR_SPEED    = 0.6    # m/s straight driving
ANGULAR_SPEED   = 0.8    # rad/s turning
LOAD_TIME_SEC   = 4.0
UNLOAD_TIME_SEC = 4.0
CTRL_DT         = 0.05   # 20 Hz control loop
LINE_HALF_WIDTH = 0.18   # metres — IR sensor detects line within this band
KP_LINE         = 1.2    # P-gain for line-follow cross-track correction

# ── Home docks ────────────────────────────────────────────────────────
DOCK_HOME = {
    "agv_1": (-6.0, -8.8),
    "agv_2": ( 0.0, -8.8),
    "agv_3": ( 6.0, -8.8),
}

# ── Line network node positions ────────────────────────────────────────
# Every named node is a physical point on the painted line grid.
NODES = {
    # Dock home positions
    "dock_1": (-6.0, -8.8),
    "dock_2": ( 0.0, -8.8),
    "dock_3": ( 6.0, -8.8),
    # South junction row  (y = -5)
    "j_sw":   (-6.0, -5.0),
    "j_sc":   ( 0.0, -5.0),
    "j_se":   ( 6.0, -5.0),
    "j_ws":   (-12.0,-5.0),
    "j_es":   ( 12.0,-5.0),
    # Mid row  (y = 0)
    "j_wm":   (-12.0, 0.0),
    "j_em":   ( 12.0, 0.0),
    "j_shop_m":( 8.0,  0.0),
    # North junction row  (y = +5)
    "j_wn":   (-12.0, 5.0),
    "j_nw":   (-6.0,  5.0),
    "j_nc":   ( 0.0,  5.0),
    "j_ne":   ( 6.0,  5.0),
    "j_en":   ( 12.0, 5.0),
    "j_shop_n":( 8.0,  5.0),
    "j_shop_s":( 8.0, -5.0),
    # Depot stops
    "depot_a":(-12.0,  8.0),
    "depot_b":( 12.0,  8.0),
    "depot_c":(-12.0, -3.0),
    # Shop stops (on shop spine, x=8)
    "shop_1": ( 8.0,  5.0),   # shop_1 at y=+5
    "shop_2": ( 8.0,  0.0),   # shop_2 at y=0
    "shop_3": ( 8.0, -5.0),   # shop_3 at y=-5
}

# ── Line segment definitions (for IR sensor simulation) ───────────────
# Each entry: (x1,y1, x2,y2) — the painted line goes between these two points.
LINE_SEGMENTS = [
    # Dock approach lanes (vertical)
    (-6,-8.8, -6,-5), (0,-8.8, 0,-5), (6,-8.8, 6,-5),
    # South row
    (-12,-5, 12,-5),
    # West spine
    (-12,-5, -12,5),
    # East spine
    (12,-5, 12,5),
    # North row
    (-12,5, 12,5),
    # Depot spurs
    (-12,5, -12,8), (12,5, 12,8),
    # Mid horizontal
    (-12,0, 8,0),
    # Shop spine
    (8,-5, 8,5),
    # Shop connectors
    (6,5, 8,5), (6,-5, 8,-5),
]

# ── Graph adjacency (which nodes connect directly on a line) ──────────
GRAPH = {
    "dock_1":   ["j_sw"],
    "dock_2":   ["j_sc"],
    "dock_3":   ["j_se"],
    "j_sw":     ["dock_1","j_sc","j_ws","j_nw"],
    "j_sc":     ["dock_2","j_sw","j_se","j_nc"],
    "j_se":     ["dock_3","j_sc","j_es","j_shop_s","j_ne"],
    "j_ws":     ["j_sw","j_es","j_wm"],
    "j_es":     ["j_se","j_ws","j_em","j_shop_s"],
    "j_wm":     ["j_ws","j_wn","j_shop_m","depot_c"],
    "j_em":     ["j_es","j_en","j_shop_m"],
    "j_shop_m": ["j_wm","j_em","j_shop_n","j_shop_s","shop_2"],
    "j_wn":     ["j_wm","j_nw","depot_a"],
    "j_nw":     ["j_sw","j_wn","j_nc"],
    "j_nc":     ["j_sc","j_nw","j_ne"],
    "j_ne":     ["j_se","j_nc","j_en","j_shop_n"],
    "j_en":     ["j_ne","j_em","depot_b"],
    "j_shop_n": ["j_ne","j_shop_m","shop_1"],
    "j_shop_s": ["j_se","j_es","j_shop_m","shop_3"],
    "depot_a":  ["j_wn"],
    "depot_b":  ["j_en"],
    "depot_c":  ["j_wm"],
    "shop_1":   ["j_shop_n"],
    "shop_2":   ["j_shop_m"],
    "shop_3":   ["j_shop_s"],
}

# ── Material depot node names ─────────────────────────────────────────
DEPOT_NODE = {"A": "depot_a", "B": "depot_b", "C": "depot_c"}

# ── Shop node names ───────────────────────────────────────────────────
SHOP_NODE  = {"shop_1": "shop_1", "shop_2": "shop_2", "shop_3": "shop_3"}

# ── Home node per AGV ─────────────────────────────────────────────────
HOME_NODE  = {"agv_1": "dock_1", "agv_2": "dock_2", "agv_3": "dock_3"}


# ── BFS path planner ──────────────────────────────────────────────────
def bfs_path(start_node: str, goal_node: str) -> list[str]:
    """Return ordered list of node names from start to goal (inclusive)."""
    if start_node == goal_node:
        return [start_node]
    from collections import deque
    visited = {start_node}
    queue   = deque([[start_node]])
    while queue:
        path = queue.popleft()
        for nbr in GRAPH.get(path[-1], []):
            if nbr not in visited:
                new_path = path + [nbr]
                if nbr == goal_node:
                    return new_path
                visited.add(nbr)
                queue.append(new_path)
    return [start_node, goal_node]   # fallback direct


def _nearest_node(x: float, y: float) -> str:
    """Snap pose to nearest graph node."""
    best, best_d = "dock_1", 1e9
    for name, (nx, ny) in NODES.items():
        d = math.hypot(x - nx, y - ny)
        if d < best_d:
            best_d = d
            best = name
    return best


def _cross_track_error(px, py, ax, ay, bx, by) -> float:
    """Signed lateral distance from point P to directed line A→B."""
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return 0.0
    # Perpendicular (right-hand positive)
    return ((py - ay) * dx - (px - ax) * dy) / length


def _on_any_line(px, py) -> bool:
    for x1, y1, x2, y2 in LINE_SEGMENTS:
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / (length * length)))
        cx = x1 + t * dx - px
        cy = y1 + t * dy - py
        if math.hypot(cx, cy) <= LINE_HALF_WIDTH:
            return True
    return False


class AGVController(Node):
    def __init__(self):
        super().__init__("agv_controller")
        self.declare_parameter("agv_id", "agv_1")
        self._agv_id: str = self.get_parameter("agv_id").value
        self.get_logger().info(f"AGVController starting for {self._agv_id}")

        # ── State ─────────────────────────────────────────────────────
        self._state  = "IDLE"
        self._order: ReplenishmentOrder | None = None

        # Navigation
        self._path: list[str] = []    # remaining node names to visit
        self._leg_elapsed  = 0.0
        self._leg_duration = 0.0
        self._turning      = False
        self._turn_elapsed = 0.0
        self._turn_dir     = 1.0

        # Wait timer
        self._wait_elapsed  = 0.0
        self._wait_duration = 0.0
        self._waiting       = False

        # Dead-reckoning pose
        hx, hy = DOCK_HOME[self._agv_id]
        self._x   = hx
        self._y   = hy
        self._yaw = math.pi / 2   # initially facing north (+Y)

        # Current leg start (for cross-track error)
        self._leg_start_x = hx
        self._leg_start_y = hy

        # ── ROS interfaces ────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist, f"/model/{self._agv_id}/cmd_vel", 10)
        self._stat_pub   = self.create_publisher(AGVStatus, f"/{self._agv_id}/status", 10)
        self._ir_pub     = self.create_publisher(String, f"/{self._agv_id}/ir_sensor", 10)
        self._inprog_pub = self.create_publisher(ReplenishmentOrder, "/orders/in_progress", 10)
        self._done_pub   = self.create_publisher(ReplenishmentOrder, "/orders/completed", 10)

        self.create_subscription(ReplenishmentOrder, f"/{self._agv_id}/task", self._on_task, 10)

        self.create_timer(CTRL_DT, self._control_loop)
        self.create_timer(0.5,     self._publish_status)

        self._publish_status()   # announce IDLE immediately

    # ── Task callback ─────────────────────────────────────────────────
    def _on_task(self, msg: ReplenishmentOrder):
        if self._state != "IDLE":
            self.get_logger().warn(f"{self._agv_id}: busy ({self._state}), rejecting")
            return
        self._order = msg
        self.get_logger().info(
            f"{self._agv_id}: task order={msg.order_id} mat={msg.material_type} → {msg.shop_id}"
        )
        self._begin_going_to_depot()

    # ── Transitions ───────────────────────────────────────────────────
    def _begin_going_to_depot(self):
        mat  = self._order.material_type
        goal = DEPOT_NODE[mat]
        self._state = "GOING_TO_DEPOT"
        self._plan_route(goal)
        self.get_logger().info(f"{self._agv_id}: → GOING_TO_DEPOT ({goal})")

    def _begin_loading(self):
        self._state         = "LOADING"
        self._waiting       = True
        self._wait_elapsed  = 0.0
        self._wait_duration = LOAD_TIME_SEC
        self._stop()
        self.get_logger().info(f"{self._agv_id}: → LOADING ({LOAD_TIME_SEC}s)")

    def _begin_going_to_shop(self):
        shop = self._order.shop_id
        goal = SHOP_NODE[shop]
        self._state = "GOING_TO_SHOP"
        self._plan_route(goal)
        self.get_logger().info(f"{self._agv_id}: → GOING_TO_SHOP ({goal})")

    def _begin_unloading(self):
        self._state         = "UNLOADING"
        self._waiting       = True
        self._wait_elapsed  = 0.0
        self._wait_duration = UNLOAD_TIME_SEC
        self._stop()
        self._order.status = "in_progress"
        self._inprog_pub.publish(self._order)
        self.get_logger().info(f"{self._agv_id}: → UNLOADING ({UNLOAD_TIME_SEC}s)")

    def _begin_returning(self):
        home = HOME_NODE[self._agv_id]
        self._state = "RETURNING"
        self._order.status = "completed"
        self._done_pub.publish(self._order)
        self._order = None
        self._plan_route(home)
        self.get_logger().info(f"{self._agv_id}: → RETURNING ({home})")

    def _set_idle(self):
        self._state = "IDLE"
        self._stop()
        hx, hy = DOCK_HOME[self._agv_id]
        self._x, self._y = hx, hy
        self.get_logger().info(f"{self._agv_id}: → IDLE (docked)")

    # ── Route planning ────────────────────────────────────────────────
    def _plan_route(self, goal_node: str):
        start = _nearest_node(self._x, self._y)
        self._path = bfs_path(start, goal_node)
        # Remove first node if we're already essentially at it
        if len(self._path) > 1:
            nx, ny = NODES[self._path[0]]
            if math.hypot(self._x - nx, self._y - ny) < 0.3:
                self._path.pop(0)
        self.get_logger().info(f"{self._agv_id}: route {' → '.join(self._path)}")
        self._advance_to_next_node()

    def _advance_to_next_node(self):
        if not self._path:
            return
        gx, gy = NODES[self._path[0]]
        dist = math.hypot(gx - self._x, gy - self._y)
        target_yaw = math.atan2(gy - self._y, gx - self._x)
        angle_err  = math.atan2(
            math.sin(target_yaw - self._yaw),
            math.cos(target_yaw - self._yaw),
        )
        self._leg_start_x = self._x
        self._leg_start_y = self._y
        if abs(angle_err) > 0.12:
            self._turning      = True
            self._turn_elapsed = 0.0
            self._turn_dir     = 1.0 if angle_err > 0 else -1.0
        else:
            self._turning = False
        self._yaw          = target_yaw
        self._leg_elapsed  = 0.0
        self._leg_duration = (dist / LINEAR_SPEED) if dist > 0.05 else 0.0

    # ── Main control loop (20 Hz) ─────────────────────────────────────
    def _control_loop(self):
        # ── Publish simulated IR sensor reading ──
        on_line = _on_any_line(self._x, self._y)
        ir_msg  = String()
        ir_msg.data = "ON_LINE" if on_line else "OFF_LINE"
        self._ir_pub.publish(ir_msg)

        # ── Waiting (LOADING / UNLOADING) ──
        if self._waiting:
            self._wait_elapsed += CTRL_DT
            if self._wait_elapsed >= self._wait_duration:
                self._waiting = False
                if self._state == "LOADING":
                    self._begin_going_to_shop()
                elif self._state == "UNLOADING":
                    self._begin_returning()
            return

        if self._state not in ("GOING_TO_DEPOT", "GOING_TO_SHOP", "RETURNING"):
            return

        # ── Path complete? ──
        if not self._path:
            if self._state == "GOING_TO_DEPOT":
                self._begin_loading()
            elif self._state == "GOING_TO_SHOP":
                self._begin_unloading()
            elif self._state == "RETURNING":
                self._set_idle()
            return

        # ── Turning phase ──
        if self._turning:
            self._turn_elapsed += CTRL_DT
            twist = Twist()
            twist.angular.z = ANGULAR_SPEED * self._turn_dir
            twist.linear.x  = 0.04
            self._cmd_pub.publish(twist)
            turn_time = abs(math.atan2(
                math.sin(self._yaw - math.atan2(
                    NODES[self._path[0]][1] - self._leg_start_y,
                    NODES[self._path[0]][0] - self._leg_start_x)),
                math.cos(self._yaw - math.atan2(
                    NODES[self._path[0]][1] - self._leg_start_y,
                    NODES[self._path[0]][0] - self._leg_start_x))
            )) / ANGULAR_SPEED + 0.3
            if self._turn_elapsed >= turn_time:
                self._turning = False
                self._leg_elapsed = 0.0
            return

        # ── Driving phase with line-follow P-controller ──
        self._leg_elapsed += CTRL_DT

        # Dead-reckoning position update
        self._x += LINEAR_SPEED * CTRL_DT * math.cos(self._yaw)
        self._y += LINEAR_SPEED * CTRL_DT * math.sin(self._yaw)

        # Cross-track error → P correction
        gx, gy = NODES[self._path[0]]
        cte = _cross_track_error(
            self._x, self._y,
            self._leg_start_x, self._leg_start_y,
            gx, gy,
        )
        angular_correction = -KP_LINE * cte
        angular_correction = max(-1.0, min(1.0, angular_correction))
        speed = LINEAR_SPEED * (1.0 - 0.4 * abs(angular_correction))

        twist = Twist()
        twist.linear.x  = speed
        twist.angular.z = angular_correction
        self._cmd_pub.publish(twist)

        # Check arrival at waypoint
        dist_to_goal = math.hypot(gx - self._x, gy - self._y)
        if self._leg_elapsed >= self._leg_duration or dist_to_goal < 0.25:
            # Snap to node
            self._x, self._y = gx, gy
            self._stop()
            node_name = self._path.pop(0)
            self.get_logger().info(f"{self._agv_id}: reached {node_name}")
            if self._path:
                self._advance_to_next_node()

    def _stop(self):
        self._cmd_pub.publish(Twist())

    # ── Status ────────────────────────────────────────────────────────
    def _publish_status(self):
        msg = AGVStatus()
        msg.header      = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.agv_id            = self._agv_id
        msg.state             = self._state
        msg.current_order_id  = self._order.order_id if self._order else ""
        msg.carrying_material = (
            self._order.material_type
            if self._order and self._state in ("GOING_TO_SHOP", "UNLOADING")
            else ""
        )
        msg.battery_pct = 100.0
        msg.pos_x = self._x
        msg.pos_y = self._y
        self._stat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
