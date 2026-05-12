"""
agv_controller.py
────────────────────────────────────────────────────────────────────────
CPPS Physical Layer: AGV Line-Follower with 5-channel IR Sensor

Pose tracking
─────────────
Ignition diff-drive publishes odometry in a local "odom" frame that
starts at (0,0,0) when the robot spawns.  We convert to world frame:

    world_x = spawn_x + odom_x * cos(spawn_yaw) - odom_y * sin(spawn_yaw)
    world_y = spawn_y + odom_x * sin(spawn_yaw) + odom_y * cos(spawn_yaw)
    world_yaw = spawn_yaw + odom_yaw

Until the first odometry message arrives we dead-reckon from spawn.

5-channel IR sensor
───────────────────
5 virtual sensors are spaced 3.5 cm apart across the robot's front
undercarriage (mirroring the physical ir_bar visual).  Each cycle we
project each sensor into world frame and check whether it lies within
LINE_HALF_WIDTH of any painted line segment.  The weighted centre of
active sensors gives a lateral error in [-2, +2].

Control strategy
────────────────
  angular.z = KP_IR   * ir_error          (line-follow)
             + KP_HEAD * heading_error      (bearing to next node)
  linear.x  = SPEED_MAX * (1 - SPEED_DAMP * |angular.z| / MAX_ANG)

Clamp angular.z to ±MAX_ANG.  When heading_error > TURN_THRESH, stop
translating (pure in-place turn) so the robot aligns before driving.

State machine: IDLE → GOING_TO_DEPOT → LOADING → GOING_TO_SHOP
             → UNLOADING → RETURNING → IDLE
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from agv_interfaces.msg import AGVStatus, ReplenishmentOrder

# ── Tuning ────────────────────────────────────────────────────────────
SPEED_MAX       = 0.50   # m/s forward speed
CREEP_SPEED     = 0.06   # m/s minimum creep during turns
SLOW_SPEED      = 0.18   # m/s approach speed near a node
SLOW_DIST       = 1.20   # m  — start slowing within this distance of next node
MAX_ANG         = 1.20   # rad/s max angular speed
SPEED_DAMP      = 0.60   # how much angular correction damps forward speed
KP_IR           = 0.55   # gain on IR lateral error
KP_HEAD         = 2.20   # gain on heading error
TURN_THRESH     = 0.22   # rad — switch to creep+turn mode above this
IR_THRESH       = 0.35   # rad — disable IR blending when heading error exceeds this
GOAL_TOL        = 0.45   # m — arrival tolerance (larger = robust to odom drift)
LINE_HW         = 0.15   # m — IR detection half-width (wider = more robust)
CTRL_DT         = 0.05   # s  — 20 Hz

# IR sensor bar geometry (robot-local, relative to base_link centre)
# 5 sensors spaced 3.5 cm apart along y-axis, 24 cm ahead (front of robot)
IR_FORWARD_OFFSET = 0.24   # m ahead of base_link origin
IR_SENSOR_Y = [-0.070, -0.035, 0.0, 0.035, 0.070]  # robot frame: +y=left, -y=right
# When robot drifts right: left sensors (positive y) near line → positive weights fire
# positive ir_err → positive angular.z → CCW = turn left → correct
IR_WEIGHTS  = [-2,     -1,     0,   +1,    +2]      # left sensors=positive, right=negative

LOAD_TIME       = 4.0
UNLOAD_TIME     = 4.0

# ── Home spawn positions ──────────────────────────────────────────────
SPAWN = {
    "agv_1": (-6.0, -8.8, math.pi / 2),   # facing north
    "agv_2": ( 0.0, -8.8, math.pi / 2),
    "agv_3": ( 6.0, -8.8, math.pi / 2),
}

# ── Line network node positions (world frame) ─────────────────────────
NODES = {
    "dock_1":    (-6.0, -8.8),
    "dock_2":    ( 0.0, -8.8),
    "dock_3":    ( 6.0, -8.8),
    "j_sw":      (-6.0, -5.0),
    "j_sc":      ( 0.0, -5.0),
    "j_se":      ( 6.0, -5.0),
    "j_ws":      (-12.0,-5.0),
    "j_es":      ( 12.0,-5.0),
    "j_wm":      (-12.0, 0.0),
    "j_em":      ( 12.0, 0.0),
    "j_shop_m":  (  8.0, 0.0),
    "j_wn":      (-12.0, 5.0),
    "j_nw":      ( -6.0, 5.0),
    "j_nc":      (  0.0, 5.0),
    "j_ne":      (  6.0, 5.0),
    "j_en":      ( 12.0, 5.0),
    "j_shop_n":  (  8.0, 5.0),
    "j_shop_s":  (  8.0,-5.0),
    "depot_a":   (-12.0, 8.0),
    "depot_b":   ( 12.0, 8.0),
    "depot_c":   (-12.0,-3.0),
    "shop_1":    (  8.0, 5.0),
    "shop_2":    (  8.0, 0.0),
    "shop_3":    (  8.0,-5.0),
}

# ── Painted line segments (x1,y1, x2,y2) ────────────────────────────
LINE_SEGS = [
    (-6.0,-8.8,  -6.0,-5.0),   # dock 1 lane
    ( 0.0,-8.8,   0.0,-5.0),   # dock 2 lane
    ( 6.0,-8.8,   6.0,-5.0),   # dock 3 lane
    (-12.0,-5.0, 12.0,-5.0),   # south row
    (-12.0,-5.0,-12.0, 5.0),   # west spine
    ( 12.0,-5.0, 12.0, 5.0),   # east spine
    (-12.0, 5.0, 12.0, 5.0),   # north row
    (-12.0, 5.0,-12.0, 8.0),   # depot-A spur
    ( 12.0, 5.0, 12.0, 8.0),   # depot-B spur
    (-12.0, 0.0,  8.0, 0.0),   # mid horizontal
    (  8.0, 0.0, 12.0, 0.0),   # east-mid connector
    (  8.0,-5.0,  8.0, 5.0),   # shop spine
]

# ── Graph ─────────────────────────────────────────────────────────────
GRAPH = {
    "dock_1":   ["j_sw"],
    "dock_2":   ["j_sc"],
    "dock_3":   ["j_se"],
    "j_sw":     ["dock_1","j_sc","j_ws"],
    "j_sc":     ["dock_2","j_sw","j_se"],
    "j_se":     ["dock_3","j_sc","j_es","j_shop_s"],
    "j_ws":     ["j_sw","j_es","j_wm"],
    "j_es":     ["j_se","j_ws","j_em","j_shop_s"],
    "j_wm":     ["j_ws","j_wn","j_shop_m","depot_c"],
    "j_em":     ["j_es","j_en","j_shop_m"],
    "j_shop_m": ["j_wm","j_em","j_shop_n","j_shop_s","shop_2"],
    "j_wn":     ["j_wm","j_nw","depot_a"],
    "j_nw":     ["j_wn","j_nc"],
    "j_nc":     ["j_nw","j_ne"],
    "j_ne":     ["j_nc","j_en","j_shop_n"],
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

DEPOT_NODE = {"A": "depot_a", "B": "depot_b", "C": "depot_c"}
SHOP_NODE  = {"shop_1": "shop_1", "shop_2": "shop_2", "shop_3": "shop_3"}
HOME_NODE  = {"agv_1": "dock_1", "agv_2": "dock_2", "agv_3": "dock_3"}


# ── Helpers ───────────────────────────────────────────────────────────
def bfs_path(start: str, goal: str) -> list:
    if start == goal:
        return [start]
    visited = {start}
    queue = deque([[start]])
    while queue:
        path = queue.popleft()
        for nbr in GRAPH.get(path[-1], []):
            if nbr not in visited:
                np = path + [nbr]
                if nbr == goal:
                    return np
                visited.add(nbr)
                queue.append(np)
    return [start, goal]


def _yaw_from_q(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _nearest_node(x: float, y: float) -> str:
    best, bd = "dock_1", 1e9
    for n, (nx, ny) in NODES.items():
        d = math.hypot(x - nx, y - ny)
        if d < bd:
            bd, best = d, n
    return best


def _pt_to_seg_dist(px, py, ax, ay, bx, by) -> float:
    """Perpendicular distance from point to line segment (unsigned)."""
    dx, dy = bx - ax, by - ay
    L2 = dx*dx + dy*dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / L2))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))


def _ir_sensor_reading(world_x, world_y, world_yaw) -> tuple:
    """
    Returns (ir_error, on_line_count).
    ir_error  in [-2.0, +2.0] — weighted lateral error from 5 sensors.
              Negative = line is to the LEFT → turn left (negative angular.z).
              Positive = line is to the RIGHT → turn right (positive angular.z).
    on_line_count = number of sensors detecting the line (0..5).
    """
    cos_y = math.cos(world_yaw)
    sin_y = math.sin(world_yaw)
    hits = []
    for sy, w in zip(IR_SENSOR_Y, IR_WEIGHTS):
        # Sensor position in world frame
        sx = world_x + IR_FORWARD_OFFSET * cos_y - sy * sin_y
        sy_w = world_y + IR_FORWARD_OFFSET * sin_y + sy * cos_y
        on = any(_pt_to_seg_dist(sx, sy_w, *seg) <= LINE_HW for seg in LINE_SEGS)
        hits.append((on, w))

    on_count = sum(1 for on, _ in hits if on)
    if on_count == 0:
        return 0.0, 0
    weighted = sum(w for on, w in hits if on)
    return weighted / on_count, on_count


class AGVController(Node):
    def __init__(self):
        super().__init__("agv_controller")
        self.declare_parameter("agv_id", "agv_1")
        self._id: str = self.get_parameter("agv_id").value

        sx, sy, syaw = SPAWN[self._id]
        self._spawn_x   = sx
        self._spawn_y   = sy
        self._spawn_yaw = syaw

        # World-frame pose (updated from odometry)
        self._x   = sx
        self._y   = sy
        self._yaw = syaw
        self._have_odom = False

        # State machine
        self._state  = "IDLE"
        self._order  = None
        self._path   = []          # remaining nodes
        self._leg_start_x = sx
        self._leg_start_y = sy

        # Wait timer (LOADING / UNLOADING)
        self._waiting       = False
        self._wait_elapsed  = 0.0
        self._wait_duration = 0.0

        # ROS interfaces
        self._cmd  = self.create_publisher(Twist,               f"/model/{self._id}/cmd_vel",    10)
        self._stat = self.create_publisher(AGVStatus,           f"/{self._id}/status",            10)
        self._ir   = self.create_publisher(String,              f"/{self._id}/ir_sensor",         10)
        self._prog = self.create_publisher(ReplenishmentOrder,  "/orders/in_progress",            10)
        self._done = self.create_publisher(ReplenishmentOrder,  "/orders/completed",              10)

        self.create_subscription(ReplenishmentOrder, f"/{self._id}/task",
                                 self._on_task, 10)
        self.create_subscription(Odometry,           f"/model/{self._id}/odometry",
                                 self._on_odom, 20)

        self.create_timer(CTRL_DT, self._ctrl_loop)
        self.create_timer(0.5,     self._pub_status)
        self._pub_status()
        self.get_logger().info(f"AGVController ready: {self._id}")

    # ── Odometry ──────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        """
        Ignition diff-drive odom frame origin = robot spawn pose.
        Convert to world frame:
            world = spawn_pos + R(spawn_yaw) * odom_pos
            world_yaw = spawn_yaw + odom_yaw
        """
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        ow = _yaw_from_q(msg.pose.pose.orientation)

        cy = math.cos(self._spawn_yaw)
        sy = math.sin(self._spawn_yaw)

        self._x   = self._spawn_x + ox * cy - oy * sy
        self._y   = self._spawn_y + ox * sy + oy * cy
        self._yaw = self._spawn_yaw + ow
        self._have_odom = True

    # ── Task ──────────────────────────────────────────────────────────
    def _on_task(self, msg: ReplenishmentOrder):
        if self._state != "IDLE":
            self.get_logger().warn(f"{self._id}: busy ({self._state}), ignoring task")
            return
        self._order = msg
        self.get_logger().info(
            f"{self._id}: task={msg.order_id} mat={msg.material_type} → {msg.shop_id}"
        )
        self._go_to_depot()

    # ── State transitions ─────────────────────────────────────────────
    def _go_to_depot(self):
        self._state = "GOING_TO_DEPOT"
        self._plan(DEPOT_NODE[self._order.material_type])
        self.get_logger().info(f"{self._id}: → GOING_TO_DEPOT")

    def _start_loading(self):
        self._state = "LOADING"
        self._waiting = True
        self._wait_elapsed = 0.0
        self._wait_duration = LOAD_TIME
        self._stop()
        self.get_logger().info(f"{self._id}: → LOADING")

    def _go_to_shop(self):
        self._state = "GOING_TO_SHOP"
        self._plan(SHOP_NODE[self._order.shop_id])
        self.get_logger().info(f"{self._id}: → GOING_TO_SHOP")

    def _start_unloading(self):
        self._state = "UNLOADING"
        self._waiting = True
        self._wait_elapsed = 0.0
        self._wait_duration = UNLOAD_TIME
        self._stop()
        self._order.status = "in_progress"
        self._prog.publish(self._order)
        self.get_logger().info(f"{self._id}: → UNLOADING")

    def _return_home(self):
        self._state = "RETURNING"
        self._order.status = "completed"
        self._done.publish(self._order)
        self._order = None
        self._plan(HOME_NODE[self._id])
        self.get_logger().info(f"{self._id}: → RETURNING")

    def _set_idle(self):
        self._state = "IDLE"
        self._stop()
        sx, sy, _ = SPAWN[self._id]
        self._x, self._y = sx, sy
        self.get_logger().info(f"{self._id}: → IDLE")

    # ── Route planning ────────────────────────────────────────────────
    def _plan(self, goal: str):
        start = _nearest_node(self._x, self._y)
        self._path = bfs_path(start, goal)
        if len(self._path) > 1:
            nx, ny = NODES[self._path[0]]
            if math.hypot(self._x - nx, self._y - ny) < GOAL_TOL:
                self._path.pop(0)
        self.get_logger().info(
            f"{self._id}: route [{' → '.join(self._path)}]"
        )
        self._leg_start_x = self._x
        self._leg_start_y = self._y

    # ── Control loop (20 Hz) ──────────────────────────────────────────
    def _ctrl_loop(self):
        # ── IR sensor reading (always compute for topic publish) ──
        ir_err, on_count = _ir_sensor_reading(self._x, self._y, self._yaw)
        ir_msg = String()
        ir_msg.data = f"ON_LINE:{on_count}" if on_count > 0 else "OFF_LINE"
        self._ir.publish(ir_msg)

        # ── Dead-reckon if odometry hasn't arrived yet ──
        if not self._have_odom:
            self._x += SPEED_MAX * 0.2 * CTRL_DT * math.cos(self._yaw)
            self._y += SPEED_MAX * 0.2 * CTRL_DT * math.sin(self._yaw)

        # ── Wait states ──
        if self._waiting:
            self._wait_elapsed += CTRL_DT
            if self._wait_elapsed >= self._wait_duration:
                self._waiting = False
                if   self._state == "LOADING":   self._go_to_shop()
                elif self._state == "UNLOADING": self._return_home()
            return

        if self._state not in ("GOING_TO_DEPOT", "GOING_TO_SHOP", "RETURNING"):
            return

        # ── Path finished ──
        if not self._path:
            if   self._state == "GOING_TO_DEPOT": self._start_loading()
            elif self._state == "GOING_TO_SHOP":  self._start_unloading()
            elif self._state == "RETURNING":      self._set_idle()
            return

        # ── Next waypoint ──
        gx, gy = NODES[self._path[0]]
        dist = math.hypot(gx - self._x, gy - self._y)

        # ── Arrived? ──
        if dist < GOAL_TOL:
            arrived = self._path.pop(0)
            self.get_logger().info(f"{self._id}: ✓ {arrived}")
            self._leg_start_x = self._x
            self._leg_start_y = self._y
            # If path is now empty, handle state transition and stop
            if not self._path:
                self._stop()
                if   self._state == "GOING_TO_DEPOT": self._start_loading()
                elif self._state == "GOING_TO_SHOP":  self._start_unloading()
                elif self._state == "RETURNING":      self._set_idle()
                return
            # Otherwise fall through immediately to start turning toward next node
            gx, gy = NODES[self._path[0]]

        # ── Heading error toward next node ──
        target_yaw = math.atan2(gy - self._y, gx - self._x)
        head_err = math.atan2(
            math.sin(target_yaw - self._yaw),
            math.cos(target_yaw - self._yaw),
        )

        # ── Angular command ──
        # During large heading errors (turning at junction): heading-only, no IR
        # interference — IR on the crossing line would fight the turn.
        # During small heading errors (driving straight): blend IR for line centering.
        if abs(head_err) > IR_THRESH:
            angular = KP_HEAD * head_err
        else:
            angular = KP_HEAD * head_err + KP_IR * ir_err
        angular = max(-MAX_ANG, min(MAX_ANG, angular))

        # Speed control: slow near node to avoid overshoot, creep during turns
        if abs(head_err) > TURN_THRESH:
            speed = CREEP_SPEED
        elif dist < SLOW_DIST:
            speed = SLOW_SPEED
        else:
            speed = SPEED_MAX * (1.0 - SPEED_DAMP * abs(angular) / MAX_ANG)
            speed = max(CREEP_SPEED, speed)

        twist = Twist()
        twist.linear.x  = speed
        twist.angular.z = angular
        self._cmd.publish(twist)

    def _stop(self):
        self._cmd.publish(Twist())

    # ── Status ────────────────────────────────────────────────────────
    def _pub_status(self):
        msg = AGVStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.agv_id            = self._id
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
        self._stat.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
