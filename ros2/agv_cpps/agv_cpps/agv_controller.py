"""
agv_controller.py
────────────────────────────────────────────────────────────────────────
CPPS Physical Layer: AGV Line-Follower with 5-channel IR Sensor

Pose tracking  ← KEY DESIGN DECISION
─────────────
Primary:  /world/agv_factory/pose/info  (tf2_msgs/TFMessage bridged from
          ignition.msgs.Pose_V).  Ignition publishes EXACT world-frame
          poses here for every model.  Zero drift, zero frame-transform
          ambiguity.  Each TFMessage contains transforms for ALL models;
          we filter by child_frame_id == "<agv_id>/base_link".

Fallback: If pose/info hasn't arrived yet, dead-reckon from spawn.

Odometry is kept subscribed only for its twist (velocity) data; the
pose portion of odometry is NOT used.

5-channel IR sensor
───────────────────
5 virtual sensors span the robot front (±7 cm, ±3.5 cm, 0) in robot y.
Each is projected to world frame using the current world pose.
Proximity to any LINE_SEG within LINE_HW → sensor ON.
Weighted average of active sensors → lateral error [-2, +2].
  err > 0 → robot drifted right → steer left (+angular.z)
  err < 0 → robot drifted left  → steer right (-angular.z)

Control law
───────────
  Turning phase (|head_err| > TURN_THRESH):
      angular = KP_HEAD * head_err   (heading only, IR disabled)
      speed   = CREEP_SPEED
  Approaching node (dist < SLOW_DIST):
      angular = KP_HEAD * head_err + KP_IR * ir_err
      speed   = SLOW_SPEED
  Cruising:
      angular = KP_HEAD * head_err + KP_IR * ir_err
      speed   = SPEED_MAX * (1 - SPEED_DAMP * |angular|/MAX_ANG)

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
from tf2_msgs.msg import TFMessage
from agv_interfaces.msg import AGVStatus, ReplenishmentOrder

# ── Tuning ────────────────────────────────────────────────────────────
SPEED_MAX    = 0.50   # m/s cruising speed
CREEP_SPEED  = 0.06   # m/s during junction turns
SLOW_SPEED   = 0.18   # m/s when within SLOW_DIST of next node
SLOW_DIST    = 1.20   # m  — start slowing down
MAX_ANG      = 1.20   # rad/s
SPEED_DAMP   = 0.60
KP_IR        = 0.55   # IR lateral error gain
KP_HEAD      = 2.20   # heading error gain
TURN_THRESH  = 0.22   # rad — use creep+turn above this
IR_THRESH    = 0.35   # rad — suppress IR above this (pure heading turn)
GOAL_TOL     = 0.45   # m   — node arrival tolerance
LINE_HW      = 0.15   # m   — IR detection half-width
CTRL_DT      = 0.05   # s   — 20 Hz

# IR sensor bar (mirrors physical ir_bar visual in model.sdf)
IR_FWD    = 0.24   # m ahead of base_link in robot +x
IR_Y      = [-0.070, -0.035, 0.0, 0.035, 0.070]  # robot frame y offsets
# Weights: robot drifts RIGHT → left sensors (+y) near line → positive → steer left (+ω)
IR_W      = [-2, -1, 0, +1, +2]

LOAD_TIME   = 4.0
UNLOAD_TIME = 4.0

# ── Spawn poses (world frame) ─────────────────────────────────────────
SPAWN = {
    "agv_1": (-6.0, -8.8, math.pi / 2),
    "agv_2": ( 0.0, -8.8, math.pi / 2),
    "agv_3": ( 6.0, -8.8, math.pi / 2),
}

# ── Node positions ────────────────────────────────────────────────────
NODES = {
    "dock_1":   (-6.0, -8.8), "dock_2":   ( 0.0, -8.8), "dock_3":   ( 6.0, -8.8),
    "j_sw":     (-6.0, -5.0), "j_sc":     ( 0.0, -5.0), "j_se":     ( 6.0, -5.0),
    "j_ws":     (-12.0,-5.0), "j_es":     ( 12.0,-5.0),
    "j_wm":     (-12.0, 0.0), "j_em":     ( 12.0, 0.0), "j_shop_m": (  8.0, 0.0),
    "j_wn":     (-12.0, 5.0), "j_nw":     ( -6.0, 5.0), "j_nc":     (  0.0, 5.0),
    "j_ne":     (  6.0, 5.0), "j_en":     ( 12.0, 5.0),
    "j_shop_n": (  8.0, 5.0), "j_shop_s": (  8.0,-5.0),
    "depot_a":  (-12.0, 8.0), "depot_b":  ( 12.0, 8.0), "depot_c":  (-12.0,-3.0),
    "shop_1":   (  8.0, 5.0), "shop_2":   (  8.0, 0.0), "shop_3":   (  8.0,-5.0),
}

# ── Line segments ─────────────────────────────────────────────────────
LINE_SEGS = [
    (-6.0,-8.8,  -6.0,-5.0), ( 0.0,-8.8,  0.0,-5.0), ( 6.0,-8.8,  6.0,-5.0),
    (-12.0,-5.0, 12.0,-5.0),
    (-12.0,-5.0,-12.0, 5.0), ( 12.0,-5.0, 12.0, 5.0),
    (-12.0, 5.0, 12.0, 5.0),
    (-12.0, 5.0,-12.0, 8.0), ( 12.0, 5.0, 12.0, 8.0),
    (-12.0, 0.0,  8.0, 0.0), (  8.0, 0.0, 12.0, 0.0),
    (  8.0,-5.0,  8.0, 5.0),
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
    "depot_a":  ["j_wn"], "depot_b": ["j_en"], "depot_c": ["j_wm"],
    "shop_1":   ["j_shop_n"], "shop_2": ["j_shop_m"], "shop_3": ["j_shop_s"],
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


def _pt_seg_dist(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx*dx + dy*dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / L2))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))


def _ir_reading(wx, wy, wyaw) -> tuple:
    """Returns (ir_err, on_count). ir_err in [-2,+2]."""
    cy, sy = math.cos(wyaw), math.sin(wyaw)
    hits = []
    for iy, iw in zip(IR_Y, IR_W):
        sx  = wx + IR_FWD * cy - iy * sy
        syw = wy + IR_FWD * sy + iy * cy
        on  = any(_pt_seg_dist(sx, syw, *seg) <= LINE_HW for seg in LINE_SEGS)
        hits.append((on, iw))
    cnt = sum(1 for on, _ in hits if on)
    if cnt == 0:
        return 0.0, 0
    return sum(iw for on, iw in hits if on) / cnt, cnt


class AGVController(Node):
    def __init__(self):
        super().__init__("agv_controller")
        self.declare_parameter("agv_id", "agv_1")
        self._id: str = self.get_parameter("agv_id").value

        sx, sy, syaw = SPAWN[self._id]
        self._x   = sx
        self._y   = sy
        self._yaw = syaw
        self._have_pose = False   # True once pose/info arrives

        # State machine
        self._state  = "IDLE"
        self._order  = None
        self._path:  list = []
        self._waiting       = False
        self._wait_elapsed  = 0.0
        self._wait_duration = 0.0

        # ROS interfaces
        self._cmd  = self.create_publisher(Twist,              f"/model/{self._id}/cmd_vel",   10)
        self._stat = self.create_publisher(AGVStatus,          f"/{self._id}/status",           10)
        self._irp  = self.create_publisher(String,             f"/{self._id}/ir_sensor",        10)
        self._prog = self.create_publisher(ReplenishmentOrder, "/orders/in_progress",           10)
        self._done = self.create_publisher(ReplenishmentOrder, "/orders/completed",             10)

        self.create_subscription(ReplenishmentOrder, f"/{self._id}/task",  self._on_task,  10)
        self.create_subscription(Odometry,  f"/model/{self._id}/odometry", self._on_odom,  20)
        self.create_subscription(TFMessage, f"/model/{self._id}/pose",      self._on_pose,  20)

        self.create_timer(CTRL_DT, self._ctrl_loop)
        self.create_timer(0.5,     self._pub_status)
        self._pub_status()
        self.get_logger().info(f"AGVController ready: {self._id}")

    # ── Pose from world pose/info topic (primary) ─────────────────────
    def _on_pose(self, msg: TFMessage):
        # Ignition publishes <model_name>/base_link as child_frame_id
        # /model/agv_N/pose publishes child_frame_id as bare link name e.g. "base_link"
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                self._x   = tf.transform.translation.x
                self._y   = tf.transform.translation.y
                self._yaw = _yaw_from_q(tf.transform.rotation)
                self._have_pose = True
                return

    # ── Odometry (NOT used for pose — kept only so bridge topic is consumed) ──
    def _on_odom(self, msg: Odometry):
        pass  # pose comes from /model/<id>/pose via _on_pose

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
        self._waiting = True; self._wait_elapsed = 0.0; self._wait_duration = LOAD_TIME
        self._stop()
        self.get_logger().info(f"{self._id}: → LOADING")

    def _go_to_shop(self):
        self._state = "GOING_TO_SHOP"
        self._plan(SHOP_NODE[self._order.shop_id])
        self.get_logger().info(f"{self._id}: → GOING_TO_SHOP")

    def _start_unloading(self):
        self._state = "UNLOADING"
        self._waiting = True; self._wait_elapsed = 0.0; self._wait_duration = UNLOAD_TIME
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
        self.get_logger().info(f"{self._id}: → IDLE")

    # ── Route planning ────────────────────────────────────────────────
    def _plan(self, goal: str):
        start = _nearest_node(self._x, self._y)
        self._path = bfs_path(start, goal)
        # Skip start node if already there
        if len(self._path) > 1:
            nx, ny = NODES[self._path[0]]
            if math.hypot(self._x - nx, self._y - ny) < GOAL_TOL:
                self._path.pop(0)
        self.get_logger().info(f"{self._id}: route [{' → '.join(self._path)}]")

    # ── Control loop (20 Hz) ──────────────────────────────────────────
    def _ctrl_loop(self):
        ir_err, on_cnt = _ir_reading(self._x, self._y, self._yaw)
        ir_msg = String()
        ir_msg.data = f"ON:{on_cnt}" if on_cnt else "OFF"
        self._irp.publish(ir_msg)

        # Do not move until we have a confirmed world pose
        if not self._have_pose:
            return

        if self._waiting:
            self._wait_elapsed += CTRL_DT
            if self._wait_elapsed >= self._wait_duration:
                self._waiting = False
                if   self._state == "LOADING":   self._go_to_shop()
                elif self._state == "UNLOADING": self._return_home()
            return

        if self._state not in ("GOING_TO_DEPOT", "GOING_TO_SHOP", "RETURNING"):
            return

        # Path empty check
        if not self._path:
            self._stop()
            if   self._state == "GOING_TO_DEPOT": self._start_loading()
            elif self._state == "GOING_TO_SHOP":  self._start_unloading()
            elif self._state == "RETURNING":      self._set_idle()
            return

        gx, gy = NODES[self._path[0]]
        dist = math.hypot(gx - self._x, gy - self._y)

        # Arrived at node?
        if dist < GOAL_TOL:
            arrived = self._path.pop(0)
            self.get_logger().info(f"{self._id}: ✓ {arrived}")
            if not self._path:
                self._stop()
                if   self._state == "GOING_TO_DEPOT": self._start_loading()
                elif self._state == "GOING_TO_SHOP":  self._start_unloading()
                elif self._state == "RETURNING":      self._set_idle()
                return
            gx, gy = NODES[self._path[0]]
            dist   = math.hypot(gx - self._x, gy - self._y)

        # Heading error
        target_yaw = math.atan2(gy - self._y, gx - self._x)
        head_err = math.atan2(
            math.sin(target_yaw - self._yaw),
            math.cos(target_yaw - self._yaw),
        )

        # Angular command — IR disabled during large turns
        if abs(head_err) > IR_THRESH:
            angular = KP_HEAD * head_err
        else:
            angular = KP_HEAD * head_err + KP_IR * ir_err
        angular = max(-MAX_ANG, min(MAX_ANG, angular))

        # Speed
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
