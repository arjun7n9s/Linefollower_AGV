"""
agv_controller.py  —  CPPS AGV Line-Follower
═══════════════════════════════════════════════════════════════════════

Pose tracking
─────────────
Ignition diff-drive odometry is in a local "odom" frame where the origin
is the robot's spawn position and x-axis aligns with the robot's initial
forward direction.  Convert to world frame via:

    world_x   = spawn_x + odom_x*cos(spawn_yaw) - odom_y*sin(spawn_yaw)
    world_y   = spawn_y + odom_x*sin(spawn_yaw) + odom_y*cos(spawn_yaw)
    world_yaw = spawn_yaw + odom_yaw

All three robots spawn facing north (spawn_yaw = π/2), so:
    world_x = spawn_x − odom_y
    world_y = spawn_y + odom_x
    world_yaw = π/2 + odom_yaw

This is verified mathematically and correct for Ignition Gazebo 6
diff-drive odometry.

5-channel IR sensor
───────────────────
Mirrors the physical ir_bar visual (5 LEDs, 3.5 cm spacing, 24 cm ahead).
Projects each sensor into world frame, checks proximity to LINE_SEGS.
Weighted average → lateral error that steers the robot back onto the line.

Control
───────
Turning phase (|head_err| > TURN_THRESH): heading-only, creep speed.
Approach zone (dist < SLOW_DIST):         blended, slow speed.
Cruise:                                   blended heading+IR, full speed.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from agv_interfaces.msg import AGVStatus, ReplenishmentOrder

# ── Tuning ─────────────────────────────────────────────────────────────
SPEED_MAX    = 0.50
CREEP_SPEED  = 0.06
SLOW_SPEED   = 0.18
SLOW_DIST    = 1.20   # m — start slowing near node
MAX_ANG      = 1.20   # rad/s
SPEED_DAMP   = 0.60
KP_IR        = 0.55
KP_HEAD      = 2.20
TURN_THRESH  = 0.22   # rad — use creep+turn above this
IR_THRESH    = 0.35   # rad — suppress IR correction during large turns
GOAL_TOL     = 0.45   # m
LINE_HW      = 0.15   # m — IR detection half-width
CTRL_DT      = 0.05   # s

# IR sensor positions in robot frame (x=forward, y=left)
IR_FWD  = 0.24
IR_Y    = [-0.070, -0.035, 0.0, 0.035, 0.070]
IR_W    = [-2, -1, 0, +1, +2]   # positive = steer left when left sensors active

LOAD_TIME   = 4.0
UNLOAD_TIME = 4.0

# ── Spawn poses ────────────────────────────────────────────────────────
SPAWN = {
    "agv_1": (-6.0, -8.8, math.pi / 2),
    "agv_2": ( 0.0, -8.8, math.pi / 2),
    "agv_3": ( 6.0, -8.8, math.pi / 2),
}

# ── Node map ───────────────────────────────────────────────────────────
NODES = {
    "dock_1":   (-6.0, -8.8), "dock_2":  ( 0.0, -8.8), "dock_3":   ( 6.0, -8.8),
    "j_sw":     (-6.0, -5.0), "j_sc":   ( 0.0, -5.0),  "j_se":     ( 6.0, -5.0),
    "j_ws":     (-12.0,-5.0), "j_es":   (12.0, -5.0),
    "j_wm":     (-12.0, 0.0), "j_em":   (12.0,  0.0),  "j_shop_m": ( 8.0,  0.0),
    "j_wn":     (-12.0, 5.0), "j_nw":   (-6.0,  5.0),  "j_nc":     ( 0.0,  5.0),
    "j_ne":     ( 6.0,  5.0), "j_en":   (12.0,  5.0),
    "j_shop_n": ( 8.0,  5.0), "j_shop_s":( 8.0,-5.0),
    "depot_a":  (-12.0, 8.0), "depot_b": (12.0, 8.0),  "depot_c":  (-12.0,-3.0),
    "shop_1":   ( 8.0,  5.0), "shop_2":  ( 8.0, 0.0),  "shop_3":   ( 8.0, -5.0),
}

# ── Line segments ──────────────────────────────────────────────────────
LINE_SEGS = [
    (-6.0,-8.8,-6.0,-5.0), (0.0,-8.8,0.0,-5.0), (6.0,-8.8,6.0,-5.0),
    (-12.0,-5.0,12.0,-5.0),
    (-12.0,-5.0,-12.0,5.0), (12.0,-5.0,12.0,5.0),
    (-12.0,5.0,12.0,5.0),
    (-12.0,5.0,-12.0,8.0), (12.0,5.0,12.0,8.0),
    (-12.0,0.0,8.0,0.0),   (8.0,0.0,12.0,0.0),
    (8.0,-5.0,8.0,5.0),
]

# ── Graph ──────────────────────────────────────────────────────────────
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

DEPOT_NODE = {"A":"depot_a","B":"depot_b","C":"depot_c"}
SHOP_NODE  = {"shop_1":"shop_1","shop_2":"shop_2","shop_3":"shop_3"}
HOME_NODE  = {"agv_1":"dock_1","agv_2":"dock_2","agv_3":"dock_3"}


# ── Helpers ────────────────────────────────────────────────────────────
def bfs_path(start, goal):
    if start == goal: return [start]
    visited = {start}; queue = deque([[start]])
    while queue:
        path = queue.popleft()
        for nbr in GRAPH.get(path[-1], []):
            if nbr not in visited:
                np = path + [nbr]
                if nbr == goal: return np
                visited.add(nbr); queue.append(np)
    return [start, goal]

def _yaw_from_q(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def _nearest_node(x, y):
    return min(NODES, key=lambda n: math.hypot(x-NODES[n][0], y-NODES[n][1]))

def _pt_seg(px, py, ax, ay, bx, by):
    dx,dy = bx-ax,by-ay; L2 = dx*dx+dy*dy
    if L2 < 1e-12: return math.hypot(px-ax,py-ay)
    t = max(0.0,min(1.0,((px-ax)*dx+(py-ay)*dy)/L2))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))

def _ir(wx, wy, wyaw):
    cy,sy = math.cos(wyaw),math.sin(wyaw)
    hits = []
    for iy,iw in zip(IR_Y,IR_W):
        sx  = wx + IR_FWD*cy - iy*sy
        syw = wy + IR_FWD*sy + iy*cy
        on  = any(_pt_seg(sx,syw,*seg) <= LINE_HW for seg in LINE_SEGS)
        hits.append((on,iw))
    cnt = sum(1 for on,_ in hits if on)
    if cnt == 0: return 0.0, 0
    return sum(iw for on,iw in hits if on)/cnt, cnt


class AGVController(Node):
    def __init__(self):
        super().__init__("agv_controller")
        self.declare_parameter("agv_id","agv_1")
        self._id = self.get_parameter("agv_id").value

        sx,sy,syaw = SPAWN[self._id]
        self._sx, self._sy, self._syaw = sx, sy, syaw
        self._x, self._y, self._yaw = sx, sy, syaw
        self._have_odom = False

        self._state   = "IDLE"
        self._order   = None
        self._path    = []
        self._waiting = False
        self._wait_t  = 0.0
        self._wait_d  = 0.0

        self._cmd  = self.create_publisher(Twist,             f"/model/{self._id}/cmd_vel", 10)
        self._stat = self.create_publisher(AGVStatus,         f"/{self._id}/status",        10)
        self._irp  = self.create_publisher(String,            f"/{self._id}/ir_sensor",     10)
        self._prog = self.create_publisher(ReplenishmentOrder,"/orders/in_progress",        10)
        self._done = self.create_publisher(ReplenishmentOrder,"/orders/completed",          10)

        self.create_subscription(ReplenishmentOrder,
            f"/{self._id}/task", self._on_task, 10)
        self.create_subscription(Odometry,
            f"/model/{self._id}/odometry", self._on_odom, 20)

        self.create_timer(CTRL_DT, self._ctrl_loop)
        self.create_timer(0.5,     self._pub_status)
        self._pub_status()
        self.get_logger().info(f"{self._id} ready")

    # ── Odometry → world pose ──────────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        ow = _yaw_from_q(msg.pose.pose.orientation)
        cy,sy = math.cos(self._syaw), math.sin(self._syaw)
        self._x   = self._sx + ox*cy - oy*sy
        self._y   = self._sy + ox*sy + oy*cy
        self._yaw = self._syaw + ow
        self._have_odom = True

    # ── Task ───────────────────────────────────────────────────────────
    def _on_task(self, msg):
        if self._state != "IDLE":
            self.get_logger().warn(f"{self._id}: busy, ignoring task"); return
        self._order = msg
        self.get_logger().info(f"{self._id}: task {msg.order_id} mat={msg.material_type}")
        self._go_depot()

    # ── State transitions ──────────────────────────────────────────────
    def _go_depot(self):
        self._state = "GOING_TO_DEPOT"
        self._plan(DEPOT_NODE[self._order.material_type])

    def _start_load(self):
        self._state="LOADING"; self._waiting=True
        self._wait_t=0.0; self._wait_d=LOAD_TIME; self._stop()

    def _go_shop(self):
        self._state="GOING_TO_SHOP"
        self._plan(SHOP_NODE[self._order.shop_id])

    def _start_unload(self):
        self._state="UNLOADING"; self._waiting=True
        self._wait_t=0.0; self._wait_d=UNLOAD_TIME; self._stop()
        self._order.status="in_progress"; self._prog.publish(self._order)

    def _return_home(self):
        self._state="RETURNING"
        self._order.status="completed"; self._done.publish(self._order)
        self._order=None
        self._plan(HOME_NODE[self._id])

    def _set_idle(self):
        self._state="IDLE"; self._stop()

    # ── Planning ───────────────────────────────────────────────────────
    def _plan(self, goal):
        start = _nearest_node(self._x, self._y)
        self._path = bfs_path(start, goal)
        if len(self._path) > 1:
            nx,ny = NODES[self._path[0]]
            if math.hypot(self._x-nx, self._y-ny) < GOAL_TOL:
                self._path.pop(0)
        self.get_logger().info(f"{self._id}: {' → '.join(self._path)}")

    # ── Control loop ───────────────────────────────────────────────────
    def _ctrl_loop(self):
        ir_err, on_cnt = _ir(self._x, self._y, self._yaw)
        m = String(); m.data = f"ON:{on_cnt}" if on_cnt else "OFF"
        self._irp.publish(m)

        if self._waiting:
            self._wait_t += CTRL_DT
            if self._wait_t >= self._wait_d:
                self._waiting = False
                if self._state == "LOADING":   self._go_shop()
                elif self._state == "UNLOADING": self._return_home()
            return

        if self._state not in ("GOING_TO_DEPOT","GOING_TO_SHOP","RETURNING"):
            return

        if not self._path:
            self._stop()
            if self._state=="GOING_TO_DEPOT": self._start_load()
            elif self._state=="GOING_TO_SHOP": self._start_unload()
            elif self._state=="RETURNING": self._set_idle()
            return

        gx,gy = NODES[self._path[0]]
        dist  = math.hypot(gx-self._x, gy-self._y)

        if dist < GOAL_TOL:
            done = self._path.pop(0)
            self.get_logger().info(f"{self._id}: ✓ {done}")
            if not self._path:
                self._stop()
                if self._state=="GOING_TO_DEPOT": self._start_load()
                elif self._state=="GOING_TO_SHOP": self._start_unload()
                elif self._state=="RETURNING": self._set_idle()
                return
            gx,gy = NODES[self._path[0]]
            dist  = math.hypot(gx-self._x, gy-self._y)

        tgt_yaw  = math.atan2(gy-self._y, gx-self._x)
        head_err = math.atan2(math.sin(tgt_yaw-self._yaw), math.cos(tgt_yaw-self._yaw))

        if abs(head_err) > IR_THRESH:
            angular = KP_HEAD * head_err
        else:
            angular = KP_HEAD * head_err + KP_IR * ir_err
        angular = max(-MAX_ANG, min(MAX_ANG, angular))

        if abs(head_err) > TURN_THRESH:
            speed = CREEP_SPEED
        elif dist < SLOW_DIST:
            speed = SLOW_SPEED
        else:
            speed = max(CREEP_SPEED, SPEED_MAX*(1.0 - SPEED_DAMP*abs(angular)/MAX_ANG))

        t = Twist(); t.linear.x = speed; t.angular.z = angular
        self._cmd.publish(t)

    def _stop(self):
        self._cmd.publish(Twist())

    # ── Status ─────────────────────────────────────────────────────────
    def _pub_status(self):
        m = AGVStatus()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.agv_id = self._id
        m.state  = self._state
        m.current_order_id  = self._order.order_id if self._order else ""
        m.carrying_material = (
            self._order.material_type
            if self._order and self._state in ("GOING_TO_SHOP","UNLOADING") else "")
        m.battery_pct = 100.0
        m.pos_x = self._x
        m.pos_y = self._y
        self._stat.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
