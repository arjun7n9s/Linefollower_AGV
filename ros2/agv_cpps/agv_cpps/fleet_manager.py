"""
fleet_manager.py
────────────────────────────────────────────────────────────────────────
CPPS Coordination Layer: Fleet Assignment

Listens for pending ReplenishmentOrders and assigns them to the best
available (IDLE) AGV using a nearest-depot heuristic.

Topics subscribed:
  /orders/new          (agv_interfaces/ReplenishmentOrder)
  /agv_1/status        (agv_interfaces/AGVStatus)
  /agv_2/status        (agv_interfaces/AGVStatus)
  /agv_3/status        (agv_interfaces/AGVStatus)

Topics published:
  /agv_1/task          (agv_interfaces/ReplenishmentOrder)
  /agv_2/task          (agv_interfaces/ReplenishmentOrder)
  /agv_3/task          (agv_interfaces/ReplenishmentOrder)
  /orders/assigned     (agv_interfaces/ReplenishmentOrder)
"""

import math
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agv_interfaces.msg import ReplenishmentOrder, AGVStatus

# Depot positions in world frame
DEPOT_POSITIONS = {
    "A": (-13.0,  8.0),
    "B": ( 13.0,  8.0),
    "C": (-13.0, -3.0),
}


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")

        self._agv_ids = ["agv_1", "agv_2", "agv_3"]
        self._agv_status: dict[str, AGVStatus] = {}
        self._order_queue: deque[ReplenishmentOrder] = deque()

        # Subscriptions
        self._order_sub = self.create_subscription(
            ReplenishmentOrder, "/orders/new", self._on_new_order, 10
        )
        for agv_id in self._agv_ids:
            self.create_subscription(
                AGVStatus, f"/{agv_id}/status",
                lambda msg, a=agv_id: self._on_agv_status(a, msg), 10
            )

        # Publishers
        self._task_pubs = {
            agv_id: self.create_publisher(ReplenishmentOrder, f"/{agv_id}/task", 10)
            for agv_id in self._agv_ids
        }
        self._assigned_pub = self.create_publisher(ReplenishmentOrder, "/orders/assigned", 10)

        # Try to dispatch queued orders every second
        self.create_timer(1.0, self._dispatch_queued)

        self.get_logger().info("FleetManager ready — managing: " + ", ".join(self._agv_ids))

    # ── New order arrives ─────────────────────────────────────────────
    def _on_new_order(self, msg: ReplenishmentOrder):
        self.get_logger().info(f"[QUEUE] order={msg.order_id} material={msg.material_type} → {msg.shop_id}")
        self._order_queue.append(msg)
        self._dispatch_queued()

    # ── AGV status update ─────────────────────────────────────────────
    def _on_agv_status(self, agv_id: str, msg: AGVStatus):
        prev_state = self._agv_status.get(agv_id, None)
        self._agv_status[agv_id] = msg
        # If an AGV just became IDLE, try to assign a queued order
        if prev_state is not None and prev_state.state != "IDLE" and msg.state == "IDLE":
            self._dispatch_queued()

    # ── Dispatch logic ────────────────────────────────────────────────
    def _dispatch_queued(self):
        while self._order_queue:
            order = self._order_queue[0]
            best_agv = self._find_best_agv(order.material_type)
            if best_agv is None:
                break  # No idle AGV right now, wait
            self._order_queue.popleft()
            self._assign(order, best_agv)

    def _find_best_agv(self, material_type: str) -> str | None:
        # Primary: preferred AGV per material keeps routes clean
        preferred = {"A": "agv_1", "B": "agv_3", "C": "agv_2"}
        pref = preferred.get(material_type)
        if pref:
            st = self._agv_status.get(pref)
            if st is not None and st.state == "IDLE":
                return pref

        # Fallback: nearest idle AGV to depot
        depot_pos = DEPOT_POSITIONS[material_type]
        best_agv = None
        best_dist = float("inf")
        for agv_id in self._agv_ids:
            st = self._agv_status.get(agv_id)
            if st is None or st.state != "IDLE":
                continue
            dist = math.hypot(st.pos_x - depot_pos[0], st.pos_y - depot_pos[1])
            if dist < best_dist:
                best_dist = dist
                best_agv = agv_id
        return best_agv

    def _assign(self, order: ReplenishmentOrder, agv_id: str):
        order.status = "assigned"
        order.assigned_agv = agv_id
        order.header.stamp = self.get_clock().now().to_msg()
        self._task_pubs[agv_id].publish(order)
        self._assigned_pub.publish(order)
        self.get_logger().info(
            f"[ASSIGN] order={order.order_id} → {agv_id}  "
            f"(material={order.material_type} shop={order.shop_id})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
