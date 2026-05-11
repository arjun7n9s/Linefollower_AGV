"""
order_manager.py
────────────────────────────────────────────────────────────────────────
CPPS Cyber Layer: Shop Inventory Digital Twin + Order Generation

Each shop tracks one material type. Stock drains on a configurable timer
(simulating consumption). When stock drops at or below the reorder threshold,
a ReplenishmentOrder is published. A new order for the same shop is not issued
while one is already in progress (status == "replenishing").

Topics published:
  /shop/inventory      (agv_interfaces/ShopInventory) — 1 Hz per shop
  /orders/new          (agv_interfaces/ReplenishmentOrder) — on demand

Topics subscribed:
  /orders/completed    (agv_interfaces/ReplenishmentOrder) — updates stock on delivery
"""

import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agv_interfaces.msg import ShopInventory, ReplenishmentOrder


SHOPS = {
    "shop_1": {"material": "A", "stock": 2, "capacity": 10, "threshold": 4, "consume_rate": 1, "status": "ok"},
    "shop_2": {"material": "B", "stock": 4, "capacity": 10, "threshold": 4, "consume_rate": 1, "status": "ok"},
    "shop_3": {"material": "C", "stock": 6, "capacity": 10, "threshold": 4, "consume_rate": 1, "status": "ok"},
}

REPLENISH_QTY = 10  # full restock per delivery


class OrderManager(Node):
    def __init__(self):
        super().__init__("order_manager")

        self.declare_parameter("consume_interval_sec", 8.0)
        self.declare_parameter("publish_interval_sec", 1.0)

        consume_interval = self.get_parameter("consume_interval_sec").value
        publish_interval = self.get_parameter("publish_interval_sec").value

        self._shops = {k: dict(v) for k, v in SHOPS.items()}
        self._active_orders: dict[str, str] = {}  # shop_id -> order_id

        self._inv_pub = self.create_publisher(ShopInventory, "/shop/inventory", 10)
        self._order_pub = self.create_publisher(ReplenishmentOrder, "/orders/new", 10)
        self._completed_sub = self.create_subscription(
            ReplenishmentOrder, "/orders/completed", self._on_completed, 10
        )

        self.create_timer(consume_interval, self._consume_tick)
        self.create_timer(publish_interval, self._publish_inventories)

        self.get_logger().info("OrderManager started — shops: shop_1(A) shop_2(B) shop_3(C)")

    # ── Consumption tick ──────────────────────────────────────────────
    def _consume_tick(self):
        for shop_id, s in self._shops.items():
            if s["stock"] > 0:
                s["stock"] = max(0, s["stock"] - s["consume_rate"])
            self._update_status(shop_id)
            self._maybe_issue_order(shop_id)

    def _update_status(self, shop_id: str):
        s = self._shops[shop_id]
        if shop_id in self._active_orders:
            s["status"] = "replenishing"
        elif s["stock"] == 0:
            s["status"] = "critical"
        elif s["stock"] <= s["threshold"]:
            s["status"] = "low"
        else:
            s["status"] = "ok"

    def _maybe_issue_order(self, shop_id: str):
        s = self._shops[shop_id]
        if s["stock"] <= s["threshold"] and shop_id not in self._active_orders:
            order_id = str(uuid.uuid4())[:8]
            self._active_orders[shop_id] = order_id
            msg = ReplenishmentOrder()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.order_id = order_id
            msg.material_type = s["material"]
            msg.shop_id = shop_id
            msg.quantity = REPLENISH_QTY
            msg.status = "pending"
            msg.assigned_agv = ""
            self._order_pub.publish(msg)
            self.get_logger().info(
                f"[ORDER] {order_id} → {shop_id} needs {REPLENISH_QTY}x Material-{s['material']} "
                f"(stock={s['stock']}/{s['capacity']})"
            )

    # ── Completion callback ───────────────────────────────────────────
    def _on_completed(self, msg: ReplenishmentOrder):
        shop_id = msg.shop_id
        if shop_id in self._active_orders and self._active_orders[shop_id] == msg.order_id:
            s = self._shops[shop_id]
            s["stock"] = min(s["capacity"], s["stock"] + msg.quantity)
            del self._active_orders[shop_id]
            self._update_status(shop_id)
            self.get_logger().info(
                f"[DELIVERED] order={msg.order_id} → {shop_id} stock now {s['stock']}/{s['capacity']}"
            )

    # ── Periodic inventory broadcast ─────────────────────────────────
    def _publish_inventories(self):
        now = self.get_clock().now().to_msg()
        for shop_id, s in self._shops.items():
            inv = ShopInventory()
            inv.header = Header()
            inv.header.stamp = now
            inv.shop_id = shop_id
            inv.material_type = s["material"]
            inv.stock = s["stock"]
            inv.capacity = s["capacity"]
            inv.threshold = s["threshold"]
            inv.status = s["status"]
            self._inv_pub.publish(inv)


def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
