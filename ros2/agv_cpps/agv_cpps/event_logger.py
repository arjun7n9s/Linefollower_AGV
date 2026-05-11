"""
event_logger.py
────────────────────────────────────────────────────────────────────────
CPPS Observability Layer: Live Console Dashboard

Subscribes to all system topics and prints a live, colour-coded
event table. Shows the CPPS loop clearly:

  [INVENTORY] shop_1 stock=2/10 ← LOW  →  [ORDER] mat=A  →  [ASSIGN] agv_2
  →  [MOVING] agv_2 GOING_TO_DEPOT  →  [LOADING]  →  [GOING_TO_SHOP]
  →  [UNLOADING]  →  [DELIVERED] shop_1 stock=7/10

Also aggregates and prints a summary table every 5 s.
"""

import rclpy
from rclpy.node import Node
from agv_interfaces.msg import ShopInventory, ReplenishmentOrder, AGVStatus

# ANSI colours
R  = "\033[91m"
G  = "\033[92m"
Y  = "\033[93m"
B  = "\033[94m"
M  = "\033[95m"
C  = "\033[96m"
W  = "\033[97m"
BOLD = "\033[1m"
RST  = "\033[0m"

STATE_COLOUR = {
    "IDLE":            G,
    "GOING_TO_DEPOT":  Y,
    "LOADING":         M,
    "GOING_TO_SHOP":   C,
    "UNLOADING":       R,
    "RETURNING":       B,
}

STATUS_COLOUR = {
    "ok":           G,
    "low":          Y,
    "critical":     R,
    "replenishing": C,
}


class EventLogger(Node):
    def __init__(self):
        super().__init__("event_logger")

        self._agv_states: dict[str, str] = {}
        self._shop_stocks: dict[str, dict] = {}
        self._order_log: list[str] = []

        # Subscriptions
        self.create_subscription(ShopInventory,     "/shop/inventory",     self._on_inventory, 30)
        self.create_subscription(ReplenishmentOrder, "/orders/new",         self._on_new_order,  10)
        self.create_subscription(ReplenishmentOrder, "/orders/assigned",    self._on_assigned,   10)
        self.create_subscription(ReplenishmentOrder, "/orders/completed",   self._on_completed,  10)
        for agv in ["agv_1", "agv_2", "agv_3"]:
            self.create_subscription(
                AGVStatus, f"/{agv}/status",
                lambda msg, a=agv: self._on_agv_status(a, msg), 10
            )

        self.create_timer(5.0, self._print_summary)
        self.get_logger().info(f"{BOLD}[EventLogger] CPPS Dashboard started{RST}")

    def _on_inventory(self, msg: ShopInventory):
        prev = self._shop_stocks.get(msg.shop_id, {})
        self._shop_stocks[msg.shop_id] = {
            "stock": msg.stock, "capacity": msg.capacity,
            "status": msg.status, "material": msg.material_type
        }
        if prev.get("status") != msg.status:
            col = STATUS_COLOUR.get(msg.status, W)
            self.get_logger().info(
                f"{col}[INVENTORY] {msg.shop_id} (Mat-{msg.material_type}) "
                f"stock={msg.stock}/{msg.capacity}  status={msg.status.upper()}{RST}"
            )

    def _on_new_order(self, msg: ReplenishmentOrder):
        self.get_logger().info(
            f"{Y}[ORDER ▶] id={msg.order_id}  mat={msg.material_type}  "
            f"shop={msg.shop_id}  qty={msg.quantity}{RST}"
        )

    def _on_assigned(self, msg: ReplenishmentOrder):
        self.get_logger().info(
            f"{C}[ASSIGN ▶] order={msg.order_id} → {msg.assigned_agv}  "
            f"(mat={msg.material_type} shop={msg.shop_id}){RST}"
        )

    def _on_completed(self, msg: ReplenishmentOrder):
        self.get_logger().info(
            f"{G}{BOLD}[DELIVERED ✔] order={msg.order_id}  "
            f"shop={msg.shop_id}  by={msg.assigned_agv}{RST}"
        )

    def _on_agv_status(self, agv_id: str, msg: AGVStatus):
        prev = self._agv_states.get(agv_id)
        self._agv_states[agv_id] = msg.state
        if prev != msg.state:
            col = STATE_COLOUR.get(msg.state, W)
            task = f" order={msg.current_order_id}" if msg.current_order_id else ""
            mat  = f" carrying={msg.carrying_material}" if msg.carrying_material else ""
            self.get_logger().info(
                f"{col}[AGV] {agv_id}  {msg.state}{task}{mat}{RST}"
            )

    def _print_summary(self):
        lines = [f"\n{BOLD}{'─'*60}"]
        lines.append(f"  CPPS LIVE SUMMARY")
        lines.append(f"{'─'*60}{RST}")
        lines.append(f"  {'SHOP':<10} {'MAT':<5} {'STOCK':<12} {'STATUS'}")
        for shop_id in sorted(self._shop_stocks):
            s = self._shop_stocks[shop_id]
            col = STATUS_COLOUR.get(s["status"], W)
            bar_full = int(s["stock"] / max(s["capacity"], 1) * 10)
            bar = "█" * bar_full + "░" * (10 - bar_full)
            lines.append(
                f"  {shop_id:<10} {s['material']:<5} {bar} {s['stock']:>2}/{s['capacity']:<3} "
                f"{col}{s['status'].upper()}{RST}"
            )
        lines.append(f"  {'AGV':<10} {'STATE'}")
        for agv_id in sorted(self._agv_states):
            st = self._agv_states[agv_id]
            col = STATE_COLOUR.get(st, W)
            lines.append(f"  {agv_id:<10} {col}{st}{RST}")
        lines.append(f"{BOLD}{'─'*60}{RST}\n")
        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = EventLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
