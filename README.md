# AGV Factory — Cyber-Physical Production System (CPPS)

A **ROS2 + Ignition Gazebo 6** simulation of an autonomous material replenishment system inside a 30×20m factory floor. Three autonomous ground vehicles (AGVs) act as the physical layer of a Cyber-Physical Production System, continuously monitoring shop inventory and executing replenishment missions without human intervention.

---

## What It Simulates

A factory with three zones:

| Zone | Role | Location |
|---|---|---|
| **Depot A** | Material A source (red boxes) | NW corner |
| **Depot B** | Material B source (green boxes) | NE corner |
| **Depot C** | Material C source (blue boxes) | SW corner |
| **Shop 1/2/3** | Consumer stations needing stock | East side |
| **Docks 1/2/3** | AGV home/charging stations | South wall |

The shops continuously consume material. When stock drops below threshold, the system automatically dispatches an AGV to collect from the correct depot and deliver to the shop.

---

## System Architecture

```
┌─────────────────────────────────────────────┐
│              ROS2 Node Layer                │
│                                             │
│  order_manager  ──►  fleet_manager          │
│       │                   │                 │
│  (monitors shops)   (assigns AGVs)          │
│                           │                 │
│              agv_controller × 3             │
│         (line-following state machine)      │
│                           │                 │
│              event_logger                   │
└──────────────────┬──────────────────────────┘
                   │  /model/agv_N/cmd_vel
                   ▼
┌─────────────────────────────────────────────┐
│         Ignition Gazebo 6 (Physics)         │
│  3× AGV robot  +  factory world + lines     │
└─────────────────────────────────────────────┘
```

---

## Key Technical Features

- **Line-following navigation** — robots travel a fixed painted-line grid using a graph-based BFS path planner + cross-track error P-controller
- **IR sensor array** — 5-LED sensor bar on each robot (visually modelled); publishes `ON_LINE`/`OFF_LINE` to `/agv_N/ir_sensor`
- **Skid-steer diff-drive** — 4-wheel robots driven by Ignition's `diff-drive-system` plugin, bridged to ROS2 via `ros_gz_bridge`
- **Digital twin inventory** — `order_manager` maintains a virtual stock model; physical AGV actions update it
- **Fleet coordination** — `fleet_manager` assigns preferred AGVs per material type with idle fallback
- **Human workers** — 3 animated pedestrian actors walking the factory floor
- **Charging/docking pads** — AGVs return to home dock between missions

---

## Tech Stack

| Layer | Technology |
|---|---|
| Robot middleware | ROS2 Humble |
| Physics simulation | Ignition Gazebo 6 (Fortress) |
| ROS↔Gazebo bridge | `ros_gz_bridge` |
| Language | Python 3 (all nodes) |
| Robot model | SDF (4-wheel skid-steer) |
| World format | SDF world file |

---

## Repository Structure

```
agv_factory/
├── ros2/
│   ├── agv_cpps/           # ROS2 Python nodes (controller, fleet, orders, logger)
│   ├── agv_factory_world/  # Gazebo world + all SDF models
│   └── agv_interfaces/     # Custom ROS2 msg/srv definitions
├── DEBUGGING_PROGRESS.md
├── .gitignore
└── README.md
```

---

## Run It

```bash
# Build
cd agv_factory
colcon build
source install/setup.bash

# Launch simulation
ros2 launch agv_cpps cpps_sim.launch.py
```

### Prerequisites

- ROS2 Humble
- Ignition Gazebo 6 (Fortress)
- `ros-humble-ros-gz-sim`, `ros-humble-ros-gz-bridge`

---

## Line Network

The factory floor has a painted yellow guide line grid connecting all depots, shops, and docks. AGVs navigate exclusively along this grid using BFS shortest-path routing. No lines intersect walls.

```
DEPOT_A(-12,8)              DEPOT_B(+12,8)
      |                           |
   J_WN ──────────────────── J_EN       y=+5
      |    J_NW──J_NC──J_NE──J_SHOP_N──SHOP_1
      |                           |
   J_WM ─────────────── J_SHOP_M──SHOP_2   y=0
      |                           |
  DEPOT_C(-12,-3)          J_SHOP_S──SHOP_3
      |                           |
   J_WS ────────────────── J_ES       y=-5
      |                           |
   J_SW      J_SC      J_SE
      |         |         |
  DOCK_1    DOCK_2    DOCK_3              y=-8.8
```
