# AGV Factory CPPS Simulation — Debugging Progress Log

> **Project:** VesperGrid — Cyber-Physical Production System (CPPS) simulation  
> **Stack:** ROS 2 Humble · Ignition Gazebo 6 (Fortress) · Python 3.10  
> **Goal:** 3 AGV robots autonomously replenish 3 factory shops by picking up materials from depots and delivering them.

---

## Table of Contents
1. [System Architecture](#1-system-architecture)
2. [What Was Built / Changed](#2-what-was-built--changed)
3. [Bug History — Root Causes & Fixes](#3-bug-history--root-causes--fixes)
4. [Current Status](#4-current-status)
5. [Active Open Issue — Robots Not Moving in Gazebo](#5-active-open-issue--robots-not-moving-in-gazebo)
6. [Diagnostic Evidence So Far](#6-diagnostic-evidence-so-far)
7. [Next Steps to Try](#7-next-steps-to-try)
8. [Key File Locations](#8-key-file-locations)

---

## 1. System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  ROS 2 Nodes (agv_cpps package)                             │
│                                                             │
│  order_manager  ──→  /orders/new  ──→  fleet_manager        │
│                                           │                 │
│                                    /agv_N/task              │
│                                           ↓                 │
│                                    agv_controller (×3)      │
│                                           │                 │
│                             /model/agv_N/cmd_vel (Twist)    │
│                                           │                 │
│  ─────────────────── ros_gz_bridge ───────┼──────────────── │
│                                           │                 │
│  Ignition Gazebo 6                        ↓                 │
│  agv_1, agv_2, agv_3 models   ←── diff-drive plugin        │
│  (agv_factory.world)               /model/agv_N/cmd_vel     │
└─────────────────────────────────────────────────────────────┘
```

### ROS 2 Nodes
| Node | Role |
|------|------|
| `order_manager` | Simulates shop inventory consumption, generates `ReplenishmentOrder` when stock < threshold |
| `fleet_manager` | Assigns orders to idle AGVs using preferred-AGV map + nearest-depot fallback |
| `agv_controller` (×3) | State machine per robot: IDLE → GOING_TO_DEPOT → LOADING → GOING_TO_SHOP → UNLOADING → RETURNING → IDLE |
| `event_logger` | Live console dashboard showing shop stock and AGV states |

### World Layout (30×20 m factory)
- **Depots:** Material-A at (-13, 8) NW corner, Material-B at (13, 8) NE corner, Material-C at (-13, -3) SW area
- **Shops:** shop_1 at (8, 5), shop_2 at (8, 0), shop_3 at (8, -5)
- **AGV home docks:** agv_1 at (-6, -8.8), agv_2 at (0, -8.8), agv_3 at (6, -8.8)

---

## 2. What Was Built / Changed

### 2.1 World File (`agv_factory.world`)
- Replaced sideways spotlights with a 3×4 grid of **downward spotlights** for factory floor illumination
- Redesigned **line track models** (yellow emissive boxes) for continuous connected navigation paths
- Added **overhead docking station lights** with emissive fixtures above each dock
- Added `load_station`, `drop_station`, `docking_station`, `charging_pad`, `carton_box` model includes

### 2.2 `agv_controller.py` — Full Rewrite
**Original approach:** Used `nav_msgs/Odometry` subscription to track robot pose and check `dist < GOAL_TOLERANCE` for waypoint completion.

**Problem:** The `ros_gz_bridge` topic remapping syntax was broken — odometry messages from Ignition never reached the ROS 2 controller. The controller's pose never updated, `dist` never dropped below tolerance, so all 3 AGVs were permanently stuck in `GOING_TO_DEPOT`.

**New approach — time-based dead-reckoning:**
- Each waypoint leg duration = `distance / LINEAR_SPEED` seconds
- Controller counts elapsed time on a 10 Hz timer
- Publishes `cmd_vel` for that duration, then snaps to waypoint and starts the next leg
- **Zero dependency on odometry or any sensor feedback**
- Added immediate `_publish_status()` on node startup so fleet_manager sees all AGVs as IDLE from second 0

### 2.3 `order_manager.py` — Stock Staggering
**Problem:** All 3 shops started at stock=10, threshold=3, same consume rate → all 3 orders fired at the exact same tick → fleet_manager dispatched 2, queued 1 → `agv_2` always sat idle.

**Fix:**
- Staggered initial stock: `shop_1=2, shop_2=4, shop_3=6` → orders fire at different ticks
- Increased `REPLENISH_QTY` from 5 → 10 (full restock per delivery) to keep the loop sustainable

### 2.4 `fleet_manager.py` — Preferred AGV Assignment
**Problem:** The nearest-depot heuristic was nearly random since all AGVs start at the same Y position. Two AGVs consistently got assigned, one was never chosen.

**Fix:** Added explicit preferred-AGV map as primary assignment strategy:
```python
preferred = {"A": "agv_1", "B": "agv_3", "C": "agv_2"}
```
Falls back to nearest-idle if preferred AGV is busy.

### 2.5 `cpps_sim.launch.py` — Bridge Fix
**Problem:** The original bridge used `remappings=` to translate `/model/agv_N/cmd_vel` → `/agv_N/cmd_vel`. This only renames the ROS 2 topic — it does NOT create a new bridge path. So when the controller published to `/agv_N/cmd_vel`, there was no bridge for that topic at all; messages went into a void.

**Fix:** Bridge arguments now use `/model/agv_N/cmd_vel` as the ROS 2 topic name directly — matching exactly what the Ignition diff-drive plugin's default scoped topic is. No remapping needed. Controller updated to publish to `/model/agv_N/cmd_vel`.

```python
# Before (broken):
f"/model/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"
# with broken remapping: (/model/agv_1/cmd_vel → /agv_1/cmd_vel)
# Controller published to /agv_1/cmd_vel → NO bridge for that topic

# After (working):
"/model/agv_1/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"
# Controller publishes to /model/agv_1/cmd_vel → bridge passes it to Ignition
```

**Confirmed working** via:
```bash
ros2 topic echo /model/agv_1/cmd_vel --once
# → linear.x: 0.5 ✓

ign topic -e -t /model/agv_1/cmd_vel -n 2
# → linear { x: 0.5 } ✓  (messages confirmed reaching Ignition)
```

### 2.6 `agv_robot/model.sdf` — Wheel Axis Fix + 4-Wheel Redesign

#### First fix attempt — axis correction on 2-wheel model
**Problem:** Both wheel joints had `<axis><xyz>0 0 1</xyz>` — spinning around the **vertical Z-axis** (like a spinning top) instead of the **lateral Y-axis** (rolling forward).

With `sdf version="1.7"` the joint axis is in the **parent frame**, so despite the wheel link being posed at 90° roll, the revolute joint was rotating the wheel around the wrong axis producing zero ground traction.

**Fix:** Changed to `<axis><xyz>0 1 0</xyz>`.

#### Second issue — Gazebo crash
The old model had `ignition-gazebo-sensors-system` placed inside `<sensor>` elements. This plugin belongs at the **world level**, not per-sensor. This caused Ignition Gazebo to crash/exit cleanly shortly after launch.

#### Current model — full 4-wheel redesign
Complete rewrite of `model.sdf`:
- **4 wheels** at `(±0.18, ±0.22)` — proper square layout
- **Chassis raised** to `z=0.12` so all 4 wheels sit flat on ground (no sinking/floating)
- **Joint axis** uses `<xyz expressed_in="__model__">0 1 0</xyz>` — `expressed_in="__model__"` eliminates all frame ambiguity in Ignition Gazebo 6
- **Diff-drive** drives `joint_wheel_fl` + `joint_wheel_fr`; rear wheels spin freely
- **Removed all sensor plugins** that caused the crash
- **Visual wheel markers:** Orange half-disc + white perpendicular stripe on each wheel face — rotation is clearly visible when wheels spin
- **White forward arrow** on yellow top plate — shows robot orientation

---

## 3. Bug History — Root Causes & Fixes

| # | Symptom | Root Cause | Fix Applied |
|---|---------|------------|-------------|
| 1 | All AGVs stuck in `GOING_TO_DEPOT` forever | `ros_gz_bridge` remapping broken → odometry never reached controller → pose never updated → waypoint distance never reached 0 | Replaced odometry-based nav with time-based dead-reckoning |
| 2 | `agv_2` always IDLE, never assigned | All shops hit threshold at same tick → 2 orders dispatched, 1 queued → always the same 2 AGVs get work | Staggered initial stock; added preferred-AGV map in fleet_manager |
| 3 | `cmd_vel` published but robots not moving | Bridge `remappings=` only renames ROS 2 side, doesn't create new bridge path → controller's `/agv_N/cmd_vel` had no bridge → Ignition received nothing | Controller publishes to `/model/agv_N/cmd_vel`; bridge uses same name on both sides |
| 4 | Gazebo crash (process exited cleanly) | `ignition-gazebo-sensors-system` placed inside `<sensor>` tags — wrong placement causes segfault/exit | Removed all sensor plugins from robot model |
| 5 | Wheels receive cmd_vel, physics running, but robot doesn't translate | **Joint axis frame confusion.** SDF 1.7 `<xyz>` is in the **child link frame** by default. Wheel links are rolled 90° around X (`pose roll=1.5708`), so child local-Z = world Y. Correct axis in child frame = `0 0 1`. Previous attempts used `0 1 0` (= world Z = vertical spinning top) and `0 1 0 expressed_in=__model__` (= model/world Y, but `expressed_in` not reliably supported). | `<xyz>0 0 1</xyz>` in child frame — world-Y rotation, correct rolling axis. Math verified: `Rx(90)⁻¹ × [0,1,0]_world = [0,0,-1]_child` → `0 0 1` drives forward |

---

## 4. Current Status

### ✅ Confirmed Working
- Gazebo loads and renders the world (floor, walls, line tracks, stations, all models visible)
- `ros_gz_bridge` starts and bridges all 6 topics (`cmd_vel` ×3, `odometry` ×3)
- `/model/agv_1/cmd_vel`, `/model/agv_2/cmd_vel`, `/model/agv_3/cmd_vel` — all publishing `linear.x: 0.5`
- Ignition receives those messages: `ign topic -e -t /model/agv_1/cmd_vel` returns `linear { x: 0.5 }`
- ROS 2 logic cycle works: all 3 AGVs cycle through IDLE → GOING_TO_DEPOT → LOADING → GOING_TO_SHOP → (presumably UNLOADING → RETURNING → IDLE)
- All 3 AGVs get assigned tasks (no more permanent idle AGV)
- Shop inventory depletes and replenishes correctly in the digital twin

### ❌ Still Not Working
- **AGVs do not physically move in the Gazebo viewport** — robots remain frozen at their spawn positions despite receiving correct `cmd_vel` messages that Ignition can see

---

## 5. Active Open Issue — Robots Not Moving in Gazebo

### What We Know
The full message pipeline is verified end-to-end:

```
agv_controller (ROS 2)
  └─ publishes Twist(linear.x=0.5) to /model/agv_1/cmd_vel (ROS 2)
        └─ ros_gz_bridge passes it to Ignition topic /model/agv_1/cmd_vel
              └─ diff-drive plugin subscribed to "cmd_vel" (= /model/agv_1/cmd_vel scoped)
                    └─ should drive joint_wheel_fl and joint_wheel_fr
                          └─ wheels should rotate → robot should translate
```

Steps 1–3 are **confirmed working**. Step 4 onwards is where the chain breaks.

### Remaining Suspects

1. **`<topic>cmd_vel</topic>` in the diff-drive plugin resolves differently than expected**
   - Inside a model named `agv_1`, the relative topic `cmd_vel` should scope to `/model/agv_1/cmd_vel` in Ignition Gazebo 6
   - But this scoping behaviour may differ between Ignition versions or when models are included via `<include>` rather than defined inline
   - **To test:** Change `<topic>cmd_vel</topic>` to `<topic>/model/agv_1/cmd_vel</topic>` in the SDF — but this hardcodes the name for only one robot instance, which breaks the shared model

2. **Physics engine not stepping / simulation paused**
   - If Gazebo launched in paused state (without `-r` flag), physics doesn't advance
   - Launch uses `gz_args: "-r {world_file}"` so it should run, but worth verifying

3. **Wheel geometry / ground contact issue**
   - If wheels don't make proper contact with the ground plane, no traction force
   - `base_link` pose is `z=0.12`, wheels at `z=0.07` — net wheel bottom = `0.12 - 0.07 = 0.05` above ground → **wheels may be floating 5 cm above the floor**

4. **Joint axis frame interpretation with `expressed_in="__model__"`**
   - Ignition Gazebo 6 may not support `expressed_in="__model__"` in all builds
   - Fallback: use `<use_parent_model_frame>true</use_parent_model_frame>` inside `<axis>` instead

5. **Diff-drive plugin not receiving messages despite bridge working**
   - The bridge creates an Ignition topic `/model/agv_1/cmd_vel`
   - The diff-drive plugin creates its own subscriber for `cmd_vel` scoped as `/model/agv_1/cmd_vel`
   - These should be the same topic but if there's a namespace conflict they could be different publishers/subscribers that never connect

---

## 6. Diagnostic Evidence So Far

```bash
# ✅ ROS 2 side — topics exist and publishing
ros2 topic list | grep cmd_vel
# /model/agv_1/cmd_vel
# /model/agv_2/cmd_vel
# /model/agv_3/cmd_vel

ros2 topic echo /model/agv_1/cmd_vel --once
# linear: x: 0.5  ← controller is publishing correctly

# ✅ Ignition side — topics exist and messages flowing
ign topic -l | grep cmd_vel
# /model/agv_1/cmd_vel ✓

ign topic -e -t /model/agv_1/cmd_vel -n 2
# linear { x: 0.5 }  ← Ignition is receiving the messages

# ❓ Not yet checked — odometry (would confirm if physics is running)
ign topic -e -t /model/agv_1/odometry -n 2
# If this shows changing values → physics running, diff-drive working
# If this shows zeros / no output → diff-drive plugin not functioning
```

---

## 7. Next Steps to Try

### Priority 1 — Fix wheel ground contact (most likely cause)
The `base_link` is at `z=0.12`, wheel links at `z=0.07` relative to model origin. Net wheel-bottom = `0.12 - 0.07 = 0.05 m` **above the ground**. Robots are probably floating/sinking with no real contact.

**Fix:** Lower wheel z-pose to `0.05` (= wheel radius) so bottom of wheel is exactly at z=0, OR raise `base_link` to `z=0.14` to match `wheel_z + radius = 0.07 + 0.07 = 0.14`.

### Priority 2 — Check odometry output
```bash
ign topic -e -t /model/agv_1/odometry -n 5
```
- If pose changes → diff-drive IS working, visual glitch only
- If pose stays zero → diff-drive plugin not activated / physics paused

### Priority 3 — Check simulation is unpaused
```bash
ign topic -e -t /world/agv_factory/stats -n 3
# Look for: sim_time advancing, real_time_factor > 0
```

### Priority 4 — Hardcode topic name as absolute path in SDF
Change `<topic>cmd_vel</topic>` to `<topic>/model/agv_1/cmd_vel</topic>` for one robot to test if scoping is the issue. If it moves, the problem is the relative topic name resolution.

### Priority 5 — Try `use_parent_model_frame`
Replace `expressed_in="__model__"` with older SDF syntax:
```xml
<axis>
  <xyz>0 1 0</xyz>
  <use_parent_model_frame>true</use_parent_model_frame>
  <limit>...</limit>
</axis>
```

---

## 8. Key File Locations

| File | Path |
|------|------|
| Robot SDF model | `ros2/agv_factory_world/models/agv_robot/model.sdf` |
| World file | `ros2/agv_factory_world/worlds/agv_factory.world` |
| AGV controller | `ros2/agv_cpps/agv_cpps/agv_controller.py` |
| Fleet manager | `ros2/agv_cpps/agv_cpps/fleet_manager.py` |
| Order manager | `ros2/agv_cpps/agv_cpps/order_manager.py` |
| Event logger | `ros2/agv_cpps/agv_cpps/event_logger.py` |
| Launch file | `ros2/agv_cpps/launch/cpps_sim.launch.py` |

### Build & Run Commands
```bash
cd /home/arjun7n9s/Downloads/AMD-S-2/agv_factory

# Build everything
colcon build --packages-select agv_factory_world agv_cpps

# Build robot model only (SDF changes)
colcon build --packages-select agv_factory_world

# Launch simulation
source install/setup.bash
ros2 launch agv_cpps cpps_sim.launch.py

# Diagnostic — check Ignition topics (in second terminal)
ign topic -l
ign topic -e -t /model/agv_1/cmd_vel -n 3
ign topic -e -t /model/agv_1/odometry -n 3
ign topic -e -t /world/agv_factory/stats -n 3
```
