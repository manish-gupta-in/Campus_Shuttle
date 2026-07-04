<div align="center">

# 🚌 BITS Pilani · Autonomous Campus Shuttle

**Production-grade Command & Control Platform for Autonomous Vehicle Navigation**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Autoware Universe](https://img.shields.io/badge/Autoware-Universe-orange?logo=autoware)](https://github.com/autowarefoundation/autoware.universe)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)](https://www.python.org/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

*BITS Pilani · Hyderabad Campus | WILP Autonomous Vehicle Research Lab*

</div>

---

## 🖥️ Dashboard Preview

![Autonomous Shuttle Mission Control Dashboard](Dashboard_Image.png)

> *Real-time Mission Control Dashboard — featuring 22,000-point PCD map, Lanelet2 OSM road overlay, live vehicle tracking, and telemetry bridge*

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [FSM State Machine](#-finite-state-machine)
- [Directory Structure](#-directory-structure)
- [Installation](#-installation--sourcing)
- [Quick-Start Guide](#-quick-start-operation)
- [Configuration](#%EF%B8%8F-configuration)
- [Campus Route](#-campus-route)
- [Bug Fixes v4](#-key-bug-fixes-in-v4)
- [Deployment](#-deployment-to-github)
- [Author](#-author)

---

## 🌐 Overview

The **BITS Pilani Autonomous Campus Shuttle** is an end-to-end production system for deploying an autonomous electric shuttle around the Hyderabad campus. It integrates:

- **Autoware Universe** — industry-grade AV planning and localization stack
- **ROS 2 Humble** — distributed middleware for real-time vehicle control
- **Custom Mission FSM Node** (`shuttle.py`) — 11-state Finite State Machine for deterministic, safe route execution
- **CustomTkinter Mission Control Dashboard** (`shuttle_dashboard.py`) — real-time command, visualization, and telemetry monitoring

The system supports both **Simulation Mode** (Autoware Planning Simulator) and **Vehicle Connect Mode** (direct hardware deployment on the Hooke2 DBW platform).

---

## ✨ Key Features

### 🤖 Autonomous Navigation Core (`shuttle.py`)
- **11-Stage FSM** — deterministic state transitions with safe recovery paths
- **Dual-Trigger Arrival Detection** — Primary: `RouteState=GOAL_REACHED`; Fallback: `/tf` proximity check (5m radius, 0.2 m/s threshold, 5s confirmation)
- **Flexible Start Index** — start the route from any waypoint (`START_WAYPOINT_INDEX`)
- **One-Shot Gear Park** — eliminates DBW actuator clicking via `_gear_park_sent` guard flag
- **Velocity Cap Keepalive** — 5 Hz `Float32(0.0)` to `/planning/scenario_planning/current_max_velocity` during stops
- **Pawl Settle Sequence** — DRIVE → 2.0s delay → release velocity cap (mechanical park pawl safe disengagement)
- **Exponential Retry Backoff** — autonomous mode service failures: 1s → 2s → 4s → 8s → 16s max

### 🖥️ Mission Control Dashboard (`shuttle_dashboard.py` v7.0)
- **Sub-millisecond Binary PCD Loader** — `struct.unpack` byte-seek on 198 MB `.pcd` file → 22,000 downsampled points rendered instantly
- **Lanelet2 OSM Road Overlay** — full campus road network parsed from `lanelet2_map.osm` via `xml.etree`
- **Direct ROS 2 Telemetry Bridge** — bypasses visualization packages, subscribes directly to:
  - `/localization/kinematic_state` → real-time X,Y pose + quaternion→yaw
  - `/vehicle/status/velocity_status` → speed in km/h
  - `/planning/route_state` → route state string
  - `/api/operation_mode/state` → AUTONOMOUS / MANUAL badge
- **Dynamic AST Waypoint Parser** — auto-syncs dashboard map to `shuttle.py` waypoint changes (no double-editing)
- **Interactive Map** — scroll-wheel zoom (0.4×–15×), left-drag pan, 3rd-person vehicle follower camera
- **Double-Buffered 60 FPS Rendering** — static layer (PCD + OSM) cached; only dynamic layer (vehicle + route) redrawn per tick
- **Hybrid Mode Bridge** — toggle between Simulation and Vehicle Connect mode on-the-fly
- **Futuristic Wireframe Vehicle Icon** — rotating LIDAR dot, transparent glass CAD chassis
- **PIL Buffered Pointcloud Engine** — 22,000 PCD points composited into one PIL.Image blit (eliminates Tcl stack overflow on Linux)

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                   OPERATOR CONTROL ROOM                          │
│          shuttle_dashboard.py (CustomTkinter v7.0)               │
│  ┌──────────────┐  ┌──────────────────────┐  ┌───────────────┐  │
│  │ Left Panel   │  │   Map Canvas (PCD    │  │ Right Panel   │  │
│  │ - Start Stop │  │   + OSM + Vehicle)   │  │ - Speed Gauge │  │
│  │ - SIM Launch │  │   22K points, 60 FPS │  │ - State Badge │  │
│  │ - Node Launch│  │   Lanelet2 overlay   │  │ - Stop Info   │  │
│  │ - Log Output │  │   Follow-cam mode    │  │ - Dwell Timer │  │
│  └──────────────┘  └──────────────────────┘  └───────────────┘  │
└────────────────────────────┬─────────────────────────────────────┘
                             │ ROS 2 DDS Topics + Process Exec
    ┌────────────────────────▼─────────────────────────────────────┐
    │              ROS 2 HUMBLE WORKSPACE  (av_ws/)                │
    │  ┌──────────────────────────────────────────────────────┐    │
    │  │     AutowareShuttleMission Node  (shuttle.py v4)     │    │
    │  │     11-State FSM  ·  Dual-Trigger Arrival Detection  │    │
    │  │     One-Shot Gear ·  Vel-Cap Keepalive  ·  TF2 Prox │    │
    │  └───────────────────────┬──────────────────────────────┘    │
    │  ┌───────────────────────▼──────────────────────────────┐    │
    │  │        Autoware Universe Planning Stack               │    │
    │  │   NDT Localizer · Mission Planner · MPC Controller   │    │
    │  └───────────────────────┬──────────────────────────────┘    │
    │  ┌───────────────────────▼──────────────────────────────┐    │
    │  │  Hooke2 DBW Interface (Vehicle) / Sim Physics (Sim)  │    │
    │  └──────────────────────────────────────────────────────┘    │
    └──────────────────────────────────────────────────────────────┘
                             │
    ┌────────────────────────▼─────────────────────────────────────┐
    │                  MAP SUBSYSTEM  (map/)                       │
    │  pointcloud_map.pcd (198 MB) · lanelet2_map.osm · configs    │
    └──────────────────────────────────────────────────────────────┘
```

> 📖 See [ARCHITECTURE.md](ARCHITECTURE.md) for full topic/service maps, FSM diagrams, and data flow details.

---

## 🔄 Finite State Machine

The `AutowareShuttleMission` node operates as an **11-state FSM**:

```
INIT → SET_INITIAL_POSE → WAIT_BEFORE_GOAL → SET_GOAL_POSE
     → WAIT_AUTONOMOUS_READY → SWITCH_AUTONOMOUS → ENGAGE_CONTROL
     → MONITORING_PROGRESS ──► JUNCTION_STOP → RESUME_AFTER_STOP
                          │         └─────────────────┘ (loop per stop)
                          └──► COMPLETE (final waypoint reached)
```

| State | Action |
|-------|--------|
| `INIT` | 2s system settle |
| `SET_INITIAL_POSE` | Publish `/initialpose` to map frame |
| `WAIT_BEFORE_GOAL` | 15s localisation warmup |
| `SET_GOAL_POSE` | Publish next waypoint to Autoware planner |
| `WAIT_AUTONOMOUS_READY` | Wait for `RouteState ∈ {SET, ARRIVED, FOLLOWING}` |
| `SWITCH_AUTONOMOUS` | Call `/api/operation_mode/change_to_autonomous` (with retry) |
| `ENGAGE_CONTROL` | Call `/api/operation_mode/enable_autoware_control` |
| `MONITORING_PROGRESS` | Monitor dual-trigger arrival; log speed/distance at 10s |
| `JUNCTION_STOP` | Vel-cap 0.0 m/s @ 5 Hz + PARK gear one-shot + 10s dwell |
| `RESUME_AFTER_STOP` | DRIVE gear → 2s pawl settle → release vel-cap |
| `COMPLETE` | Cancel all timers, safe ROS 2 shutdown |

---

## 📁 Directory Structure

```
Campus_Shuttle/
├── av_ws/                          # ROS 2 Humble Workspace
│   ├── src/
│   │   └── campus_pkg/             # Core Shuttle Controller Package
│   │       ├── campus_pkg/
│   │       │   ├── __init__.py
│   │       │   └── shuttle.py      # 🤖 Main ROS 2 Autonomous Navigation Node (v4)
│   │       ├── resource/
│   │       ├── test/
│   │       ├── package.xml         # ROS 2 Package Dependencies
│   │       ├── setup.cfg
│   │       └── setup.py            # Node Entrypoint Registrations
│   ├── edited_launch/              # Custom Autoware launch XML configs
│   ├── maps/                       # Symlink/mirror to global Autoware maps dir
│   └── college.sh                  # Hardware vehicle multi-terminal launcher
│
├── map/                            # Campus Map Assets
│   ├── lanelet2_map.osm            # Lanelet2 Vector Map (road center lines)
│   ├── pointcloud_map.pcd          # ⚠️ 198 MB 3D Pointcloud Map (Git LFS)
│   ├── map_config.yaml             # Map coordinate origin metadata
│   └── map_projector_info.yaml     # MGRS/UTM projection info
│
├── shuttle_sim_Waypoints/
│   └── waypoints.txt               # Raw UTM waypoint coordinates
│
├── .gitignore                      # Build artifacts + large .pcd excluded
├── .gitattributes                  # Git LFS tracking rules
├── Dashboard_Image.png             # 🖼️ Dashboard screenshot
├── car.jpeg                        # Vehicle asset for dashboard header
├── wilp_logo.png                   # BITS WILP academic affiliation logo
├── logo.webp                       # Mission branding logo
├── shuttle_dashboard.py            # 🖥️ Multi-mode Mission Control Dashboard (v7.0)
├── test_dashboard.py               # Unit tests for dashboard modules
├── run_dashboard.sh                # Quick-launch shell script
├── requirements.txt                # Python pip dependencies
├── ARCHITECTURE.md                 # Full technical architecture documentation
├── WALKTHROUGH.md                  # Operator quickstart guide
└── README.md                       # This file
```

---

## 🛠️ Installation & Sourcing

**Prerequisites:** Ubuntu 22.04 LTS · ROS 2 Humble · Autoware Universe (installed)

### 1. Clone the Repository

```bash
git clone https://github.com/manish-gupta-in/Campus_Shuttle.git
cd Campus_Shuttle
```

### 2. Install Python Dashboard Dependencies

```bash
pip3 install -r requirements.txt
```

### 3. Build the ROS 2 Workspace

```bash
cd av_ws

# (Optional) Clean previous build
rm -rf build install log

# Build with colcon
colcon build --symlink-install

# Source environments (run in every new terminal!)
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 🚀 Quick-Start Operation

Always run each component in a **separate terminal**, in this order:

### Terminal 1 — Autoware Simulator

```bash
source /opt/ros/humble/setup.bash
source /path/to/Campus_Shuttle/av_ws/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=/path/to/Campus_Shuttle/map
```

### Terminal 2 — Mission Control Dashboard

```bash
cd /path/to/Campus_Shuttle
python3 shuttle_dashboard.py
```

Or use the quick-launch script:

```bash
bash run_dashboard.sh
```

### Terminal 3 — Shuttle Mission Node

```bash
source /opt/ros/humble/setup.bash
source /path/to/Campus_Shuttle/av_ws/install/setup.bash
ros2 run campus_pkg shuttle
```

### Hardware Vehicle (Real Deploy — `college.sh`)

For full hardware deployment (Hooke2 DBW vehicle):

```bash
cd av_ws
bash college.sh
```

This script:
1. Copies the custom `autoware.launch.xml` to the Autoware install directory
2. Opens a terminal running Autoware with the full sensor stack (`sa` command)
3. Opens a terminal running the Autoware control interface (`autoware` command)
4. Opens a terminal running `ros2 run campus_pkg shuttle` after 15s warmup

---

## ⚙️ Configuration

All tunable parameters live at the top of their respective files:

### `shuttle.py` — Mission Tuning

```python
STOP_WAIT_SEC       = 10    # Dwell time at junction stops (seconds)
GOAL_PUBLISH_DELAY  = 15    # Localisation warm-up after initial pose (seconds)
PROXIMITY_RADIUS    = 5.0   # Fallback arrival detection radius (metres)
STOP_SPEED_THRESH   = 0.2   # Speed threshold for proximity trigger (m/s)
PROXIMITY_CONFIRM   = 5.0   # Seconds to confirm proximity arrival
VEL_CAP_HZ          = 5     # Velocity cap keepalive rate during stops (Hz)
RESUME_GEAR_SETTLE  = 2.0   # Gear settle delay before releasing vel-cap (seconds)
START_WAYPOINT_INDEX = 0    # Start route from this waypoint index (0–8)
```

### `shuttle_dashboard.py` — Paths & Layout

```python
BASE_DIR           = os.path.dirname(os.path.abspath(__file__))  # Auto-resolved
WS_PATH            = os.path.join(BASE_DIR, "av_ws")
MAPS_PATH          = os.path.join(WS_PATH, "maps")
LANELET2_OSM_MAP   = os.path.join(BASE_DIR, "map", "lanelet2_map.osm")
POINTCLOUD_PCD_MAP = os.path.join(BASE_DIR, "map", "pointcloud_map.pcd")
```

---

## 🗺️ Campus Route

The shuttle navigates a **10-waypoint loop** around BITS Pilani Hyderabad Campus:

```
Security Main Gate → A-Block → Hostel Circle → CP →
E-Block → WILP-Lab → K-Block → H-Block → I-Block → Security (End)
```

| # | Stop | Dwell |
|---|------|-------|
| 0 | Security Main Gate | Pass-through |
| 1 | A-Block | ✅ 10s stop |
| 2 | Hostel Circle | ✅ 10s stop |
| 3 | CP (Central Plaza) | ✅ 10s stop |
| 4 | E-Block | ✅ 10s stop |
| 5 | WILP-Lab | ✅ 10s stop |
| 6 | K-Block | ✅ 10s stop |
| 7 | H-Block | ✅ 10s stop |
| 8 | I-Block | ✅ 10s stop |
| 9 | Security (End) | Final destination |

---

## 🛡️ Key Bug Fixes in v4

| Issue | Root Cause | Fix Applied |
|-------|-----------|------------|
| **Clicking sound at stops** | `GearCommand(PARK)` published at 2 Hz continuously — Hooke2 DBW re-actuates each message | `_gear_park_sent` one-shot flag: gear sent **exactly once** per stop |
| **Park re-engaging after DRIVE** | `_brake_hold_active` shared between JUNCTION and RESUME; timer kept re-sending PARK | Split into independent `_vel_cap_active` and `_gear_park_sent` flags |
| **Multiple engage calls per stop** | Shared flag cleared mid-mission; `_engage_brake_hold()` re-fired | Per-leg `_gear_park_sent` reset only in `_advance_to_next_waypoint()` |
| **Speed always 0.00 m/s** | Wrong message type subscribed | Correct `VelocityReport.longitudinal_velocity` + `abs()` |
| **Autonomous retry storm** | No backoff on service failures | Exponential backoff: 1s → 2s → 4s → 8s → 16s max |
| **Dashboard PCD Tcl overflow** | 22,000 canvas rectangle objects → Tcl/Tk stack overflow on Linux | PIL.Image compositing → single `PhotoImage` blit per render |
| **DDS/X11 segfault at startup** | `rclpy` import before X11 display acquired | Delayed import inside `ROS2Bridge.start()` method |

---

## 📦 Deployment to GitHub

Use the included deployment script to push cleanly (handles LFS + ignore rules):

```bash
cd Campus_Shuttle
bash push_to_github.sh
```

The script handles:
- Initialising git if needed
- Setting up `.gitattributes` for Git LFS (`*.pcd` tracking)
- Staging all tracked files (excluding build artifacts and `pointcloud_map.pcd`)
- Committing with a timestamped message
- Pushing to `origin main`

---

## 🧪 Running Tests

```bash
cd Campus_Shuttle
python3 test_dashboard.py
```

All unit tests should complete with `OK` status, verifying:
- PCD parser binary read
- Lanelet2 OSM XML parser
- AST waypoint extractor from `shuttle.py`
- ROS 2 bridge initialization (mocked)
- Dashboard widget construction

---

## 👤 Author

**Manish Gupta** — Autonomous Systems & Robotics Engineer

> Specializing in secure command & control systems for autonomous vehicles, drones, and robotic platforms. This project demonstrates end-to-end AV stack integration, real-time localization bridging, and multi-modal fleet management.

- 🐙 **GitHub:** [@manish-gupta-in](https://github.com/manish-gupta-in)
- 💼 **LinkedIn:** [Manish Gupta](https://www.linkedin.com/in/manish-gupta-in/)
- 🏫 **Institution:** BITS Pilani, WILP — Autonomous Vehicle Research Lab
- 📧 Open for collaborations, consulting, and contributions!

---

## 📄 License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

---

<div align="center">

*Built with ❤️ at BITS Pilani · Hyderabad Campus*

**Autoware Universe · ROS 2 Humble · CustomTkinter · Hooke2 DBW**

</div>
