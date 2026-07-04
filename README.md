<div align="center">

# 🚌 Autonomous Campus Shuttle

**Real-time Command & Control Platform for Autonomous Vehicle Navigation**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Autoware Universe](https://img.shields.io/badge/Autoware-Universe-orange)](https://github.com/autowarefoundation/autoware.universe)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)](https://www.python.org/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

</div>

---

## 🖥️ Dashboard

![Mission Control Dashboard](Dashboard_Image.png)

---

## Overview

An end-to-end platform for deploying an autonomous campus shuttle. The system combines an **11-state mission FSM node** built on ROS 2 with a **real-time mission control dashboard** that visualizes live vehicle telemetry, a 198 MB 3D pointcloud map, and Lanelet2 road overlays — all at 60 FPS.

Supports **Simulation Mode** (Autoware Planning Simulator) and **Vehicle Connect Mode** (Hooke2 DBW hardware).

---

## 📁 Repository Structure

```
Campus_Shuttle/
├── av_ws/                      # ROS 2 Humble workspace
│   ├── src/campus_pkg/         # Shuttle mission node (shuttle.py v4)
│   ├── edited_launch/          # Custom Autoware launch XML
│   └── college.sh              # Hardware vehicle launcher
├── map/
│   ├── pointcloud_map.pcd      # 198 MB 3D LiDAR map  [Git LFS]
│   ├── lanelet2_map.osm        # Lanelet2 vector road map
│   └── map_config.yaml         # Map coordinate origin
├── shuttle_sim_Waypoints/
│   └── waypoints.txt           # UTM waypoint coordinates
├── shuttle_dashboard.py        # Mission Control Dashboard (v7.0)
├── test_dashboard.py           # Unit tests
├── run_dashboard.sh            # Quick-launch script
├── requirements.txt            # Python dependencies
├── ARCHITECTURE.md             # 📐 Full technical architecture & diagrams
└── WALKTHROUGH.md              # 📖 Operator step-by-step guide
```

---

## ⚡ Quick Start

**Requirements:** Ubuntu 22.04 · ROS 2 Humble · Autoware Universe

```bash
# 1. Clone
git clone https://github.com/manish-gupta-in/Campus_Shuttle.git
cd Campus_Shuttle

# 2. Install Python dependencies
pip3 install -r requirements.txt

# 3. Build ROS 2 workspace
cd av_ws && colcon build --symlink-install && cd ..

# 4. Source (run in every new terminal)
source /opt/ros/humble/setup.bash
source av_ws/install/setup.bash
```

Open **3 terminals** and run in order:

| Terminal | Command |
|----------|---------|
| **1** — Autoware Simulator | `ros2 launch autoware_launch planning_simulator.launch.xml map_path:=./map` |
| **2** — Dashboard | `python3 shuttle_dashboard.py` |
| **3** — Mission Node | `ros2 run campus_pkg shuttle` |

> For hardware vehicle deployment, see [`college.sh`](av_ws/college.sh) and [WALKTHROUGH.md](WALKTHROUGH.md).

---

## 🗺️ Campus Route

10-waypoint loop — 8 passenger stops with 10-second dwell:

```
Security Gate → A-Block → Hostel Circle → CP → E-Block
→ WILP-Lab → K-Block → H-Block → I-Block → Security (End)
```

---

## ⚙️ Key Tuning Parameters (`shuttle.py`)

```python
STOP_WAIT_SEC        = 10    # Dwell time at each stop (seconds)
GOAL_PUBLISH_DELAY   = 15    # Localisation warmup delay (seconds)
PROXIMITY_RADIUS     = 5.0   # Fallback arrival detection radius (m)
START_WAYPOINT_INDEX = 0     # Start route from this waypoint index
```

---

## 📐 Architecture & Diagrams

Full system architecture, ROS 2 topic maps, FSM state machine, data flow diagrams, and component breakdown are in:

**→ [ARCHITECTURE.md](ARCHITECTURE.md)**

---

## 📖 Operator Guide

Step-by-step launch instructions, UI guide, troubleshooting, and hardware deploy:

**→ [WALKTHROUGH.md](WALKTHROUGH.md)**

---

## 🧪 Tests

```bash
python3 test_dashboard.py
```

---

## 👤 Author

**Manish Gupta** — Autonomous Systems & Robotics Engineer

[![GitHub](https://img.shields.io/badge/GitHub-manish--gupta--in-181717?logo=github)](https://github.com/manish-gupta-in)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Manish%20Gupta-0A66C2?logo=linkedin)](https://www.linkedin.com/in/manish-gupta-in/)

---

## 📄 License

[MIT License](LICENSE)
