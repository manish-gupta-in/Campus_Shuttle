# WALKTHROUGH.md — Operator Quickstart Guide

# BITS Autonomous Shuttle Mission Control Dashboard [v7.0]

Welcome to the **Autonomous Shuttle Mission Control Dashboard** for BITS Pilani Hyderabad Campus! This production release provides a professional-grade visual control environment to monitor and launch your autonomous vehicle.

---

## 1. What's New in v7.0 (Production Release)

### 🔗 Dynamic AST Waypoint Parser (Auto-Sync)
- **Problem:** Previously, modifying `shuttle.py` waypoints required manually editing `shuttle_dashboard.py` too.
- **Solution:** Built-in Python **AST (Abstract Syntax Tree) Parser** inside the dashboard. On boot, it safely reads and parses `shuttle.py` without executing any code, extracting the `WAYPOINTS` config directly.
- **Result:** The visualizer bounding box, dropdown menus, and stop lists auto-update to match any changes made in `shuttle.py`. Falls back gracefully to static config if file not found.

### 📂 Zero-Configuration Relative Path Resolution
- All file directories, map paths, and assets centralized in a single config block.
- Resolves dynamically relative to the script location (`os.path.dirname(os.path.abspath(__file__))`).
- Copy the entire directory onto any machine and run immediately — no path edits needed.

### 🖥️ Multi-Mode Bridge
- **Simulation Mode:** Spawns the Autoware Planning Simulator directly from the dashboard.
- **Vehicle Connect Mode:** Attaches directly to real vehicle telemetry via the Hooke2 DBW interface.
- Toggle between modes on-the-fly with the top-right segmented button.

### 🗺️ Sub-Millisecond Binary PCD Visualizer
- Parses the 198 MB `pointcloud_map.pcd` in under 15ms using `struct.unpack + f.seek()` byte-slicing.
- Renders 22,000 downsampled points onto the map canvas.
- PIL Image compositing → single blit operation (eliminates Linux Tcl/Tk stack overflow).

### 🛣️ Lanelet2 OSM Vector Overlay
- Parses `lanelet2_map.osm` using `xml.etree.ElementTree`.
- Extracts `local_x / local_y` node coordinates and draws road polylines.
- Shows true campus roads, lane boundaries, and intersection geometry.

### 🎥 Interactive Map & Third-Person Follower Camera
- **Scroll-wheel zoom:** 0.4× to 15× magnification.
- **Left-drag pan:** UTM-space accurate pan (pixel delta → UTM delta conversion).
- **Follow Vehicle button:** Locks camera behind shuttle; auto-disables when user pans.

### ⚡ 60 FPS Double-Buffered Rendering
- **Static layer:** PCD + OSM + stop markers (only redrawn on resize/init), cached.
- **Dynamic layer:** Vehicle, route ribbon, sparkles (redrawn every ~16ms tick under `"dynamic"` tag).
- Loop execution < 0.5ms per frame.

### 🔌 Direct ROS 2 Telemetry Bridge
- Skips heavy visualization packages.
- Subscribes directly to:
  - `/localization/kinematic_state` → X,Y pose + quaternion yaw
  - `/vehicle/status/velocity_status` → speed (m/s → km/h)
  - `/planning/route_state` → route state string
  - `/api/operation_mode/state` → AUTONOMOUS / MANUAL / LOCAL badge
- All imports delayed to inside `ROS2Bridge.start()` to prevent X11/DDS segfaults.

---

## 2. System Requirements

| Component | Requirement |
|-----------|------------|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| AV Stack | Autoware Universe |
| Python | 3.10+ |
| Display | X11 (GTK3 compatible) |
| RAM | ≥ 16 GB recommended |
| GPU | NVIDIA (for Autoware NDT localizer) |

---

## 3. Installation

```bash
# 1. Clone the repository
git clone https://github.com/manish-gupta-in/Campus_Shuttle.git
cd Campus_Shuttle

# 2. Install Python dependencies
pip3 install -r requirements.txt

# 3. Build the ROS 2 workspace
cd av_ws
colcon build --symlink-install
cd ..
```

---

## 4. Quick-Start Operation Guide

### Step 1 — Source Environments
Open each component in a separate terminal, always sourcing first:

```bash
source /opt/ros/humble/setup.bash
source /path/to/Campus_Shuttle/av_ws/install/setup.bash
```

### Step 2 — Launch Autoware Simulator (Terminal 1)

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=/path/to/Campus_Shuttle/map
```

Wait for Autoware to fully initialize (you'll see the RViz window open).

### Step 3 — Start Mission Control Dashboard (Terminal 2)

```bash
cd /path/to/Campus_Shuttle
python3 shuttle_dashboard.py
```

Or use the launcher:

```bash
bash run_dashboard.sh
```

### Step 4 — Select Starting Stop

In the dashboard's **Left Panel** → **MISSION ROUTING**:
- Select your starting stop from the dropdown (e.g., `Stop 0: Main Gate`)
- The map will center on the selected start position

### Step 5 — Launch Mission Processes

1. Click **▶ LAUNCH** under **AUTOWARE SIMULATOR** — LED turns `● ONLINE` (green)
2. Click **▶ LAUNCH** under **SHUTTLE MISSION NODE** — LED turns `● ONLINE` (green)

### Step 6 — Run the Shuttle Node (Terminal 3)

```bash
ros2 run campus_pkg shuttle
```

The FSM will begin:
1. `INIT` → `SET_INITIAL_POSE` → `WAIT_BEFORE_GOAL` (15s warmup)
2. `SET_GOAL_POSE` → `WAIT_AUTONOMOUS_READY`
3. `SWITCH_AUTONOMOUS` → `ENGAGE_CONTROL`
4. `MONITORING_PROGRESS` — shuttle drives autonomously!
5. At each stop: `JUNCTION_STOP` (10s dwell) → `RESUME_AFTER_STOP` → next goal

### Step 7 — Monitor in Real Time

As soon as Autoware publishes to `/localization/kinematic_state`:
- Vehicle icon appears on map, rotating with real yaw heading
- Speed displays in large km/h digits on right panel
- Stop sequence advances automatically
- Mission logs stream in the `💻 MISSION LOGS` tab

---

## 5. Dashboard UI Guide

### Left Panel

| Control | Function |
|---------|----------|
| Start Stop dropdown | Select which waypoint the shuttle starts from |
| AUTOWARE SIMULATOR card | Launch/stop the Autoware planning simulator |
| SHUTTLE MISSION NODE card | Launch/stop the `shuttle.py` ROS 2 node |

### Center Map Panel

| Action | Effect |
|--------|--------|
| Scroll wheel | Zoom in/out (0.4× – 15×) |
| Left-click drag | Pan the map |
| FOLLOW VEHICLE button | Lock camera behind the moving shuttle |

**Map Elements:**

- 🔵 **Cyan dots** — 22,000-point pointcloud (campus buildings, trees, curbs)
- 🟡 **Yellow lines** — Lanelet2 OSM road centerlines
- 🔴 **Red hexagons** — Junction stop waypoints
- 🟢 **Green hexagon** — Current active stop
- 🚌 **Cyan wireframe capsule** — Shuttle vehicle (with rotating LIDAR dot)
- 🟢 **Emerald ribbon** — Active navigation path

### Right Telemetry Panel

| Indicator | Shows |
|-----------|-------|
| Large number (km/h) | Current vehicle speed |
| MISSION STATE | Current FSM state (color-coded) |
| CURRENT STOP | Last/current stop name |
| NEXT STOP | Next waypoint |
| ROUTE STATE | Autoware planner route state |
| DWELL TIMER | Countdown during junction stops |
| OPERATION MODE | AUTONOMOUS / MANUAL / LOCAL |

---

## 6. Hardware Vehicle Deployment

For deploying on the physical Hooke2 DBW vehicle:

```bash
cd av_ws
bash college.sh
```

This opens 3 coordinated terminals:
1. **Autoware** with full sensor stack
2. **Autoware control interface**
3. **Shuttle node** (delayed 15s for stack warmup)

In the dashboard, switch to **VEHICLE CONNECT MODE** using the top-right segmented button.

---

## 7. Adjusting the Route

To add, remove, or modify waypoints:

1. Edit the `WAYPOINTS` list in `av_ws/src/campus_pkg/campus_pkg/shuttle.py`
2. Each waypoint is a dict: `{"label", "x", "y", "z", "xo", "yo", "zo", "wo", "is_stop"}`
3. UTM coordinates `(x, y)` must match your `map_config.yaml` origin
4. Set `"is_stop": True` for passenger boarding stops; `False` for pass-through points
5. The dashboard **automatically re-reads** the updated waypoints on next launch (AST parser)

---

## 8. Running Tests

```bash
python3 test_dashboard.py
```

Verifies:
- ✅ PCD binary parser
- ✅ Lanelet2 OSM XML parser
- ✅ AST waypoint extractor
- ✅ ROS 2 bridge initialization (mocked)
- ✅ Dashboard widget construction

---

## 9. Troubleshooting

| Problem | Solution |
|---------|----------|
| Dashboard fails to start | Check `pip3 install customtkinter Pillow` |
| Map shows no pointcloud | Verify `map/pointcloud_map.pcd` exists (198 MB file) |
| Speed always shows 0.00 | ROS 2 not running; check `ros2 topic list` |
| Vehicle doesn't move | Check FSM logs in terminal; verify autonomous mode service is ready |
| Clicking sound at stops | Upgrade to `shuttle.py v4` (already included; check `_gear_park_sent`) |
| X11/DDS crash at startup | Ensure `rclpy` not imported at module level; imports delayed inside `ROS2Bridge.start()` |
| Tcl font abort on Linux | Confirmed fixed in v7.0 (PIL blit engine + stable container binding) |

---

*For full technical architecture and ROS 2 topic maps, see [ARCHITECTURE.md](ARCHITECTURE.md)*
