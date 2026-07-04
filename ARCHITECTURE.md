# ARCHITECTURE.md — BITS Pilani Autonomous Campus Shuttle

## System Architecture Overview

This document describes the full technical architecture of the **BITS Pilani Hyderabad Campus Autonomous Shuttle** — an end-to-end autonomous vehicle navigation platform integrating Autoware Universe, ROS 2 Humble, a custom mission FSM node, and a real-time mission control dashboard.

---

## 1. High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   OPERATOR CONTROL ROOM (UI)                    │
│                                                                 │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │         Mission Control Dashboard (shuttle_dashboard.py) │   │
│   │                                                         │   │
│   │  • 3-panel CustomTkinter UI  (1460×860)                 │   │
│   │  • Sub-ms binary .pcd pointcloud visualizer             │   │
│   │  • Lanelet2 OSM road overlay parser                     │   │
│   │  • Live ROS 2 telemetry bridge (direct topic sub)       │   │
│   │  • Process lifecycle manager (SIM + SHUTTLE nodes)      │   │
│   │  • Interactive pan/zoom + 3rd-person follower cam       │   │
│   └──────────────────────────┬──────────────────────────────┘   │
└─────────────────────────────┼───────────────────────────────────┘
                              │  ROS 2 DDS (local or remote)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                  ROS 2 HUMBLE WORKSPACE  (av_ws)                │
│                                                                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │         AutowareShuttleMission Node  (shuttle.py)          │ │
│  │                                                            │ │
│  │   11-State Finite State Machine:                           │ │
│  │   INIT → SET_INITIAL_POSE → WAIT_BEFORE_GOAL →            │ │
│  │   SET_GOAL_POSE → WAIT_AUTONOMOUS_READY →                  │ │
│  │   SWITCH_AUTONOMOUS → ENGAGE_CONTROL →                     │ │
│  │   MONITORING_PROGRESS → JUNCTION_STOP →                    │ │
│  │   RESUME_AFTER_STOP → [loop] / COMPLETE                    │ │
│  └────────────────────────────┬───────────────────────────────┘ │
│                               │                                 │
│  ┌────────────────────────────▼───────────────────────────────┐ │
│  │              Autoware Universe Planning Suite               │ │
│  │                                                            │ │
│  │   • Scenario Planner      • Mission Planner                │ │
│  │   • NDT/EKF Localizer     • Behavior Planner               │ │
│  │   • MPC Trajectory Follower                                │ │
│  └────────────────────────────┬───────────────────────────────┘ │
│                               │                                 │
│  ┌────────────────────────────▼───────────────────────────────┐ │
│  │         Hooke2 DBW / Vehicle Interface (Real Mode)         │ │
│  │         Simulation Physics Engine (Sim Mode)               │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────▼───────────────────────────────────┐
│                  ENVIRONMENT & MAP SUBSYSTEM                    │
│                                                                 │
│   ┌─────────────────┐  ┌─────────────────┐  ┌───────────────┐  │
│   │  pointcloud_map │  │  lanelet2_map   │  │  waypoints    │  │
│   │  .pcd (198 MB)  │  │  .osm (Lnlt2)  │  │  .txt / .py   │  │
│   └─────────────────┘  └─────────────────┘  └───────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. ROS 2 Topic & Service Map

### Published Topics (by `shuttle.py`)

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Set AMCL/NDT initial vehicle pose |
| `/planning/mission_planning/goal` | `geometry_msgs/PoseStamped` | Send next navigation goal |
| `/planning/scenario_planning/current_max_velocity` | `std_msgs/Float32` | Velocity cap (0.0 m/s during stops) |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/GearCommand` | PARK/DRIVE gear commands |

### Subscribed Topics (by `shuttle.py`)

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/api/operation_mode/state` | `autoware_adapi_v1_msgs/OperationModeState` | Detect AUTONOMOUS mode |
| `/planning/route_state` | `autoware_planning_msgs/RouteState` | Detect GOAL_REACHED |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | Real-time speed |
| `/tf` | TF2 tree | `base_link → map` proximity check |

### Dashboard Subscriptions (direct bridge, `shuttle_dashboard.py`)

| Topic | Purpose |
|-------|---------|
| `/localization/kinematic_state` | Real-time X,Y pose + quaternion yaw |
| `/vehicle/status/velocity_status` | Speed display |
| `/planning/route_state` | Route state indicator |
| `/api/operation_mode/state` | AUTONOMOUS / MANUAL mode badge |

### Service Calls (by `shuttle.py`)

| Service | Type | Purpose |
|---------|------|---------|
| `/api/operation_mode/change_to_autonomous` | `ChangeOperationMode` | Engage autonomous mode |
| `/api/operation_mode/enable_autoware_control` | `ChangeOperationMode` | Enable Autoware lateral/longitudinal control |

---

## 3. Finite State Machine (FSM) — `shuttle.py`

```
                         ┌─────────┐
             Node Boot   │  INIT   │  2.0s system settle
                    ────►│         │──────────────────────►
                         └─────────┘
                                             ▼
                         ┌──────────────────────────────────┐
                         │       SET_INITIAL_POSE           │
                         │  Publish /initialpose to map     │
                         └──────────────┬───────────────────┘
                                        │
                                        ▼  15s warmup delay
                         ┌──────────────────────────────────┐
                         │        WAIT_BEFORE_GOAL          │
                         │  Localisation + AMCL convergence │
                         └──────────────┬───────────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────────┐
                         │          SET_GOAL_POSE           │
                         │  Publish next waypoint goal pose │
                         └──────────────┬───────────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────────┐
                         │      WAIT_AUTONOMOUS_READY       │
                         │  Wait for route_state SETTLED    │
                         └──────────────┬───────────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────────┐
                         │       SWITCH_AUTONOMOUS          │
                         │  Call change_to_autonomous srv   │
                         │  Exponential retry backoff       │
                         └──────────────┬───────────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────────┐
                         │        ENGAGE_CONTROL            │
                         │  Call enable_autoware_control    │
                         └──────────────┬───────────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────────┐
                    ┌───►│      MONITORING_PROGRESS         │◄──┐
                    │    │  Monitor via dual-trigger:       │   │
                    │    │  PRIMARY: RouteState=GOAL_REACHED│   │
                    │    │  FALLBACK: /tf proximity 5m+5s   │   │
                    │    └──────────────┬───────────────────┘   │
                    │                   │  Arrived & is_stop=True│
                    │                   ▼                       │
                    │    ┌──────────────────────────────────┐   │
                    │    │         JUNCTION_STOP            │   │
                    │    │  • Vel-cap 0.0 m/s @ 5 Hz       │   │
                    │    │  • PARK gear one-shot            │   │
                    │    │  • 10s dwell timer               │   │
                    │    └──────────────┬───────────────────┘   │
                    │                   │  Dwell complete        │
                    │                   ▼                       │
                    │    ┌──────────────────────────────────┐   │
                    │    │       RESUME_AFTER_STOP          │   │
                    │    │  Step1: Send DRIVE gear          │   │
                    │    │  Step2: 2.0s pawl settle wait    │   │
                    │    │  Step3: Release velocity cap     │   │
                    │    └──────────────┬───────────────────┘   │
                    │                   │  Advance waypoint idx  │
                    └───────────────────┘  → SET_GOAL_POSE      │
                                        │                       │
                                        │  Final waypoint reached
                                        ▼
                         ┌──────────────────────────────────┐
                         │           COMPLETE               │
                         │  Cancel timers, safe shutdown    │
                         └──────────────────────────────────┘
```

---

## 4. Dual-Trigger Arrival Detection

To prevent mission failures from single-point-of-failure route detection:

```
ARRIVAL EVENT
      │
      ├─ PRIMARY TRIGGER ────────────────────────────────────────►
      │   Autoware RouteState == GOAL_REACHED (state=6)
      │   Fired by: route_state_callback() → _trigger_arrival()
      │   Latency: ~100ms after Autoware planner settles
      │
      └─ FALLBACK TRIGGER ──────────────────────────────────────►
          /tf proximity check at 5 Hz (every 0.2s)
          Conditions:   dist(base_link → goal) ≤ 5.0 m
                    AND speed ≤ 0.2 m/s
                    AND confirmed for 5.0 continuous seconds
          Fired by: proximity_check_callback() → _trigger_arrival()
```

---

## 5. Dashboard Architecture

```
MissionControlDashboard (CTk Window, 1460×860)
│
├── Header Bar
│   ├── BITS WILP Logo (PNG)
│   ├── Car Asset (JPEG)
│   ├── Title + Subtitle Labels
│   ├── Mode Selector (SIMULATION / VEHICLE CONNECT)
│   └── Real-time IST Clock
│
├── Left Panel (320px, scrollable)
│   ├── Mission Routing Section
│   │   └── Start Stop OptionMenu (dynamic from shuttle.py AST)
│   ├── Process Runners Section
│   │   ├── Autoware Simulator Card [▶ LAUNCH / ■ STOP]
│   │   └── Shuttle Mission Node Card [▶ LAUNCH / ■ STOP]
│   └── Live Mission Data Section
│       ├── Speed Gauge (km/h, large display)
│       ├── Mission State Badge (color-coded)
│       ├── Waypoint Progress Bar
│       ├── Current / Next Stop Labels
│       ├── Route State Indicator
│       ├── Dwell Timer
│       └── Operation Mode Badge
│
├── Center Panel (TabView)
│   ├── Tab 1: BITS PILANI MAP (PCD + OSM)
│   │   ├── tk.Canvas (dark #050811 background)
│   │   ├── Static Layer (PCD points + OSM roads + stop markers)
│   │   │   ├── 22,000 downsampled pointcloud dots
│   │   │   ├── Lanelet2 OSM road polylines
│   │   │   └── Stop hexagon markers (red/green)
│   │   ├── Dynamic Layer (re-rendered each frame)
│   │   │   ├── Active route ribbon (emerald green)
│   │   │   ├── Wireframe vehicle capsule (cyan)
│   │   │   ├── LIDAR roof dot (rotating)
│   │   │   └── Exhaust sparkle particles
│   │   └── Overlay: "FOLLOW VEHICLE" toggle button
│   │
│   └── Tab 2: MISSION LOGS
│       └── CTkTextbox (color-coded by severity)
│
└── Right Panel (320px)
    ├── Speed Arc Gauge (canvas drawn)
    ├── Mission State Label
    ├── Stop Segment Progress Bar
    ├── Telemetry Cards (Current Stop, Next Stop, Route State)
    ├── Dwell Countdown Timer
    └── Operation Mode Badge
```

---

## 6. Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                       DATA FLOW                                 │
│                                                                 │
│  [PCD File]  ─── _parse_pcd_points() ──► [22K point list]      │
│                  (struct.unpack 'fff')     (x, y tuples)        │
│                                                                 │
│  [OSM File]  ─── _parse_osm_lanelet() ──► [way polylines]      │
│                  (xml.etree + local_x/y)  (list of (x,y) lists)│
│                                                                 │
│  [shuttle.py] ── AST parser ──────────► [WAYPOINTS dict]       │
│                  (ast.literal_eval)       (dynamic sync)        │
│                                                                 │
│  [ROS 2 Topics] ── ROS2Bridge thread ──► [telemetry dict]      │
│   /localization/kinematic_state           pose, heading         │
│   /vehicle/status/velocity_status         speed (m/s)           │
│   /planning/route_state                   route state string    │
│   /api/operation_mode/state               mode string           │
│                                                                 │
│  [Telemetry Dict] ── _run_render_loop() ──► Canvas redraws     │
│                      (~60 FPS, 16ms tick)   (dynamic layer only)│
└─────────────────────────────────────────────────────────────────┘
```

---

## 7. Campus Route Waypoints (UTM Coordinates)

| # | Stop | UTM-X | UTM-Y | Is Stop |
|---|------|-------|-------|---------|
| 0 | Security Main Gate | 42302.34 | 41729.12 | ❌ Start |
| 1 | A-Block | 42220.48 | 41821.66 | ✅ |
| 2 | Hostel Circle | 42194.18 | 41997.10 | ✅ |
| 3 | CP (Central Plaza) | 42241.18 | 42075.82 | ✅ |
| 4 | E-Block | 42365.14 | 42086.45 | ✅ |
| 5 | WILP-Lab | 42577.55 | 42049.71 | ✅ |
| 6 | K-Block | 42601.29 | 41917.12 | ✅ |
| 7 | H-Block | 42532.79 | 41806.83 | ✅ |
| 8 | I-Block | 42559.08 | 41746.37 | ✅ |
| 9 | Security (End) | 42314.15 | 41721.62 | ❌ Final |

---

## 8. Key v4 Bug Fixes & Optimizations

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Clicking sound at stops | `GearCommand(PARK)` published continuously at 2 Hz; Hooke2 DBW re-actuates on every message | One-shot `_gear_park_sent` flag; gear published **exactly once** per stop leg |
| Park re-engaging after DRIVE | `_brake_hold_active` flag still `True` during RESUME; timer kept sending PARK overriding DRIVE | Split into two independent flags: `_vel_cap_active` (keepalive needed) and `_gear_park_sent` (one-shot) |
| Multiple ENGAGE calls | `_brake_hold_active` shared between JUNCTION_STOP and RESUME paths | `_gear_park_sent` is per-leg, reset only in `_advance_to_next_waypoint()` |
| RESUME_AFTER_STOP never fired | Complex multi-flag dependency; state machine could miss transition | Explicit `elapsed-only` gate for clarity |
| Speed always 0.00 m/s | Wrong message type; needed `VelocityReport.longitudinal_velocity` | Correct import + `abs()` extraction |
| Autonomous retry storm | No backoff on failed service calls | Exponential backoff: 1s → 2s → 4s → ... → 16s max |
