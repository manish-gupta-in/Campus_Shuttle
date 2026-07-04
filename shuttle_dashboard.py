#!/usr/bin/env python3
"""
BITS Pilani · Hyderabad Campus
Autonomous Shuttle Mission Control Dashboard [v7.0 - Production]
Rebuilt with direct Autoware topic bridge to '/localization/kinematic_state',
Real-time yaw orientation extraction, sub-millisecond binary .pcd pointcloud loader,
Lanelet2 OSM vector road overlays, interactive map zooming & panning,
and automatic 3rd-person vehicle follower perspective tracking.
"""

import sys
import os
import math
import time
import re
import queue
import signal
import subprocess
import threading
import struct
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta

import customtkinter as ctk
import tkinter as tk
from PIL import Image, ImageTk

# Set appearance and default theme
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ─── ROS2 Message Imports (Delayed inside class methods to prevent X11 display/DDS segfaults!) ───
import importlib.util
ROS2_AVAILABLE = importlib.util.find_spec("rclpy") is not None

# ─── Colors ───────────────────────────────────────────────────────────────────
BG_COLOR       = "#080d1a"  # Cyber Deep Navy
PANEL_COLOR    = "#0f172a"  # Dark Slate Slate
CARD_COLOR     = "#1e293b"  # High-contrast card
BORDER_COLOR   = "#334155"  # Card Border
ACCENT_CYAN    = "#00f0ff"  # Neon Cyan
ACCENT_PURPLE  = "#bd00ff"  # Neon Purple
COLOR_GREEN    = "#00ff88"  # Neon Green
COLOR_AMBER    = "#fbbf24"  # Neon Gold/Yellow
COLOR_RED      = "#ff0055"  # Neon Red/Pink
TEXT_WHITE     = "#f8fafc"  # Clean White
TEXT_MUTED     = "#94a3b8"  # Slate Muted Blue
ROAD_COLOR     = "#0f172a"  # Dark Asphalt Gray
ROAD_LINE_COL  = "#fbbf24"  # Yellow Center Stripes
OSM_ROAD_COLOR = "#475569"  # Sleek vector roadnet color
PCD_POINT_COL  = "#384a62"  # RViz-style pointcloud background dots (increased intensity)

# ─── BITS Campus Waypoints (UTM coordinates matching shuttle.py exactly) ──────
def load_waypoints_from_shuttle():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    shuttle_path = os.path.join(base_dir, "av_ws", "src", "campus_pkg", "campus_pkg", "shuttle.py")
    if not os.path.exists(shuttle_path):
        return None
    try:
        import ast
        with open(shuttle_path, "r", encoding="utf-8") as f:
            tree = ast.parse(f.read())
        
        # Traverse AST to find assignment of WAYPOINTS
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == "WAYPOINTS":
                        # Safely evaluate the literal assignment to Python object
                        waypoints = ast.literal_eval(node.value)
                        if isinstance(waypoints, list) and len(waypoints) > 0:
                            # Normalize short labels if missing
                            for w in waypoints:
                                if "short" not in w:
                                    w["short"] = w["label"].split()[0] if w["label"] else "WP"
                            return waypoints
    except Exception as e:
        print("Failed to dynamically parse waypoints from shuttle.py:", e)
    return None

STATIC_WAYPOINTS = [
    {
        "label": "Security Main Gate", "short": "Main Gate",
        "x": 42302.34375, "y": 41729.1171875, "z": -0.25177824,
        "xo": 0.00689144, "yo": -0.00357485, "zo": 0.88764836, "wo": 0.46045641,
        "is_stop": False
    },
    {
        "label": "A-Block", "short": "A-Block",
        "x": 42220.4765625, "y": 41821.6640625, "z": -1.319,
        "xo": 0.0027, "yo": -0.0016, "zo": 0.8635, "wo": 0.5042,
        "is_stop": True
    },
    {
        "label": "Hostel Circle", "short": "Hostel Circle",
        "x": 42194.18359375, "y": 41997.09765625, "z": 5.48,
        "xo": 0.0113, "yo": -0.0125, "zo": 0.6717, "wo": 0.7406,
        "is_stop": True
    },
    {
        "label": "CP", "short": "CP",
        "x": 42241.18359375, "y": 42075.81640625, "z": 8.18,
        "xo": 0.0045, "yo": -0.0071, "zo": 0.5364, "wo": 0.8439,
        "is_stop": True
    },
    {
        "label": "E-Block", "short": "E-Block",
        "x": 42365.14303504, "y": 42086.45246682, "z": 14.44,
        "xo": -0.0041, "yo": 0.0069, "zo": 0.5140, "wo": 0.8577,
        "is_stop": True
    },
    {
        "label": "WILP-Lab", "short": "WILP-Lab",
        "x": 42577.55078125, "y": 42049.7109375, "z": 15.98,
        "xo": -9.84e-05, "yo": 0.0013, "zo": 0.0730, "wo": 0.9973,
        "is_stop": True
    },
    {
        "label": "K-Block", "short": "K-Block",
        "x": 42601.28854122, "y": 41917.11737506, "z": 6.68,
        "xo": 0.0076, "yo": 0.0042, "zo": -0.8769, "wo": 0.4804,
        "is_stop": True
    },
    {
        "label": "H-Block", "short": "H-Block",
        "x": 42532.79296875, "y": 41806.828125, "z": 2.68,
        "xo": 0.0178, "yo": 0.0096, "zo": -0.8784, "wo": 0.4773,
        "is_stop": True
    },
    {
        "label": "I-Block", "short": "I-Block",
        "x": 42559.07603364, "y": 41746.37363606, "z": -3.71,
        "xo": 0.0046, "yo": 0.0129, "zo": -0.3391, "wo": 0.9406,
        "is_stop": True
    },
    {
        "label": "Security (End)", "short": "Sec. End",
        "x": 42314.15061019, "y": 41721.6197422, "z": -0.20,
        "xo": 0.0073, "yo": -0.0012, "zo": -0.9860, "wo": -0.1661,
        "is_stop": False
    }
]

# Load waypoints dynamically from shuttle.py if available, otherwise fall back to static list
WAYPOINTS = load_waypoints_from_shuttle()
if not WAYPOINTS:
    WAYPOINTS = STATIC_WAYPOINTS

# Build STOP_INDEX dynamically from WAYPOINTS
STOP_INDEX = {w["label"].lower(): i for i, w in enumerate(WAYPOINTS)}

# For backwards compatibility with string matches, map standard stops to their matching waypoint label index if present
for i, w in enumerate(WAYPOINTS):
    label_lower = w["label"].lower()
    if "main" in label_lower or "gate" in label_lower:
        STOP_INDEX["main gate"] = i
    if "hostel" in label_lower:
        STOP_INDEX["hostel"] = i
    if "wilp" in label_lower:
        STOP_INDEX["wilp lab"] = i
    if "end" in label_lower:
        STOP_INDEX["security end"] = i

STATE_MAP = {
    "IDLE":                  ("SYSTEM READY",         TEXT_MUTED),
    "INIT":                  ("INITIALISING…",        TEXT_MUTED),
    "SET_INITIAL_POSE":      ("SETTING INITIAL POSE", ACCENT_CYAN),
    "WAIT_BEFORE_GOAL":      ("LOCALISING SHUTTLE",   COLOR_AMBER),
    "WAIT_AUTONOMOUS_READY": ("WAITING AUTO ENGAGE",  COLOR_AMBER),
    "SWITCH_AUTONOMOUS":     ("ENGAGING AUTONOMOUS",  ACCENT_PURPLE),
    "ENGAGE_CONTROL":        ("ENGAGING CONTROL",     ACCENT_PURPLE),
    "MONITORING_PROGRESS":   ("MISSION ACTIVE 🚌",    COLOR_GREEN),
    "JUNCTION_STOP":         ("HOLDING AT DWELL 🛑",  COLOR_RED),
    "RESUME_AFTER_STOP":     ("RESUMING MISSION",     COLOR_AMBER),
    "SET_GOAL_POSE":         ("PUBLISHING NEXT GOAL", ACCENT_CYAN),
    "COMPLETE":              ("MISSION COMPLETE 🎉",  COLOR_GREEN),
}

# ══════════════════════════════════════════════════════════════════════════════
#  DASHBOARD PATHS & DIRECTORY CONFIGURATION (ONE-PLACE CONFIG)
# ══════════════════════════════════════════════════════════════════════════════
# Base directory where the shuttle simulation workspace/files reside.
# Change this directory path when running on a different system.
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ROS2 Workspace path (used for sourcing and starting nodes)
# Change this path if your ROS2 workspace is located elsewhere.
WS_PATH = os.path.join(BASE_DIR, "av_ws")

# Maps path for Autoware planning simulation
# Change this path to point to your Autoware simulator maps directory if different.
MAPS_PATH = os.path.join(WS_PATH, "maps")

# Map files for the dashboard visualizer
# By default, these resolve relative to BASE_DIR/map/
LANELET2_OSM_MAP = os.path.join(BASE_DIR, "map", "lanelet2_map.osm")
POINTCLOUD_PCD_MAP = os.path.join(BASE_DIR, "map", "pointcloud_map.pcd")

# Asset image paths
LOGO_IMAGE_PATH = os.path.join(BASE_DIR, "wilp_logo.png")
CAR_IMAGE_PATH  = os.path.join(BASE_DIR, "car.jpeg")
# ══════════════════════════════════════════════════════════════════════════════

SIM_LAUNCH  = (
    f"source /opt/ros/humble/setup.bash && "
    f"source {WS_PATH}/install/setup.bash && "
    f"ros2 launch autoware_launch planning_simulator.launch.xml map_path:={MAPS_PATH}"
)
SHUTTLE_RUN = (
    f"source /opt/ros/humble/setup.bash && "
    f"source {WS_PATH}/install/setup.bash && "
    f"ros2 run campus_pkg shuttle"
)

HOME_DIR = os.path.expanduser("~")
VEHICLE_LAUNCH = (
    f"mkdir -p {HOME_DIR}/project/autoware/src/launcher/autoware_launch/autoware_launch/launch && "
    f"cp {WS_PATH}/edited_launch/autoware.launch.xml {HOME_DIR}/project/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml && "
    f"gnome-terminal -- bash -i -c 'source ~/.bashrc; source ~/.autoware_start.sh; setup_aw_ws; sleep 5; sa; exec bash' & "
    f"gnome-terminal -- bash -i -c 'source ~/.bashrc; source ~/.autoware_start.sh; setup_aw_ws; sleep 5; echo \"c2\" | autoware; exec bash'"
)

# ─── Bounding Box Coordinates of UTM waypoints ────────────────────────────────
X_COORDS = [w["x"] for w in WAYPOINTS]
Y_COORDS = [w["y"] for w in WAYPOINTS]
MIN_X, MAX_X = min(X_COORDS), max(X_COORDS)
MIN_Y, MAX_Y = min(Y_COORDS), max(Y_COORDS)
SPAN_X = MAX_X - MIN_X
SPAN_Y = MAX_Y - MIN_Y

# ==============================================================================
#  PROCESS LIFECYCLE MANAGER
# ==============================================================================
class ProcessManager:
    def __init__(self, name, command, log_queue, on_start, on_stop):
        self.name = name
        self.command = command
        self.log_queue = log_queue
        self.on_start = on_start
        self.on_stop = on_stop
        self.proc = None
        self.thread = None
        self.is_running = False

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        self.on_start()
        try:
            self.proc = subprocess.Popen(
                ["bash", "-c", self.command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                start_new_session=True
            )
            for line in self.proc.stdout:
                if not self.is_running:
                    break
                self.log_queue.put((self.name, line.rstrip()))
            self.proc.wait()
        except Exception as e:
            self.log_queue.put(("SYSTEM", f"[{self.name}] Process error: {e}"))
        finally:
            self.is_running = False
            self.on_stop()

    def stop(self):
        if not self.is_running or not self.proc:
            return
        self.is_running = False
        try:
            pgid = os.getpgid(self.proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(0.6)
            try:
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass
        except Exception:
            try:
                self.proc.terminate()
            except Exception:
                pass
        self.proc = None

# ==============================================================================
#  ROS2 DIRECT BRIDGE
# ==============================================================================
class ROS2Bridge:
    def __init__(self, telemetry_callback):
        self.callback = telemetry_callback
        self.node = None
        self.thread = None
        self.active = False

    def start(self):
        if not ROS2_AVAILABLE:
            return False
        if self.active:
            return True
        try:
            global rclpy, Node, VelocityReport, RouteState, Odometry, OperationModeState
            import rclpy
            from rclpy.node import Node
            
            # Safely attempt to import message structures to prevent crashes on systems missing custom Autoware definitions
            try:
                from autoware_vehicle_msgs.msg import VelocityReport
            except ImportError:
                VelocityReport = None
                print("[ROS2 Bridge] warning: autoware_vehicle_msgs.msg.VelocityReport not available.")

            try:
                from autoware_planning_msgs.msg import RouteState
            except ImportError:
                RouteState = None
                print("[ROS2 Bridge] warning: autoware_planning_msgs.msg.RouteState not available.")

            try:
                from nav_msgs.msg import Odometry
            except ImportError:
                Odometry = None
                print("[ROS2 Bridge] warning: nav_msgs.msg.Odometry not available.")

            try:
                from autoware_adapi_v1_msgs.msg import OperationModeState
            except ImportError:
                OperationModeState = None
                print("[ROS2 Bridge] warning: autoware_adapi_v1_msgs.msg.OperationModeState not available.")

            rclpy.init(args=None)
            self.node = Node('dashboard_direct_bridge')
            
            # Setup standard topic subscriptions (only if message types are supported on this system)
            if VelocityReport is not None:
                self.node.create_subscription(
                    VelocityReport,
                    '/vehicle/status/velocity_status',
                    self._vel_callback,
                    10
                )
            if RouteState is not None:
                self.node.create_subscription(
                    RouteState,
                    '/planning/route_state',
                    self._route_callback,
                    10
                )
            if Odometry is not None:
                self.node.create_subscription(
                    Odometry,
                    '/localization/kinematic_state',
                    self._odom_callback,
                    10
                )
            if OperationModeState is not None:
                self.node.create_subscription(
                    OperationModeState,
                    '/api/operation_mode/state',
                    self._op_mode_callback,
                    10
                )
            
            self.active = True
            self.thread = threading.Thread(target=self._spin, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print("Failed to initialize ROS2 Bridge Node:", e)
            return False

    def _spin(self):
        while self.active and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                print("ROS2 spin error:", e)
                break
        self.active = False

    def _vel_callback(self, msg):
        speed = abs(msg.longitudinal_velocity)
        self.callback('speed', speed)

    def _route_callback(self, msg):
        states = {1: "INITIALIZING", 2: "UNROUTED", 3: "ROUTING", 4: "ARRIVED", 5: "ARRIVED_GOAL", 6: "TRANSITING"}
        self.callback('route_state', states.get(msg.state, f"STATE_{msg.state}"))

    def _odom_callback(self, msg):
        # Extract precise X, Y coordinates
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.callback('pose', (x, y))
        
        # Extract precise yaw heading angle from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.callback('heading', yaw)
        
        # Extract real-time speed from kinematic twist state
        speed = abs(msg.twist.twist.linear.x)
        self.callback('speed', speed)

    def _op_mode_callback(self, msg):
        modes = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "MANUAL", 4: "LOCAL", 5: "REMOTE"}
        mode_str = modes.get(msg.mode, f"UNKNOWN ({msg.mode})")
        self.callback('operation_mode', mode_str)

    def stop(self):
        self.active = False
        if self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        try:
            rclpy.shutdown()
        except Exception:
            pass

# ==============================================================================
#  MISSION CONTROL DASHBOARD (MAIN WINDOW)
# ==============================================================================
class MissionControlDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        self.title("BITS PILANI · CAMPUS AUTONOMOUS SHUTTLE MISSION CONTROL")
        self.geometry("1460x860")
        self.minsize(1200, 720)
        self.configure(fg_color=BG_COLOR)

        # ── State Initialization ──────────────────────────────────────────────
        self.running      = False
        self.log_q        = queue.Queue()
        self.ros_bridge   = None
        
        # Processes
        self.sim_manager  = ProcessManager("SIM", SIM_LAUNCH, self.log_q, self._on_sim_start, self._on_sim_stop)
        self.node_manager = ProcessManager("SHUTTLE", SHUTTLE_RUN, self.log_q, self._on_shuttle_start, self._on_shuttle_stop)

        # Live telemetry
        self.spd          = 0.0
        self.dist         = 0.0
        self.curr_stop    = "—"
        self.nxt_stop     = "—"
        self.rt_state     = "—"
        self.dwell_left   = 0.0
        self.op_mode      = "—"
        self.mission_st   = "IDLE"
        
        self.seg_from     = 0
        self.seg_to       = 0
        self.vehicle_x    = WAYPOINTS[0]["x"]
        self.vehicle_y    = WAYPOINTS[0]["y"]
        self.vehicle_yaw  = 0.0
        self.use_ros_tf   = False
        self.is_vehicle_connect = False
        self.elapsed_s    = 0
        
        self._t           = 0.0
        self._pulse       = 0.0
        self._sparkles    = []
        self._grid_offset = 0.0
        self._clock_tick  = False

        # Camera Zooming & Tracking State Variables
        self.zoom_level     = 1.0
        self.center_x       = MIN_X + SPAN_X / 2
        self.center_y       = MIN_Y + SPAN_Y / 2
        self.follow_vehicle = False
        
        self._drag_last_x   = 0
        self._drag_last_y   = 0

        # Loading High-Performance Static Vector Map & Downsampled PCD Points
        self.static_drawn = False
        self.osm_ways     = []
        self.pcd_points   = []
        
        if os.path.exists(LANELET2_OSM_MAP):
            self.osm_ways = self._parse_osm_lanelet(LANELET2_OSM_MAP)

        if os.path.exists(POINTCLOUD_PCD_MAP):
            self.pcd_points = self._parse_pcd_points(POINTCLOUD_PCD_MAP)

        self._build_interface()
        self._initialize_bridge()
        self._tick_system_clock()
        self._run_render_loop()

    # ══════════════════════════════════════════════════════════════════════════
    #  BINARY PCD & LANELET2 XML PARSERS (SUB-MILLISECOND SEEK LOADER)
    # ══════════════════════════════════════════════════════════════════════════
    def _parse_pcd_points(self, pcd_path):
        try:
            with open(pcd_path, 'rb') as f:
                header_size = 0
                points_total = 0
                for _ in range(50):
                    line = f.readline()
                    header_size += len(line)
                    if b'POINTS' in line:
                        points_total = int(line.split()[-1])
                    if b'DATA' in line:
                        break
                
                if points_total == 0:
                    return []
                
                # Slicing downsampled density points instantly via f.seek()
                pts = []
                stride = max(1, points_total // 22000)
                for i in range(22000):
                    offset = header_size + i * stride * 12
                    if offset >= os.path.getsize(pcd_path):
                        break
                    f.seek(offset)
                    data = f.read(12)
                    if len(data) == 12:
                        x, y, z = struct.unpack('fff', data)
                        pts.append((x, y))
                return pts
        except Exception as e:
            print("Failed to parse binary PCD pointcloud:", e)
            return []

    def _parse_osm_lanelet(self, osm_path):
        try:
            tree = ET.parse(osm_path)
            root = tree.getroot()
            
            nodes = {}
            for node in root.findall('node'):
                nid = node.get('id')
                lx = None
                ly = None
                for tag in node.findall('tag'):
                    k = tag.get('k')
                    v = tag.get('v')
                    if k == 'local_x':
                        lx = float(v)
                    elif k == 'local_y':
                        ly = float(v)
                if lx is not None and ly is not None:
                    nodes[nid] = (lx, ly)
                    
            ways = []
            for way in root.findall('way'):
                pts = []
                for nd in way.findall('nd'):
                    ref = nd.get('ref')
                    if ref in nodes:
                        pts.append(nodes[ref])
                if len(pts) >= 2:
                    ways.append(pts)
            return ways
        except Exception as e:
            print("Failed to parse lanelet2 OSM map:", e)
            return []

    # ══════════════════════════════════════════════════════════════════════════
    #  GUI LAYOUT & CONSTRUCTION
    # ══════════════════════════════════════════════════════════════════════════
    def _build_interface(self):
        # Configure overall grid structure
        self.grid_rowconfigure(0, weight=0)  # Header
        self.grid_rowconfigure(1, weight=1)  # Body content
        self.grid_columnconfigure(0, weight=0, minsize=320)  # Left panel
        self.grid_columnconfigure(1, weight=1)                # Center visualizer
        self.grid_columnconfigure(2, weight=0, minsize=320)  # Right panel

        # ─── HEADER PANEL ─────────────────────────────────────────────────────
        hdr_frame = ctk.CTkFrame(self, height=72, fg_color=PANEL_COLOR, corner_radius=0, border_width=1, border_color=BORDER_COLOR)
        hdr_frame.grid(row=0, column=0, columnspan=3, sticky="nsew")
        hdr_frame.grid_propagate(False)

        logo_f = ctk.CTkFrame(hdr_frame, fg_color="transparent")
        logo_f.pack(side="left", padx=16, fill="y")
        self._render_logos(logo_f)

        title_f = ctk.CTkFrame(hdr_frame, fg_color="transparent")
        title_f.pack(side="left", fill="y", pady=10)
        ctk.CTkLabel(title_f, text="AUTONOMOUS SHUTTLE MISSION CONTROL",
                     font=("Courier New", 18, "bold"), text_color=ACCENT_CYAN).pack(anchor="sw")
        ctk.CTkLabel(title_f, text="BITS Pilani · Hyderabad Campus  ·  Autoware Mission Control Room [v7.0]",
                     font=("Courier New", 10), text_color=TEXT_MUTED).pack(anchor="nw")

        mode_f = ctk.CTkFrame(hdr_frame, fg_color="transparent")
        mode_f.pack(side="right", padx=(20, 10), fill="y")
        
        self.mode_btn = ctk.CTkSegmentedButton(
            mode_f, 
            values=["SIMULATION MODE", "VEHICLE CONNECT MODE"],
            selected_color=ACCENT_PURPLE,
            font=("Courier New", 10, "bold"),
            command=self._switch_operating_mode
        )
        self.mode_btn.pack(side="left", padx=10, pady=10)
        self.mode_btn.set("SIMULATION MODE")

        self.clock_lbl = ctk.CTkLabel(hdr_frame, text="00:00:00 IST", font=("Courier New", 20, "bold"), text_color=ACCENT_CYAN)
        self.clock_lbl.pack(side="right", padx=20)

        # ─── LEFT CONTROL PANEL ───────────────────────────────────────────────
        left_p = ctk.CTkScrollableFrame(self, fg_color=PANEL_COLOR, border_width=1, border_color=BORDER_COLOR)
        left_p.grid(row=1, column=0, sticky="nsew", padx=(10, 4), pady=10)
        self._build_left_controls(left_p)

        # ─── CENTER PANELS ────────────────────────────────────────────────────
        center_f = ctk.CTkFrame(self, fg_color="transparent")
        center_f.grid(row=1, column=1, sticky="nsew", padx=4, pady=10)
        center_f.grid_rowconfigure(0, weight=1)
        center_f.grid_columnconfigure(0, weight=1)

        self.tab_stack = ctk.CTkTabview(center_f, fg_color=PANEL_COLOR, border_width=1, border_color=BORDER_COLOR)
        self.tab_stack.grid(row=0, column=0, sticky="nsew")
        
        tab_map = self.tab_stack.add("🗺️  BITS PILANI MAP (PCD + OSM)")
        tab_logs = self.tab_stack.add("💻  MISSION LOGS")

        # Tab 1: Map Canvas
        self.map_canvas = tk.Canvas(tab_map, bg="#050811", highlightthickness=0)
        self.map_canvas.pack(fill="both", expand=True, padx=4, pady=4)
        
        # Trigger static map redraws on window resizing
        self.map_canvas.bind("<Configure>", self._on_canvas_resize)

        # Scroll Wheel & Mouse Drag Panning Bindings
        self.map_canvas.bind("<Button-4>", self._on_zoom_in)
        self.map_canvas.bind("<Button-5>", self._on_zoom_out)
        self.map_canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.map_canvas.bind("<ButtonPress-1>", self._on_pan_start)
        self.map_canvas.bind("<B1-Motion>", self._on_pan_drag)

        # Floating Camera Auto-Follow Button
        self._btn_follow = ctk.CTkButton(
            tab_map,
            text="FOLLOW VEHICLE",
            width=140,
            height=30,
            fg_color=CARD_COLOR,
            text_color=TEXT_MUTED,
            border_width=1,
            border_color=BORDER_COLOR,
            hover_color=ACCENT_PURPLE,
            command=self._toggle_follow_vehicle
        )
        self._btn_follow.place(relx=0.98, rely=0.03, anchor="ne")

        # Tab 2: Code Highlight Terminal
        self.terminal_log = ctk.CTkTextbox(tab_logs, fg_color="#030712", text_color=TEXT_MUTED, font=("Courier New", 12),
                                           activate_scrollbars=True, border_width=1, border_color=BORDER_COLOR)
        self.terminal_log.pack(fill="both", expand=True, padx=8, pady=8)
        self.terminal_log.tag_config("INFO",  foreground=TEXT_MUTED)
        self.terminal_log.tag_config("WARN",  foreground=COLOR_AMBER)
        self.terminal_log.tag_config("ERROR", foreground=COLOR_RED)
        self.terminal_log.tag_config("SYS",   foreground="#38bdf8")
        self.terminal_log.tag_config("ROUTE", foreground="#d8b4fe")
        self.terminal_log.tag_config("DEMO",  foreground="#fbcfe8")

        # ─── RIGHT TELEMETRY PANEL ────────────────────────────────────────────
        right_p = ctk.CTkFrame(self, fg_color=PANEL_COLOR, border_width=1, border_color=BORDER_COLOR)
        right_p.grid(row=1, column=2, sticky="nsew", padx=(4, 10), pady=10)
        self._build_right_telemetry(right_p)

    def _render_logos(self, parent):
        logo_paths = [LOGO_IMAGE_PATH, "./wilp_logo.png", "/home/bits/Desktop/Manish/Shuttle_Simulation/wilp_logo.png"]
        car_paths = [CAR_IMAGE_PATH, "./car.jpeg", "/home/bits/Desktop/Manish/Shuttle_Simulation/car.jpeg"]
        
        self.logo_tk = None
        for path in logo_paths:
            if os.path.exists(path):
                try:
                    img = Image.open(path)
                    img = img.resize((100, 40), Image.Resampling.LANCZOS)
                    self.logo_tk = ImageTk.PhotoImage(img)
                    lbl = tk.Label(parent, image=self.logo_tk, bg=PANEL_COLOR)
                    lbl.pack(side="left", padx=4)
                    break
                except Exception:
                    pass
        if not self.logo_tk:
            ctk.CTkLabel(parent, text="🎓 BITS WILP", font=("Courier New", 12, "bold"), text_color=ACCENT_CYAN).pack(side="left", padx=4)

        self.car_tk = None
        for path in car_paths:
            if os.path.exists(path):
                try:
                    img = Image.open(path)
                    img = img.resize((48, 40), Image.Resampling.LANCZOS)
                    self.car_tk = ImageTk.PhotoImage(img)
                    lbl = tk.Label(parent, image=self.car_tk, bg=PANEL_COLOR)
                    lbl.pack(side="left", padx=4)
                    break
                except Exception:
                    pass

    def _on_canvas_resize(self, event):
        self.static_drawn = False

    # ── Map Zooming & Panning Handlers ───────────────────────────────────────
    def _on_zoom_in(self, event):
        self.zoom_level = min(15.0, self.zoom_level * 1.15)
        self.static_drawn = False

    def _on_zoom_out(self, event):
        self.zoom_level = max(0.4, self.zoom_level / 1.15)
        self.static_drawn = False

    def _on_mousewheel(self, event):
        if event.delta > 0:
            self.zoom_level = min(15.0, self.zoom_level * 1.15)
        elif event.delta < 0:
            self.zoom_level = max(0.4, self.zoom_level / 1.15)
        self.static_drawn = False

    def _on_pan_start(self, event):
        self._drag_last_x = event.x
        self._drag_last_y = event.y
        # If user drags, disable auto-follower mode so they can pan freely
        if self.follow_vehicle:
            self.follow_vehicle = False
            self._btn_follow.configure(text_color=TEXT_MUTED, fg_color=CARD_COLOR)

    def _on_pan_drag(self, event):
        dx = event.x - self._drag_last_x
        dy = event.y - self._drag_last_y
        self._drag_last_x = event.x
        self._drag_last_y = event.y

        # Convert canvas drag delta pixels into UTM delta based on scale and zoom
        W = self.map_canvas.winfo_width()
        H = self.map_canvas.winfo_height()
        pad = 50
        scale_x = (W - 2 * pad) / SPAN_X
        scale_y = (H - 2 * pad) / SPAN_Y
        base_scale = min(scale_x, scale_y)

        utm_dx = dx / (base_scale * self.zoom_level)
        utm_dy = dy / (base_scale * self.zoom_level)

        self.center_x -= utm_dx
        self.center_y += utm_dy  # Inverted Y
        self.static_drawn = False

    def _toggle_follow_vehicle(self):
        self.follow_vehicle = not self.follow_vehicle
        if self.follow_vehicle:
            self._btn_follow.configure(text_color=BG_COLOR, fg_color=COLOR_GREEN)
            self._log_terminal("🎥 Enabled Third-Person Follower Perspective.", "SYS")
        else:
            self._btn_follow.configure(text_color=TEXT_MUTED, fg_color=CARD_COLOR)
            self._log_terminal("🎥 Disabled Third-Person Follower Perspective.", "WARN")
        self.static_drawn = False

    # ── Left Panel Elements ───────────────────────────────────────────────────
    def _build_left_controls(self, parent):
        def header_divider(title):
            divider = ctk.CTkFrame(parent, fg_color="transparent")
            divider.pack(fill="x", pady=(14, 4))
            ctk.CTkLabel(divider, text=title, font=("Courier New", 10, "bold"), text_color=TEXT_MUTED).pack(side="left")
            ctk.CTkFrame(divider, fg_color=BORDER_COLOR, height=1).pack(side="left", fill="x", expand=True, padx=(8, 0))

        # 1. Waypoint Selector
        header_divider("MISSION ROUTING")
        ctk.CTkLabel(parent, text="Choose Starting Stop:", font=("Courier New", 11), text_color=TEXT_WHITE).pack(anchor="w", padx=6)
        
        waypoint_labels = [f"Stop {i}: {w['short']}" for i, w in enumerate(WAYPOINTS)]
        self.start_drop = ctk.CTkOptionMenu(
            parent, 
            values=waypoint_labels, 
            fg_color=CARD_COLOR,
            button_color=BORDER_COLOR,
            button_hover_color=ACCENT_PURPLE,
            font=("Courier New", 11),
            dropdown_font=("Courier New", 11),
            command=self._on_start_stop_change
        )
        self.start_drop.pack(fill="x", padx=6, pady=4)
        self.start_drop.set(waypoint_labels[0])

        # 2. Simulator Launch Card
        header_divider("PROCESS RUNNERS")
        self.sim_card = ctk.CTkFrame(parent, fg_color=CARD_COLOR, border_width=1, border_color=BORDER_COLOR)
        self.sim_card.pack(fill="x", padx=4, pady=4)
        
        self.sim_header = ctk.CTkLabel(self.sim_card, text="🖥️  AUTOWARE SIMULATOR", font=("Courier New", 11, "bold"), text_color=TEXT_WHITE)
        self.sim_header.pack(anchor="w", padx=10, pady=(6, 2))
        
        btn_box1 = ctk.CTkFrame(self.sim_card, fg_color="transparent")
        btn_box1.pack(fill="x", padx=10, pady=4)
        self.sim_play = ctk.CTkButton(btn_box1, text="▶ LAUNCH", width=90, height=28, fg_color=ACCENT_PURPLE, hover_color="#9d00db",
                                      font=("Courier New", 10, "bold"), command=self.sim_manager.start)
        self.sim_play.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.sim_stop = ctk.CTkButton(btn_box1, text="■ TERMINATE", width=90, height=28, fg_color="#475569", hover_color=COLOR_RED,
                                      font=("Courier New", 10, "bold"), command=self.sim_manager.stop)
        self.sim_stop.pack(side="left", fill="x", expand=True)

        self.sim_led = ctk.CTkLabel(self.sim_card, text="● OFFLINE", font=("Courier New", 9, "bold"), text_color=COLOR_RED)
        self.sim_led.pack(anchor="w", padx=12, pady=(0, 6))

        # 3. Shuttle Node Card
        self.node_card = ctk.CTkFrame(parent, fg_color=CARD_COLOR, border_width=1, border_color=BORDER_COLOR)
        self.node_card.pack(fill="x", padx=4, pady=4)
        
        ctk.CTkLabel(self.node_card, text="🚌  SHUTTLE MISSION NODE", font=("Courier New", 11, "bold"), text_color=TEXT_WHITE).pack(anchor="w", padx=10, pady=(6, 2))
        
        btn_box2 = ctk.CTkFrame(self.node_card, fg_color="transparent")
        btn_box2.pack(fill="x", padx=10, pady=4)
        self.shuttle_play = ctk.CTkButton(btn_box2, text="▶ LAUNCH", width=90, height=28, fg_color=ACCENT_PURPLE, hover_color="#9d00db",
                                          font=("Courier New", 10, "bold"), command=self._start_shuttle_node)
        self.shuttle_play.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.shuttle_stop = ctk.CTkButton(btn_box2, text="■ TERMINATE", width=90, height=28, fg_color="#475569", hover_color=COLOR_RED,
                                          font=("Courier New", 10, "bold"), command=self.node_manager.stop)
        self.shuttle_stop.pack(side="left", fill="x", expand=True)

        self.shuttle_led = ctk.CTkLabel(self.node_card, text="● OFFLINE", font=("Courier New", 9, "bold"), text_color=COLOR_RED)
        self.shuttle_led.pack(anchor="w", padx=12, pady=(0, 6))

        # 4. Mission State Card
        header_divider("MISSION MONITORING")
        state_f = ctk.CTkFrame(parent, fg_color=CARD_COLOR, border_width=1, border_color=BORDER_COLOR)
        state_f.pack(fill="x", padx=4, pady=4, ipady=4)
        self.state_lbl = ctk.CTkLabel(state_f, text="SYSTEM READY", font=("Courier New", 14, "bold"), text_color=TEXT_MUTED)
        self.state_lbl.pack(pady=(4, 0))
        self.state_desc = ctk.CTkLabel(state_f, text="Dashboard idle. Select route & launch.", font=("Courier New", 9), text_color=TEXT_MUTED)
        self.state_desc.pack()

        # 5. Elapsed Time Card
        elapsed_f = ctk.CTkFrame(parent, fg_color=CARD_COLOR, border_width=1, border_color=BORDER_COLOR)
        elapsed_f.pack(fill="x", padx=4, pady=4, ipady=4)
        ctk.CTkLabel(elapsed_f, text="MISSION ELAPSED TIME", font=("Courier New", 9, "bold"), text_color=TEXT_MUTED).pack()
        self.elapsed_lbl = ctk.CTkLabel(elapsed_f, text="00:00:00", font=("Courier New", 20, "bold"), text_color=ACCENT_CYAN)
        self.elapsed_lbl.pack()

        # Emergency Button
        self.emergency_btn = ctk.CTkButton(
            parent,
            text="EMERGENCY ALL-STOP",
            fg_color=COLOR_RED,
            hover_color="#b3003c",
            font=("Courier New", 12, "bold"),
            command=self._trigger_emergency_stop
        )
        self.emergency_btn.pack(fill="x", padx=6, pady=(16, 10))

    # ── Right Panel Elements ──────────────────────────────────────────────────
    def _build_right_telemetry(self, parent):
        def add_header(title):
            lbl = ctk.CTkLabel(parent, text=title, font=("Courier New", 10, "bold"), text_color=TEXT_MUTED)
            lbl.pack(anchor="w", padx=10, pady=(10, 2))

        # 1. Speed display
        add_header("VEHICLE SPEED")
        speed_box = ctk.CTkFrame(parent, fg_color=CARD_COLOR, border_width=1, border_color=BORDER_COLOR)
        speed_box.pack(fill="x", padx=6, pady=2, ipady=8)
        self.lbl_speed_kmh = ctk.CTkLabel(speed_box, text="0.00 km/h", font=("Courier New", 28, "bold"), text_color=COLOR_GREEN)
        self.lbl_speed_kmh.pack()
        self.lbl_speed_val = ctk.CTkLabel(speed_box, text="0.00 m/s", font=("Courier New", 11, "bold"), text_color=TEXT_MUTED)
        self.lbl_speed_val.pack()

        # 2. Key statistics grid
        add_header("LIVE MISSION DATA")
        stats_box = ctk.CTkFrame(parent, fg_color=CARD_COLOR, border_width=1, border_color=BORDER_COLOR)
        stats_box.pack(fill="x", padx=6, pady=2, ipady=4)

        def add_stat_line(label, init):
            line = ctk.CTkFrame(stats_box, fg_color="transparent")
            line.pack(fill="x", padx=8, pady=3)
            ctk.CTkLabel(line, text=label, font=("Courier New", 10, "bold"), text_color=TEXT_MUTED).pack(side="left")
            val = ctk.CTkLabel(line, text=init, font=("Courier New", 11, "bold"), text_color=TEXT_WHITE)
            val.pack(side="right")
            return val

        self.tele_dist  = add_stat_line("🎯 DIST. TO GOAL", "—")
        self.tele_curr  = add_stat_line("🏢 CURRENT STOP",  "—")
        self.tele_next  = add_stat_line("➡️ NEXT STOP",     "—")
        self.tele_route = add_stat_line("📊 ROUTE STATE",   "—")
        self.tele_dwell = add_stat_line("⏱️ DWELL REMAIN",  "—")
        self.tele_opmode = add_stat_line("🤖 CONTROL MODE", "—")

        # 3. Active Gear indicator
        add_header("TRANSMISSION ACTUATOR")
        gear_f = ctk.CTkFrame(parent, fg_color="transparent")
        gear_f.pack(fill="x", padx=6, pady=4)
        
        self.gear_lbls = {}
        for g_char in ["P", "R", "N", "D"]:
            lbl = ctk.CTkLabel(gear_f, text=g_char, font=("Courier New", 18, "bold"), text_color=TEXT_MUTED,
                               fg_color=CARD_COLOR, width=42, height=36, corner_radius=4)
            lbl.pack(side="left", padx=4, fill="x", expand=True)
            self.gear_lbls[g_char] = lbl
        self._update_gear_indicator("P")

        # 4. Route Checklist
        add_header("MISSION TIMELINE")
        self.checklist_scroll = ctk.CTkScrollableFrame(parent, fg_color=CARD_COLOR, border_width=1, border_color=BORDER_COLOR, height=310)
        self.checklist_scroll.pack(fill="both", expand=True, padx=6, pady=(2, 8))
        
        self.checklist_widgets = []
        for i, w in enumerate(WAYPOINTS):
            line = ctk.CTkFrame(self.checklist_scroll, fg_color="transparent")
            line.pack(fill="x", pady=2)
            
            lbl_dot = ctk.CTkLabel(line, text="○", font=("Courier New", 12, "bold"), text_color=TEXT_MUTED, width=16)
            lbl_dot.pack(side="left", padx=(4, 0))
            
            lbl_name = ctk.CTkLabel(line, text=w["short"], font=("Courier New", 10), text_color=TEXT_MUTED)
            lbl_name.pack(side="left", padx=8)
            
            lbl_status = ctk.CTkLabel(line, text="", font=("Courier New", 8, "bold"), text_color=TEXT_MUTED)
            lbl_status.pack(side="right", padx=6)
            
            self.checklist_widgets.append((lbl_dot, lbl_name, lbl_status))
        self._update_route_timeline_graphics()

    # ══════════════════════════════════════════════════════════════════════════
    #  PROCESS MONITOR CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════
    def _on_sim_start(self):
        self._log_terminal("▶ Spawning Autoware Planning Simulator...", "SYS")
        self.after(0, lambda: self.sim_led.configure(text="● ONLINE", text_color=COLOR_GREEN))

    def _on_sim_stop(self):
        self._log_terminal("■ Autoware Planning Simulator terminated. Performing clean-up...", "WARN")
        self.after(0, lambda: self.sim_led.configure(text="● OFFLINE", text_color=COLOR_RED))
        threading.Thread(target=self._deep_cleanup, daemon=True).start()

    def _deep_cleanup(self):
        try:
            subprocess.run(["pkill", "-f", "rviz2"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "autoware"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "planning_simulator"], stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def _start_shuttle_node(self):
        selected = self.start_drop.get()
        m = re.match(r'Stop (\d+):', selected)
        if m:
            start_idx = int(m.group(1))
            self._log_terminal(f"🔧 Overwriting shuttle.py START_WAYPOINT_INDEX = {start_idx}...", "SYS")
            
            def edit_task():
                success = self._edit_shuttle_start_parameter(start_idx)
                if success:
                    self._log_terminal("✅ shuttle.py edited successfully. Launching node.", "SYS")
                else:
                    self._log_terminal("⚠️ Edit failed (not critical). Launching node.", "WARN")
                self.after(0, self.node_manager.start)
            
            threading.Thread(target=edit_task, daemon=True).start()
        else:
            self.node_manager.start()

    def _edit_shuttle_start_parameter(self, idx):
        shuttle_py = f"{WS_PATH}/src/campus_pkg/campus_pkg/shuttle.py"
        if not os.path.exists(shuttle_py):
            return False
        try:
            with open(shuttle_py, 'r') as f:
                content = f.read()
            new_content = re.sub(
                r'(#\s*FLEXIBLE START INDEX\s*#\s*\n*)START_WAYPOINT_INDEX\s*=\s*\d+',
                rf'\g<1>START_WAYPOINT_INDEX = {idx}',
                content
            )
            if new_content == content:
                new_content = re.sub(
                    r'START_WAYPOINT_INDEX\s*=\s*\d+',
                    f'START_WAYPOINT_INDEX = {idx}',
                    content
                )
            with open(shuttle_py, 'w') as f:
                f.write(new_content)
            return True
        except Exception as e:
            print("Failed to overwrite shuttle start index dynamically:", e)
            return False

    def _on_shuttle_start(self):
        self._log_terminal("▶ Spawning Campus Autonomous Shuttle Mission Node...", "SYS")
        self.running = True
        self.elapsed_s = 0
        
        # Engage Auto-Follower Mode instantly upon starting navigation
        self.follow_vehicle = True
        self._btn_follow.configure(text_color=BG_COLOR, fg_color=COLOR_GREEN)

        self.after(0, lambda: self.shuttle_led.configure(text="● ONLINE", text_color=COLOR_GREEN))

    def _on_shuttle_stop(self):
        self._log_terminal("■ Shuttle Mission Node terminated.", "WARN")
        self.running = False
        self.follow_vehicle = False
        self._btn_follow.configure(text_color=TEXT_MUTED, fg_color=CARD_COLOR)
        
        # Reset camera view to general campus layout
        self.zoom_level = 1.0
        self.center_x = MIN_X + SPAN_X / 2
        self.center_y = MIN_Y + SPAN_Y / 2
        self.static_drawn = False

        self.after(0, lambda: self.shuttle_led.configure(text="● OFFLINE", text_color=COLOR_RED))
        self.after(0, lambda: self._set_mission_state("IDLE"))

    # ══════════════════════════════════════════════════════════════════════════
    #  MODE SELECT & ROS2 INTERFACE
    # ══════════════════════════════════════════════════════════════════════════
    def _switch_operating_mode(self, mode):
        self._log_terminal(f"🔄 Switching Dashboard Mode to: {mode}", "SYS")
        if mode == "VEHICLE CONNECT MODE":
            self.is_vehicle_connect = True
            self.sim_header.configure(text="🎮  VEHICLE CONTROLLERS")
            self.sim_play.configure(state="normal", fg_color=ACCENT_PURPLE, text="▶ LAUNCH")
            self.sim_stop.configure(state="normal")
            
            # Swap simulator manager to the physical vehicle command
            self.sim_manager.command = VEHICLE_LAUNCH
            
            success = self._initialize_bridge()
            if success:
                self._log_terminal("📡 Connected to vehicle network: ROS2 subscriptions established.", "SYS")
                self.use_ros_tf = True
            else:
                self._log_terminal("⚠️ ROS2 bridge offline. Sourced ROS environment required.", "WARN")
        else:
            self.is_vehicle_connect = False
            self.sim_header.configure(text="🖥️  AUTOWARE SIMULATOR")
            self.sim_play.configure(state="normal", fg_color=ACCENT_PURPLE, text="▶ LAUNCH")
            self.sim_stop.configure(state="normal")
            
            # Swap back to planning simulator command
            self.sim_manager.command = SIM_LAUNCH
            
            self.use_ros_tf = False

    def _initialize_bridge(self):
        if not ROS2_AVAILABLE:
            return False
        if not self.ros_bridge:
            self.ros_bridge = ROS2Bridge(self._ros_bridge_callback)
        success = self.ros_bridge.start()
        if success:
            # Enable ROS positioning automatically in both modes when ROS is active
            self.use_ros_tf = True
        return success

    def _ros_bridge_callback(self, data_type, value):
        if data_type == 'speed':
            self.spd = value
            self.after(0, self._refresh_telemetry_widgets)
        elif data_type == 'route_state':
            self.rt_state = value
            self.after(0, self._refresh_telemetry_widgets)
        elif data_type == 'pose':
            x, y = value
            self.vehicle_x = x
            self.vehicle_y = y
            self.use_ros_tf = True
            
            # Dynamically calculate the closest waypoint index as self.seg_from
            closest_idx = 0
            min_d = float('inf')
            for i, wp in enumerate(WAYPOINTS):
                d = math.hypot(x - wp["x"], y - wp["y"])
                if d < min_d:
                    min_d = d
                    closest_idx = i
            self.seg_from = closest_idx
        elif data_type == 'heading':
            self.vehicle_yaw = value
        elif data_type == 'operation_mode':
            self.op_mode = value
            self.after(0, self._refresh_telemetry_widgets)

    def _on_start_stop_change(self, val):
        m = re.match(r'Stop (\d+):', val)
        if m:
            start_idx = int(m.group(1))
            self.seg_from = start_idx
            self.seg_to = start_idx
            self.vehicle_x = WAYPOINTS[start_idx]["x"]
            self.vehicle_y = WAYPOINTS[start_idx]["y"]
            self.vehicle_yaw = 0.0
            
            # Snap camera to new start stop
            self.center_x = self.vehicle_x
            self.center_y = self.vehicle_y
            
            self.curr_stop = WAYPOINTS[start_idx]["short"]
            self._refresh_telemetry_widgets()
            self._update_route_timeline_graphics()
            self.static_drawn = False

    def _trigger_emergency_stop(self):
        self._log_terminal("🚨 EMERGENCY ALL-STOP TRIGGERED! TERMINATING ALL PROCESSES...", "ERROR")
        self.sim_manager.stop()
        self.node_manager.stop()
        self.running = False
        
        if self.ros_bridge and self.ros_bridge.active:
            try:
                from autoware_vehicle_msgs.msg import GearCommand
                pub = self.ros_bridge.node.create_publisher(GearCommand, '/control/command/gear_cmd', 10)
                msg = GearCommand()
                msg.stamp = self.ros_bridge.node.get_clock().now().to_msg()
                msg.command = 22 # PARK
                pub.publish(msg)
                self.ros_bridge.node.destroy_publisher(pub)
                self._log_terminal("🚨 Park GearCommand sent to vehicle actuator.", "ERROR")
            except Exception:
                pass
        self._update_gear_indicator("P")

    # ══════════════════════════════════════════════════════════════════════════
    #  LOG CONSOLE PARSER & HIGHLIGHTS
    # ══════════════════════════════════════════════════════════════════════════
    def _log_terminal(self, msg, style="INFO"):
        ts = datetime.now().strftime("%H:%M:%S")
        self.terminal_log.configure(state="normal")
        self.terminal_log.insert("end", f"[{ts}] ", "INFO")
        self.terminal_log.insert("end", msg + "\n", style)
        self.terminal_log.see("end")
        self.terminal_log.configure(state="disabled")

    def _process_log_queue(self):
        try:
            for _ in range(40):
                tag, raw_line = self.log_q.get_nowait()
                
                clean_msg = raw_line.strip()
                if "[autoware_shuttle_mission]:" in clean_msg:
                    clean_msg = clean_msg.split("[autoware_shuttle_mission]:", 1)[1].strip()
                clean_msg = re.sub(r'\s*\(.*?\s+at\s+.*?\.py:\d+\)\s*$', '', clean_msg).strip()

                if clean_msg:
                    style = "INFO"
                    if any(x in raw_line for x in ["✅", "🎉", "COMPLETE", "activated", "enabled"]):
                        style = "ROUTE"
                    elif any(x in raw_line for x in ["❌", "ERROR", "FAIL", "failed"]):
                        style = "ERROR"
                    elif any(x in raw_line for x in ["⚠️", "WARN", "incompatible"]):
                        style = "WARN"
                    elif "🔄" in raw_line:
                        style = "SYS"
                    elif any(x in raw_line for x in ["🚌", "En-route", "Goal", "Next", "Arrived", "Dwell"]):
                        style = "ROUTE"
                    
                    if tag == "DEMO":
                        style = "DEMO"
                        
                    self._log_terminal(f"[{tag}] {clean_msg[:120]}", style)

                if tag in ["SHUTTLE", "DEMO"]:
                    self._parse_telemetry_line(raw_line)

        except queue.Empty:
            pass

    def _parse_telemetry_line(self, raw):
        line = raw.strip()
        if "[autoware_shuttle_mission]:" in line:
            line = line.split("[autoware_shuttle_mission]:", 1)[1].strip()
        line = re.sub(r'\s*\(.*?\s+at\s+.*?\.py:\d+\)\s*$', '', line).strip()

        m = re.search(r'🔄\s*(\w+)\s*→\s*(\w+)', line)
        if m:
            self._set_mission_state(m.group(2))
            return

        m = re.search(r'En-route\s*→\s*(.+?)\s+route=(\S+)\s+dist=([\d.]+)m\s+spd=([\d.]+)m/s', line)
        if m:
            target, route, dist, spd = m.groups()
            self.nxt_stop = target.strip()
            self.rt_state = route
            self.dist = float(dist)
            if not self.use_ros_tf:
                self.spd = float(spd)
            self._refresh_telemetry_widgets()
            
            t_idx = STOP_INDEX.get(self.nxt_stop.lower())
            if t_idx is not None:
                self.seg_to = t_idx
                if self.seg_to > 0:
                    self.seg_from = self.seg_to - 1
            return

        m = re.search(r'Dwell\s+([\d.-]+)s\s+left\s+\[(.+?)\]\s+spd=([\d.]+)m/s', line)
        if m:
            dwell_s, stop_name, spd = m.groups()
            self.dwell_left = float(dwell_s)
            self.curr_stop = stop_name.strip()
            if not self.use_ros_tf:
                self.spd = float(spd)
            self._refresh_telemetry_widgets()
            return

        m = re.search(r'Arrived\s+\[.+?\]\s+dist=[\d.]+m\s*→\s*(.+)', line)
        if m:
            stop_name = m.group(1).strip()
            self.curr_stop = stop_name
            idx = STOP_INDEX.get(stop_name.lower())
            if idx is not None:
                self.seg_from = idx
                self.seg_to = idx
            self._refresh_telemetry_widgets()
            return

        m = re.search(r'Goal\s+\[(\d+)/\d+\]\s*→\s*(.+?)\s*\(', line)
        if m:
            idx, stop_name = m.groups()
            self.nxt_stop = stop_name.strip()
            self.seg_to = int(idx)
            self._t = 0.0
            self._refresh_telemetry_widgets()
            return

        m = re.search(r'Next\s+\[(\d+)/\d+\]:\s*(.+?)\s*\(', line)
        if m:
            idx, stop_name = m.groups()
            self.nxt_stop = stop_name.strip()
            self.seg_to = int(idx)
            self._t = 0.0
            self._refresh_telemetry_widgets()
            return

        m = re.search(r'Route State\s*→\s*(\w+)', line)
        if m:
            self.rt_state = m.group(1)
            self._refresh_telemetry_widgets()
            return

        if "SHUTTLE MISSION COMPLETE" in line:
            self._set_mission_state("COMPLETE")
            self.seg_from = len(WAYPOINTS) - 1
            self.seg_to = len(WAYPOINTS) - 1
            self._refresh_telemetry_widgets()

    def _set_mission_state(self, state_key):
        self.mission_st = state_key
        lbl, col = STATE_MAP.get(state_key, (state_key, TEXT_WHITE))
        self.state_lbl.configure(text=lbl, text_color=col)
        
        subs = {
            "MONITORING_PROGRESS": "Vehicle active. Autonomous driving.",
            "JUNCTION_STOP":       "Dwell active. Holding electric brakes.",
            "COMPLETE":            "Mission complete. All stops visited.",
            "WAIT_BEFORE_GOAL":    "Warmup active. Aligning IMU & pose.",
            "SWITCH_AUTONOMOUS":   "Transitioning control mode to AUTO.",
            "RESUME_AFTER_STOP":   "Releasing brake system. Shifting DRIVE.",
            "SET_GOAL_POSE":       "Routing new path. Generating vector mesh."
        }
        self.state_desc.configure(text=subs.get(state_key, "Running mission scripts."))
        
        if state_key in ["MONITORING_PROGRESS", "RESUME_AFTER_STOP", "SWITCH_AUTONOMOUS"]:
            self._update_gear_indicator("D")
        elif state_key in ["JUNCTION_STOP", "COMPLETE", "IDLE"]:
            self._update_gear_indicator("P")

    # ══════════════════════════════════════════════════════════════════════════
    #  UI REFRESH & TELEMETRY UPDATES
    # ══════════════════════════════════════════════════════════════════════════
    def _refresh_telemetry_widgets(self):
        spd_col = COLOR_GREEN if self.spd > 0.1 else TEXT_MUTED
        self.lbl_speed_val.configure(text=f"{self.spd:.2f} m/s")
        
        kmh = self.spd * 3.6
        self.lbl_speed_kmh.configure(text=f"{kmh:.2f} km/h", text_color=spd_col)

        self.tele_dist.configure(text=f"{self.dist:.1f} m" if self.dist > 0.05 else "—")
        self.tele_curr.configure(text=self.curr_stop[:18])
        self.tele_next.configure(text=self.nxt_stop[:18])
        self.tele_route.configure(text=self.rt_state)
        
        if self.dwell_left > 0.05:
            self.tele_dwell.configure(text=f"{self.dwell_left:.0f} s", text_color=COLOR_RED)
        else:
            self.tele_dwell.configure(text="—", text_color=TEXT_WHITE)

        mode_col = TEXT_WHITE
        if self.op_mode == "AUTONOMOUS":
            mode_col = COLOR_GREEN
        elif self.op_mode in ["MANUAL", "LOCAL"]:
            mode_col = COLOR_AMBER
        self.tele_opmode.configure(text=self.op_mode, text_color=mode_col)

        self._update_route_timeline_graphics()

    def _update_gear_indicator(self, active_g):
        for gear_char, lbl in self.gear_lbls.items():
            if gear_char == active_g:
                lbl.configure(text_color=BG_COLOR, fg_color=ACCENT_CYAN)
            else:
                lbl.configure(text_color=TEXT_MUTED, fg_color=CARD_COLOR)

    def _update_route_timeline_graphics(self):
        for i, (lbl_dot, lbl_name, lbl_status) in enumerate(self.checklist_widgets):
            if i < self.seg_from:
                lbl_dot.configure(text="✓", text_color=COLOR_GREEN)
                lbl_name.configure(text_color=COLOR_GREEN)
                lbl_status.configure(text="VISITED", text_color=COLOR_GREEN)
            elif i == self.seg_from and self.mission_st == "JUNCTION_STOP":
                lbl_dot.configure(text="⏸", text_color=COLOR_RED)
                lbl_name.configure(text_color=COLOR_RED)
                lbl_status.configure(text="DWELLING", text_color=COLOR_RED)
            elif i == self.seg_to and self.mission_st == "MONITORING_PROGRESS":
                lbl_dot.configure(text="●", text_color=ACCENT_CYAN)
                lbl_name.configure(text_color=ACCENT_CYAN)
                lbl_status.configure(text="TARGET", text_color=ACCENT_CYAN)
            else:
                lbl_dot.configure(text="○", text_color=TEXT_MUTED)
                lbl_name.configure(text_color=TEXT_MUTED)
                lbl_status.configure(text="", text_color=TEXT_MUTED)

    # ══════════════════════════════════════════════════════════════════════════
    #  SCIENTIFIC SCALING MAP & HIGH-FIDELITY VECTOR RENDER
    # ══════════════════════════════════════════════════════════════════════════
    def _draw_static_map(self):
        """Draw static environment elements. Only drawn once on startup/resize to maximize FPS."""
        c = self.map_canvas
        c.delete("all")
        
        W = c.winfo_width()
        H = c.winfo_height()
        if W < 30 or H < 30:
            return

        # 1. Background Grid Lines
        for x in range(0, W, 40):
            c.create_line(x, 0, x, H, fill="#0b132b", width=1)
        for y in range(0, H, 40):
            c.create_line(0, y, W, y, fill="#0b132b", width=1)

        c.create_rectangle(15, 15, W-15, H-15, outline="#1e293b", width=1, dash=(5, 5))

        pad = 50
        scale_x = (W - 2 * pad) / SPAN_X
        scale_y = (H - 2 * pad) / SPAN_Y
        base_scale = min(scale_x, scale_y)

        # Coordinate translation with dynamic zoom & panning centers
        def translate_utm_to_canvas(ux, uy):
            dx = (ux - self.center_x) * base_scale * self.zoom_level
            dy = (uy - self.center_y) * base_scale * self.zoom_level
            px = W / 2 + dx
            py = H / 2 - dy
            return int(px), int(py)

        # 2. Draw Pointcloud Map Background (.pcd radar scan density layer) via PIL image buffer
        # This completely resolves Tkinter canvas object limits and prevents X11 / graphics driver segmentation faults!
        from PIL import ImageDraw
        pil_img = Image.new("RGBA", (W, H), (5, 8, 17, 255))
        draw = ImageDraw.Draw(pil_img)

        for (ux, uy) in self.pcd_points:
            px, py = translate_utm_to_canvas(ux, uy)
            if 0 <= px < W and 0 <= py < H:
                draw.rectangle([px, py, px+1, py+1], fill=PCD_POINT_COL)

        self.bg_image_tk = ImageTk.PhotoImage(pil_img)
        c.create_image(0, 0, image=self.bg_image_tk, anchor="nw")

        # 3. Draw Lanelet2 OSM Campus Road Networks (High-Fidelity Vector skeleton)
        for way_pts in self.osm_ways:
            canvas_pts = []
            for (ux, uy) in way_pts:
                px, py = translate_utm_to_canvas(ux, uy)
                canvas_pts.append((px, py))
            if len(canvas_pts) >= 2:
                c.create_line([coord for pt in canvas_pts for coord in pt], fill=OSM_ROAD_COLOR, width=1.5, capstyle="round", joinstyle="round")

        # 4. Highlight stoppages directly on UTM locations
        pts = [translate_utm_to_canvas(w["x"], w["y"]) for w in WAYPOINTS]

        # 5. Draw Waypoints stopping ring nodes
        for i, (w, pt) in enumerate(zip(WAYPOINTS, pts)):
            x, y = pt
            completed = i < self.seg_from
            
            # Clip stop nodes rendering inside viewport boundary
            if not (0 <= x <= W and 0 <= y <= H):
                continue

            if w["is_stop"]:
                r = 9
                octagon_pts = [
                    (x - r, y - r/2), (x - r/2, y - r),
                    (x + r/2, y - r), (x + r, y - r/2),
                    (x + r, y + r/2), (x + r/2, y + r),
                    (x - r/2, y + r), (x - r, y + r/2)
                ]
                if completed:
                    c.create_polygon([coord for pt in octagon_pts for coord in pt], fill=COLOR_GREEN, outline="#ffffff", width=1)
                else:
                    c.create_polygon([coord for pt in octagon_pts for coord in pt], fill=COLOR_RED, outline="#ffffff", width=1)
                c.create_text(x, y, text="S", font=("Courier New", 8, "bold"), fill="#ffffff")
            else:
                ring_col = COLOR_GREEN if completed else BORDER_COLOR
                fill_col = COLOR_GREEN if completed else "#0b1528"
                c.create_oval(x-8-2, y-8-2, x+8+2, y+8+2, fill="#050811", outline=ring_col, width=2)
                c.create_oval(x-8, y-8, x+8, y+8, fill=fill_col, outline="")

            label_y = y - 22 if y > H * 0.6 else y + 22
            anchor_dir = "s" if y > H * 0.6 else "n"
            text_color = TEXT_WHITE if completed else TEXT_MUTED
            c.create_text(x, label_y, text=w["short"], font=("Courier New", 9, "bold"), fill=text_color, anchor=anchor_dir)

        # 6. Static Compass
        cx, cy = W - 40, H - 40
        c.create_oval(cx-20, cy-20, cx+20, cy+20, fill=PANEL_COLOR, outline=BORDER_COLOR, width=2)
        c.create_text(cx, cy, text="N", font=("Courier New", 9, "bold"), fill=TEXT_WHITE)
        c.create_line(cx, cy-18, cx, cy-8, fill=COLOR_RED, width=2)

        # 7. Map Legend
        c.create_rectangle(15, 15, 195, 138, fill="#0b1329", outline=BORDER_COLOR, width=1)
        c.create_text(25, 25, text="MAP LEGEND", font=("Courier New", 9, "bold"), fill=ACCENT_CYAN, anchor="w")
        
        r_stop = 5
        x_s, y_s = 25, 45
        oct_p = [
            (x_s - r_stop, y_s - r_stop/2), (x_s - r_stop/2, y_s - r_stop),
            (x_s + r_stop/2, y_s - r_stop), (x_s + r_stop, y_s - r_stop/2),
            (x_s + r_stop, y_s + r_stop/2), (x_s + r_stop/2, y_s + r_stop),
            (x_s - r_stop/2, y_s + r_stop), (x_s - r_stop, y_s + r_stop/2)
        ]
        c.create_polygon([coord for pt in oct_p for coord in pt], fill=COLOR_RED, outline="#ffffff", width=1)
        c.create_text(38, 45, text="Bus Stop (Octagon)", font=("Courier New", 8), fill=TEXT_MUTED, anchor="w")

        c.create_oval(25-4, 65-4, 25+4, 65+4, fill=ACCENT_CYAN, outline="")
        c.create_text(38, 65, text="Target stop", font=("Courier New", 8), fill=TEXT_MUTED, anchor="w")

        c.create_oval(25-4, 85-4, 25+4, 85+4, fill=COLOR_GREEN, outline="")
        c.create_text(38, 85, text="Visited stop", font=("Courier New", 8), fill=TEXT_MUTED, anchor="w")

        c.create_line(20, 105, 30, 105, fill=OSM_ROAD_COLOR, width=2.5)
        c.create_text(38, 105, text="OSM Roadnet (Lanelet2)", font=("Courier New", 8), fill=TEXT_MUTED, anchor="w")

        c.create_oval(23, 122, 27, 126, fill=PCD_POINT_COL, outline="")
        c.create_text(38, 123, text="PCD Pointcloud map", font=("Courier New", 8), fill=TEXT_MUTED, anchor="w")

        self.static_drawn = True

    # ══════════════════════════════════════════════════════════════════════════
    #  DYNAMIC REDRAW CYCLE (ONLY CLEAR DYNAMIC ASSETS FOR 60 FPS PERFORMANCE)
    # ══════════════════════════════════════════════════════════════════════════
    def _redraw_dynamic_elements(self):
        c = self.map_canvas
        c.delete("dynamic")  # Free old dynamic elements instantly
        
        W = c.winfo_width()
        H = c.winfo_height()
        if W < 30 or H < 30:
            return

        pad = 50
        scale_x = (W - 2 * pad) / SPAN_X
        scale_y = (H - 2 * pad) / SPAN_Y
        base_scale = min(scale_x, scale_y)

        # Dynamic coordinate translation using active center and zoom
        def translate_utm_to_canvas(ux, uy):
            dx = (ux - self.center_x) * base_scale * self.zoom_level
            dy = (uy - self.center_y) * base_scale * self.zoom_level
            px = W / 2 + dx
            py = H / 2 - dy
            return int(px), int(py)

        pts = [translate_utm_to_canvas(w["x"], w["y"]) for w in WAYPOINTS]

        # 1. Glowing target stop ring pulse
        if self.running and self.seg_to < len(pts):
            tx, ty = pts[self.seg_to]
            r_pulse = 11 + 3 + int(self._pulse * 15)
            c.create_oval(tx-r_pulse, ty-r_pulse, tx+r_pulse, ty+r_pulse, outline=ACCENT_CYAN, width=2, tags="dynamic")

        # 2. Dynamic Vehicle Position & Yaw Orientation
        if (self.running or self.use_ros_tf) and len(pts) > 1:
            if self.use_ros_tf:
                sx, sy = translate_utm_to_canvas(self.vehicle_x, self.vehicle_y)
                heading_rad = -self.vehicle_yaw
            else:
                fi = max(0, min(self.seg_from, len(pts)-2))
                ti = min(self.seg_to, len(pts)-1)
                if fi == ti and ti < len(pts) - 1:
                    ti += 1
                
                x1, y1 = pts[fi]
                x2, y2 = pts[ti]
                t_factor = max(0.0, min(1.0, self._t))
                
                # Smooth interpolation in canvas meters
                utm_w1 = WAYPOINTS[fi]
                utm_w2 = WAYPOINTS[ti]
                ux = utm_w1["x"] + (utm_w2["x"] - utm_w1["x"]) * t_factor
                uy = utm_w1["y"] + (utm_w2["y"] - utm_w1["y"]) * t_factor
                
                self.vehicle_x = ux
                self.vehicle_y = uy
                
                sx, sy = translate_utm_to_canvas(ux, uy)
                self.vehicle_yaw = math.atan2(utm_w2["y"] - utm_w1["y"], utm_w2["x"] - utm_w1["x"])
                heading_rad = -self.vehicle_yaw

            ca, sa = math.cos(heading_rad), math.sin(heading_rad)
            def rotate_pts(px, py):
                return sx + px * ca - py * sa, sy + px * sa + py * ca

            # Emitter Particle Trail
            if self.spd > 0.05:
                self._sparkles.append((sx, sy, time.time()))
            self._sparkles = [(px, py, t) for px, py, t in self._sparkles if time.time() - t < 0.4]
            for px, py, st in self._sparkles:
                age = time.time() - st
                rad_sz = max(1, int((1.0 - age / 0.4) * 4))
                c.create_oval(px-rad_sz, py-rad_sz, px+rad_sz, py+rad_sz, fill=ACCENT_PURPLE, outline="", tags="dynamic")

            # Soft vehicle glow halo ring
            c.create_oval(sx-36, sy-36, sx+36, sy+36, fill="", outline="#00f0ff", width=1, tags="dynamic")

            # 1. High-Fidelity 10-Meter Centerline Trajectory Tracing (Always perfectly centered in the middle of the road)
            vx, vy = self.vehicle_x, self.vehicle_y
            trajectory_pts = [(vx, vy)]
            
            # Find the closest boundary way (Way_A)
            closest_way = None
            closest_node_idx = -1
            min_d = float('inf')
            
            for way in self.osm_ways:
                for i, pt in enumerate(way):
                    d = math.hypot(vx - pt[0], vy - pt[1])
                    if d < min_d:
                        min_d = d
                        closest_way = way
                        closest_node_idx = i
            
            if closest_way is not None and len(closest_way) >= 2:
                # Find the opposite boundary way (Way_B) to compute centerlines
                second_way = None
                min_d2 = float('inf')
                for way in self.osm_ways:
                    if way is closest_way:
                        continue
                    for pt in way:
                        d = math.hypot(vx - pt[0], vy - pt[1])
                        if d < min_d2:
                            min_d2 = d
                            second_way = way
                
                yaw = self.vehicle_yaw
                dir_x = math.cos(yaw)
                dir_y = math.sin(yaw)
                
                # Determine forward direction index progression (dot product test)
                idx = closest_node_idx
                go_forward = True
                if idx < len(closest_way) - 1:
                    nxt = closest_way[idx + 1]
                    dot = (nxt[0] - closest_way[idx][0]) * dir_x + (nxt[1] - closest_way[idx][1]) * dir_y
                    if dot < 0 and idx > 0:
                        go_forward = False
                elif idx > 0:
                    go_forward = False
                    
                # Trace exactly up to 10.0 meters along the computed centerlines
                step = 1 if go_forward else -1
                curr_idx = idx
                accum_d = 0.0
                curr_x, curr_y = vx, vy
                
                while 0 <= curr_idx < len(closest_way) and accum_d < 10.0:
                    next_pt = closest_way[curr_idx]
                    
                    # Compute road centerline midpoint between closest_way and second_way
                    if second_way is not None and len(second_way) >= 2:
                        # Find closest point in second_way to next_pt
                        opp_pt = second_way[0]
                        min_opp_d = float('inf')
                        for p in second_way:
                            od = math.hypot(next_pt[0] - p[0], next_pt[1] - p[1])
                            if od < min_opp_d:
                                min_opp_d = od
                                opp_pt = p
                        # Midpoint representing exact road centerline center!
                        center_pt = ((next_pt[0] + opp_pt[0]) / 2.0, (next_pt[1] + opp_pt[1]) / 2.0)
                    else:
                        center_pt = next_pt
                        
                    seg_len = math.hypot(center_pt[0] - curr_x, center_pt[1] - curr_y)
                    
                    if accum_d + seg_len >= 10.0:
                        ratio = (10.0 - accum_d) / seg_len
                        interp_x = curr_x + (center_pt[0] - curr_x) * ratio
                        interp_y = curr_y + (center_pt[1] - curr_y) * ratio
                        trajectory_pts.append((interp_x, interp_y))
                        break
                    else:
                        if curr_idx != idx:
                            trajectory_pts.append(center_pt)
                        accum_d += seg_len
                        curr_x, curr_y = center_pt[0], center_pt[1]
                        curr_idx += step
                
                # Project the final segment vector if lane length is less than 10 meters
                if accum_d < 10.0 and len(trajectory_pts) >= 2:
                    rem_d = 10.0 - accum_d
                    last_p = trajectory_pts[-1]
                    prev_p = trajectory_pts[-2]
                    dx = last_p[0] - prev_p[0]
                    dy = last_p[1] - prev_p[1]
                    h_len = math.hypot(dx, dy)
                    if h_len > 0.01:
                        proj_x = last_p[0] + (dx / h_len) * rem_d
                        proj_y = last_p[1] + (dy / h_len) * rem_d
                        trajectory_pts.append((proj_x, proj_y))
            
            # Translate local UTM coordinates of the 10m path to canvas coordinates
            canvas_trajectory_pts = [translate_utm_to_canvas(pt[0], pt[1]) for pt in trajectory_pts]
            
            if len(canvas_trajectory_pts) >= 2:
                coords = [coord for pt in canvas_trajectory_pts for coord in pt]
                # Bumper region width matching outer glow halo
                c.create_line(coords, fill="#047857", width=18, capstyle="round", joinstyle="round", tags="dynamic")
                # Bright main trajectory ribbon
                c.create_line(coords, fill="#10b981", width=12, capstyle="round", joinstyle="round", tags="dynamic")
                # Intense neon core highlight line
                c.create_line(coords, fill="#a7f3d0", width=4, capstyle="round", joinstyle="round", tags="dynamic")

            # A. Wheels (sleek dark charcoal rectangles with thin cyan outline)
            for wx, wy in [(-16, -11), (-16, 11), (12, -11), (12, 11)]:
                w_p = [
                    rotate_pts(wx-6, wy-2.5),
                    rotate_pts(wx+6, wy-2.5),
                    rotate_pts(wx+6, wy+2.5),
                    rotate_pts(wx-6, wy+2.5)
                ]
                c.create_polygon([coord for pt in w_p for coord in pt], fill="#1e293b", outline=ACCENT_CYAN, width=1, tags="dynamic")

            # B. Sleek aerodynamic curved body (length 52, width 22)
            body_p = [
                rotate_pts(-26, -11),
                rotate_pts(20, -11),
                rotate_pts(26, -5),
                rotate_pts(26, 5),
                rotate_pts(20, 11),
                rotate_pts(-26, 11),
                rotate_pts(-28, 6),
                rotate_pts(-28, -6)
            ]
            c.create_polygon([coord for pt in body_p for coord in pt], fill="#0f172a", outline=ACCENT_CYAN, width=2.2, tags="dynamic")

            # C. Clean panoramic windshield front glass (sleek Waymo-style highlight)
            windshield = [
                rotate_pts(6, -8),
                rotate_pts(16, -7),
                rotate_pts(18, -3),
                rotate_pts(18, 3),
                rotate_pts(16, 7),
                rotate_pts(6, 8)
            ]
            c.create_polygon([coord for pt in windshield for coord in pt], fill="#38bdf8", outline="", tags="dynamic")

            # D. Rear window glass
            rear_win = [
                rotate_pts(-22, -8),
                rotate_pts(-16, -8),
                rotate_pts(-16, 8),
                rotate_pts(-22, 8)
            ]
            c.create_polygon([coord for pt in rear_win for coord in pt], fill="#334155", outline="", tags="dynamic")

            # E. Dynamic headlights and brake indicators
            for hx, hy in [(25, -7), (25, 7)]:
                hxx, hyy = rotate_pts(hx, hy)
                c.create_oval(hxx-1.5, hyy-1.5, hxx+1.5, hyy+1.5, fill="#fef08a", outline="", tags="dynamic")
            for rx, ry in [(-27, -7), (-27, 7)]:
                rxx, ryy = rotate_pts(rx, ry)
                c.create_oval(rxx-1.5, ryy-1.5, rxx+1.5, ryy+1.5, fill=COLOR_RED, outline="", tags="dynamic")

            # F. Roof-mounted minimalist LIDAR sensor pod
            lidar_center = rotate_pts(-4, 0)
            c.create_oval(
                lidar_center[0]-5, lidar_center[1]-5,
                lidar_center[0]+5, lidar_center[1]+5,
                fill="#1e293b", outline=ACCENT_CYAN, width=1.5, tags="dynamic"
            )
            # Rotating laser sweep dot (very subtle green light)
            lidar_angle = (time.time() * 6) % (2 * math.pi)
            lxx, lyy = rotate_pts(-4 + 3.5 * math.cos(lidar_angle), 3.5 * math.sin(lidar_angle))
            c.create_oval(lxx-1.2, lyy-1.2, lxx+1.2, lyy+1.2, fill=COLOR_GREEN, outline="", tags="dynamic")

    # ══════════════════════════════════════════════════════════════════════════
    #  ANIMATION TIMER INTERFACE LOOP (30 FPS)
    # ══════════════════════════════════════════════════════════════════════════
    def _run_render_loop(self):
        self._pulse = (self._pulse + 0.05) % 1.0

        if self.running and self.seg_from < len(WAYPOINTS) - 1:
            w1 = WAYPOINTS[self.seg_from]
            w2 = WAYPOINTS[self.seg_to]
            total_d = math.hypot(w2["x"] - w1["x"], w2["y"] - w1["y"])
            
            if total_d > 0.1:
                progress = (total_d - self.dist) / total_d
                target_t = max(0.0, min(1.0, progress))
                self._t += (target_t - self._t) * 0.12
            else:
                self._t = 0.0
        else:
            self._t = 0.0

        # Smoothly update precise real-time distance-to-goal (millimeter-level precision)
        if (self.running or self.use_ros_tf) and self.seg_to < len(WAYPOINTS):
            target_wp = WAYPOINTS[self.seg_to]
            self.dist = math.hypot(self.vehicle_x - target_wp["x"], self.vehicle_y - target_wp["y"])
            self._refresh_telemetry_widgets()

        # Smooth Auto-Centering Follower Camera Calculations
        if self.follow_vehicle and (self.running or self.use_ros_tf):
            # Smoothly shift the focal center to track the vehicle's UTM position
            self.center_x += (self.vehicle_x - self.center_x) * 0.08
            self.center_y += (self.vehicle_y - self.center_y) * 0.08
            # Smoothly zoom camera close up to 6.5x zoom
            self.zoom_level += (6.5 - self.zoom_level) * 0.05
            # Force static background layers to slide beneath
            self.static_drawn = False

        # Draw static layer once, then redraw dynamic elements seamlessly
        if not self.static_drawn:
            self._draw_static_map()
            
        self._process_log_queue()
        self._redraw_dynamic_elements()

        self.after(33, self._run_render_loop)

    # ══════════════════════════════════════════════════════════════════════════
    #  SYSTEM HEARTBEAT & TIMERS (1 HZ)
    # ══════════════════════════════════════════════════════════════════════════
    def _tick_system_clock(self):
        self._clock_tick = not self._clock_tick
        colon = ":" if self._clock_tick else " "
        
        # Calculate India Standard Time (IST) offset of +5:30 from UTC
        ist_now = datetime.utcnow() + timedelta(hours=5, minutes=30)
        current_time = ist_now.strftime(f"%H{colon}%M{colon}%S") + " IST"
        self.clock_lbl.configure(text=current_time)

        if self.running:
            self.elapsed_s += 1
            h = self.elapsed_s // 3600
            m = (self.elapsed_s % 3600) // 60
            s = self.elapsed_s % 60
            self.elapsed_lbl.configure(text=f"{h:02d}:{m:02d}:{s:02d}", text_color=ACCENT_CYAN)
            
            if self.dwell_left > 0:
                self.dwell_left = max(0.0, self.dwell_left - 1.0)
                self._refresh_telemetry_widgets()
        else:
            self.elapsed_lbl.configure(text="00:00:00", text_color=TEXT_MUTED)

        self.after(1000, self._tick_system_clock)


if __name__ == "__main__":
    app = MissionControlDashboard()
    
    def on_closing():
        app.sim_manager.stop()
        app.node_manager.stop()
        if app.ros_bridge:
            app.ros_bridge.stop()
        
        try:
            import subprocess
            subprocess.run(["pkill", "-f", "rviz2"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "autoware"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "campus_pkg"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "planning_simulator"], stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "shuttle"], stderr=subprocess.DEVNULL)
        except Exception:
            pass
            
        app.destroy()
        
    app.protocol("WM_DELETE_WINDOW", on_closing)
    app.mainloop()
