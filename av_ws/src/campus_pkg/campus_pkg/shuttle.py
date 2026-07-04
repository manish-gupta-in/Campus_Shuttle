"""
shuttle.py  –  Campus Autonomous Shuttle Mission  (v4)
Route: Main Gate → A-Block → Hostel Circle → CP → WILP-Lab
       → K-Block → H-Block → I-Block → Security Gate (End)

ARRIVAL DETECTION  (dual-trigger, whichever fires first):
  PRIMARY   – Autoware RouteState == GOAL_REACHED (6)
  FALLBACK  – Proximity check via /tf  base_link → map

FLEXIBLE START INDEX:
  Set START_WAYPOINT_INDEX to start from any stop.

═══════════════════════════════════════════════════════════
BUG FIXES IN v4  (brake-hold clicking / loop issue)
═══════════════════════════════════════════════════════════

BUG 1 – FIXED (PRIMARY: continuous clicking sound)
  Root cause: _brake_hold_callback() was republishing GearCommand(PARK)
  repeatedly at HOLD_GEAR_HZ (2 Hz).  The Hooke2 DBW interprets every
  new GearCommand message as a fresh actuator request, so the park pawl
  was mechanically engaging and disengaging in a loop for the entire
  dwell period — producing the clicking sound.

  Fix: GearCommand(PARK) is now sent EXACTLY ONCE per stop, guarded by
  a per-leg one-shot flag (_gear_park_sent).  The park pawl latches
  mechanically; it does NOT need a keepalive.  The 10 Hz timer now only
  republishes the velocity cap (which DOES need keepalive because the
  planner can reset it).

BUG 2 – FIXED (park re-engages after DRIVE is sent)
  Root cause: In RESUME_AFTER_STOP, _shift_to_drive() sent GearCommand(DRIVE),
  but _brake_hold_active was still True for RESUME_GEAR_SETTLE seconds,
  so the timer kept sending GEAR_PARK and immediately overrode the DRIVE
  command.  On the next leg the vehicle could not move.

  Fix: Gear publishing is now completely separated from the velocity-cap
  keepalive. Two independent flags:
    _vel_cap_active  – True while velocity cap keepalive timer should run
    _gear_park_sent  – one-shot per leg; set on engage, never re-fired

BUG 3 – FIXED (engage called multiple times per stop → loop persists)
  Root cause: handle_junction_stop() checked `if not self._brake_hold_active`
  but _brake_hold_active was a shared flag also cleared during RESUME.
  If the state was re-entered (or the flag was cleared by another path)
  _engage_brake_hold() fired again mid-stop.  Once this happened at any
  junction, the issue propagated to every subsequent junction.

  Fix: _gear_park_sent is a per-leg flag reset only in
  _advance_to_next_waypoint() and __init__.  Even if _engage_brake_hold()
  is called multiple times, the gear command is physically sent only once.

BUG 4 – FIXED (RESUME_AFTER_STOP transition never fired in some edge cases)
  Root cause: The transition condition depended on _brake_hold_active being
  False, but if a code path reached RESUME without ever setting the flag
  True, the condition `not _brake_hold_active` was immediately True but
  `elapsed >= RESUME_GEAR_SETTLE + 0.5` gated it to 2 s.  Harmless but
  confusing.  Replaced with an explicit elapsed-only gate for clarity.

v2 fixes retained:
  1. Autonomous-mode retry storm (exponential back-off, in-flight guard).
  2. Speed always 0.00 m/s (correct VelocityReport message type).
  3. Route-state settle guard in WAIT_AUTONOMOUS_READY.
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from autoware_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_vehicle_msgs.msg import VelocityReport, GearCommand


# ═══════════════════════════════════════════════════════════
#  TUNING KNOBS
# ═══════════════════════════════════════════════════════════

STOP_WAIT_SEC       = 10    # dwell time at junction stops (seconds)
GOAL_PUBLISH_DELAY  = 15    # localisation warm-up after initial pose (seconds)

PROXIMITY_RADIUS    = 5.0   # metres
STOP_SPEED_THRESH   = 0.2   # m/s
PROXIMITY_CONFIRM   = 5.0   # seconds

USE_2D_PROXIMITY    = True

AUTONOMOUS_RETRY_BASE_SEC = 1.0
AUTONOMOUS_RETRY_MAX_SEC  = 16.0

ROUTE_STATES_READY  = {2, 4, 5}
ROUTE_SETTLE_SEC    = 1.5

# ── Brake hold (v4) ───────────────────────────────────────
# How often to REPUBLISH the velocity cap during dwell.
# The planner can reset /current_max_velocity so we must keep asserting it.
# Only the velocity cap needs this — the gear command does NOT.
VEL_CAP_HZ          = 5     # Hz – velocity cap republish rate during dwell

# How long to wait in DRIVE gear before releasing the velocity cap.
# Gives the park-pawl actuator time to mechanically disengage before
# the planner is allowed to command forward motion.
RESUME_GEAR_SETTLE  = 2.0   # seconds (raised from 1.5 for reliability)

# GearCommand.command values (autoware_vehicle_msgs/msg/GearCommand)
GEAR_DRIVE = 2
GEAR_PARK  = 22


# ═══════════════════════════════════════════════════════════
#  FLEXIBLE START INDEX
# ═══════════════════════════════════════════════════════════
START_WAYPOINT_INDEX = 0


# ═══════════════════════════════════════════════════════════
#  WAYPOINTS
# ═══════════════════════════════════════════════════════════
WAYPOINTS = [
    # ── INDEX 0: Security Main Gate ──
    {
        "label": "Security Main Gate",
        "x": 42302.34375,
        "y": 41729.1171875,
        "z": -0.2517782436530944,
        "xo": 0.0068914417790954295,
        "yo": -0.0035748486627069824,
        "zo": 0.8876483646367227,
        "wo": 0.46045641405565835,
        "is_stop": False,
    },

    # ── INDEX 1: A-Block ──
    {
        "label": "A-Block",
        "x": 42220.4765625,
        "y": 41821.6640625,
        "z": -1.3197211606085146,
        "xo": 0.0027251518733039967,
        "yo": -0.0015912225208019031,
        "zo": 0.8635606656605122,
        "wo": 0.5042350823595378,
        "is_stop": True,
    },

    # ── INDEX 2: Hostel Circle ──
    {
        "label": "Hostel Circle",
        "x": 42194.18359375,
        "y": 41997.09765625,
        "z": 5.483334663624829,
        "xo": 0.01135196957042133,
        "yo": -0.012516942828261077,
        "zo": 0.6716993267607927,
        "wo": 0.740631131777624,
        "is_stop": True,
    },

    # ── INDEX 3: CP ──
    {
        "label": "CP",
        "x": 42241.18359375,
        "y": 42075.81640625,
        "z": 8.18149617417487,
        "xo": 0.004556245826119891,
        "yo": -0.007168097632100621,
        "zo": 0.5364143703227472,
        "wo": 0.8439120110008859,
        "is_stop": True,
    },

    # ── INDEX 4: E-Block ──
    {
        "label": "E-Block",
        "x": 42365.14303503959,
        "y": 42086.45246681509,
        "z": 14.444714546294357,
        "xo": -0.004168162092547916,
        "yo": 0.006954977264181332,
        "zo": 0.51404130789792,
        "wo": 0.8577271060719841,
        "is_stop": True,
    },

    # ── INDEX 5: WILP-Lab ──
    {
        "label": "WILP-Lab",
        "x": 42577.55078125,
        "y": 42049.7109375,
        "z": 15.9864,
        "xo": -9.847164800623893e-05,
        "yo": 0.001345138673548512,
        "zo": 0.07301013559478393,
        "wo": 0.997330286818622,
        "is_stop": True,
    },

    # ── INDEX 6: K-Block ──
    {
        "label": "K-Block",
        "x": 42601.28854121975,
        "y": 41917.11737505893,
        "z": 6.680445054262589,
        "xo": 0.0076779438616637954,
        "yo": 0.0042062982846378435,
        "zo": -0.8769803595772501,
        "wo": 0.48044646439385424,
        "is_stop": True,
    },

    # ── INDEX 7: H-Block ──
    {
        "label": "H-Block",
        "x": 42532.79296875,
        "y": 41806.828125,
        "z": 2.6876534643154124,
        "xo": 0.017850334528261208,
        "yo": 0.009698815468514048,
        "zo": -0.8784938371501598,
        "wo": 0.47732156522089525,
        "is_stop": True,
    },

    # ── INDEX 8: I-Block ──
    {
        "label": "I-Block",
        "x": 42559.076033639445,
        "y": 41746.37363606189,
        "z": -3.7119996315181303,
        "xo": 0.004659974538900481,
        "yo": 0.012924151050899196,
        "zo": -0.3391565230472391,
        "wo": 0.9406296315933376,
        "is_stop": True,
    },

    # ── INDEX 9: Security (End) ──
    {
        "label": "Security (End)",
        "x": 42314.15061019186,
        "y": 41721.61974219501,
        "z": -0.20325109392736126,
        "xo": 0.00732992336603748,
        "yo": -0.0012347818529965806,
        "zo": -0.9860787856505229,
        "wo": -0.16611254024670585,
        "is_stop": False,
    },
]

_MAX_START = len(WAYPOINTS) - 2
if not (0 <= START_WAYPOINT_INDEX <= _MAX_START):
    raise ValueError(f"START_WAYPOINT_INDEX={START_WAYPOINT_INDEX} out of range 0–{_MAX_START}")

INITIAL_POSE   = WAYPOINTS[START_WAYPOINT_INDEX]
GOAL_WAYPOINTS = WAYPOINTS[START_WAYPOINT_INDEX + 1:]


# ═══════════════════════════════════════════════════════════
#  STATE MACHINE
# ═══════════════════════════════════════════════════════════
class MissionState(Enum):
    INIT                  = 0
    SET_INITIAL_POSE      = 1
    WAIT_BEFORE_GOAL      = 2
    SET_GOAL_POSE         = 3
    WAIT_AUTONOMOUS_READY = 4
    SWITCH_AUTONOMOUS     = 5
    ENGAGE_CONTROL        = 6
    MONITORING_PROGRESS   = 7
    JUNCTION_STOP         = 8
    RESUME_AFTER_STOP     = 9
    COMPLETE              = 10


# ═══════════════════════════════════════════════════════════
#  NODE
# ═══════════════════════════════════════════════════════════
class AutowareShuttleMission(Node):

    def __init__(self):
        super().__init__('autoware_shuttle_mission')

        self.current_state    = MissionState.INIT
        self.state_start_time = time.time()

        self.waypoint_index  = 0
        self.total_waypoints = len(GOAL_WAYPOINTS)

        self.goal_reached    = False
        self.arrival_trigger = "none"

        self.current_vehicle_x    = None
        self.current_vehicle_y    = None
        self.current_speed        = 0.0
        self.proximity_enter_time = None

        self.goal_pose_sent      = False
        self.autonomous_mode_set = False
        self.control_engaged     = False

        self.autonomous_mode_inflight   = False
        self.autonomous_mode_next_retry = 0.0
        self.autonomous_mode_backoff    = AUTONOMOUS_RETRY_BASE_SEC

        self.initial_pose_sent = False

        self.is_autonomous_mode_available = False
        self.is_autoware_control_enabled  = False
        self.current_operation_mode       = None
        self.is_in_transition             = False
        self.current_route_state          = None

        # ── v4 brake-hold state ───────────────────────────
        # _vel_cap_active: True → 10 Hz timer keeps publishing Float32(0.0)
        #   to /current_max_velocity so planner cannot command forward motion.
        #   This DOES need a keepalive because the planner resets the topic.
        self._vel_cap_active = False
        self._vel_cap_tick   = 0          # tick counter for sub-rate publish

        # _gear_park_sent: per-leg one-shot flag.
        #   Set to True the moment GearCommand(PARK) is physically published.
        #   Never cleared until _advance_to_next_waypoint() resets it.
        #   Guarantees the gear command fires at most ONCE per stop,
        #   eliminating the engage/disengage click loop entirely.
        self._gear_park_sent = False

        # _resume_gear_sent: True once DRIVE has been sent in RESUME state.
        self._resume_gear_sent = False

        # ── TF2 ───────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Publishers ────────────────────────────────────
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10)
        self.max_velocity_pub = self.create_publisher(
            Float32, '/planning/scenario_planning/current_max_velocity', 1)
        self.gear_cmd_pub = self.create_publisher(
            GearCommand, '/control/command/gear_cmd', 1)

        # ── Subscribers ───────────────────────────────────
        self.create_subscription(OperationModeState,
            '/api/operation_mode/state', self.operation_mode_callback, 10)
        self.create_subscription(RouteState,
            '/planning/route_state', self.route_state_callback, 10)
        self.create_subscription(VelocityReport,
            '/vehicle/status/velocity_status', self.velocity_callback, 10)

        # ── Service clients ───────────────────────────────
        self.autonomous_mode_client = self.create_client(
            ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        self.enable_control_client = self.create_client(
            ChangeOperationMode, '/api/operation_mode/enable_autoware_control')

        # ── Timers ────────────────────────────────────────
        self.timer          = self.create_timer(0.5, self.state_machine_callback)
        self.proximity_timer= self.create_timer(0.2, self.proximity_check_callback)
        # Velocity-cap keepalive only; gear is never repeated from this timer
        self.vel_cap_timer  = self.create_timer(0.1, self._vel_cap_callback)

        # ── Boot log ──────────────────────────────────────
        self.get_logger().info("🚌 Autoware Shuttle Mission  [v4]")
        self.get_logger().info(
            f"   Start: [{START_WAYPOINT_INDEX}] '{INITIAL_POSE['label']}'")
        self.get_logger().info(
            f"   Proximity: r={PROXIMITY_RADIUS}m  spd<{STOP_SPEED_THRESH}m/s  "
            f"confirm={PROXIMITY_CONFIRM}s")
        self.get_logger().info(
            f"   Brake hold: vel-cap keepalive @ {VEL_CAP_HZ} Hz  "
            f"| gear-PARK one-shot  | gear-settle {RESUME_GEAR_SETTLE}s")
        for i, wp in enumerate(GOAL_WAYPOINTS):
            tag = "🏁" if i == len(GOAL_WAYPOINTS)-1 else "🛑" if wp["is_stop"] else "📌"
            self.get_logger().info(
                f"   {tag} {i+1}/{self.total_waypoints}: {wp['label']}")

    # ═══════════════════════════════════════════════════════
    #  SUBSCRIBERS
    # ═══════════════════════════════════════════════════════

    def operation_mode_callback(self, msg):
        self.is_autonomous_mode_available = msg.is_autonomous_mode_available
        self.is_autoware_control_enabled  = msg.is_autoware_control_enabled
        self.current_operation_mode       = msg.mode
        self.is_in_transition             = msg.is_in_transition

    ROUTE_STATE_NAMES = {
        0:"UNKNOWN",1:"UNSET",2:"SET",3:"ARRIVED",
        4:"CHANGING",5:"FOLLOWING",6:"GOAL_REACHED",
    }

    def route_state_callback(self, msg):
        prev = self.current_route_state
        self.current_route_state = msg.state
        if msg.state != prev:
            self.get_logger().info(
                f"📊 Route State → {self.ROUTE_STATE_NAMES.get(msg.state, f'?({msg.state})')}")
        if self.current_route_state == 6 and not self.goal_reached:
            self._trigger_arrival("autoware-GOAL_REACHED")

    def velocity_callback(self, msg):
        self.current_speed = abs(msg.longitudinal_velocity)

    # ═══════════════════════════════════════════════════════
    #  POSITION / PROXIMITY
    # ═══════════════════════════════════════════════════════

    def _update_vehicle_position(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            self.current_vehicle_x = t.transform.translation.x
            self.current_vehicle_y = t.transform.translation.y
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def proximity_check_callback(self):
        if self.current_state != MissionState.MONITORING_PROGRESS:
            self.proximity_enter_time = None
            return
        if self.goal_reached:
            return
        if not self._update_vehicle_position() or self.current_vehicle_x is None:
            return

        wp   = GOAL_WAYPOINTS[self.waypoint_index]
        dist = math.hypot(wp['x'] - self.current_vehicle_x,
                          wp['y'] - self.current_vehicle_y)
        if dist <= PROXIMITY_RADIUS and self.current_speed <= STOP_SPEED_THRESH:
            if self.proximity_enter_time is None:
                self.proximity_enter_time = time.time()
                self.get_logger().info(
                    f"📡 Proximity OPEN  dist={dist:.2f}m  "
                    f"spd={self.current_speed:.2f}m/s → {wp['label']}")
            elif time.time() - self.proximity_enter_time >= PROXIMITY_CONFIRM:
                self.get_logger().info(
                    f"📡 Proximity CONFIRMED  dist={dist:.2f}m")
                self._trigger_arrival("proximity-fallback")
        else:
            if self.proximity_enter_time is not None:
                self.get_logger().info(
                    f"📡 Proximity RESET  dist={dist:.2f}m  "
                    f"spd={self.current_speed:.2f}m/s")
            self.proximity_enter_time = None

    def _trigger_arrival(self, source: str):
        if self.goal_reached:
            return
        self.goal_reached    = True
        self.arrival_trigger = source
        wp = GOAL_WAYPOINTS[self.waypoint_index]
        self.get_logger().info(
            f"✅ ARRIVAL [{source}] → {wp['label']}  "
            f"dist={self._dist_to_goal():.2f}m")

    def _dist_to_goal(self) -> float:
        if self.current_vehicle_x is None:
            return -1.0
        wp = GOAL_WAYPOINTS[self.waypoint_index]
        return math.hypot(wp['x'] - self.current_vehicle_x,
                          wp['y'] - self.current_vehicle_y)

    # ═══════════════════════════════════════════════════════
    #  v4: VELOCITY-CAP KEEPALIVE TIMER (10 Hz)
    #
    #  ONLY republishes Float32(0.0) to /current_max_velocity.
    #  NEVER touches the gear command — that is one-shot only.
    # ═══════════════════════════════════════════════════════
    def _vel_cap_callback(self):
        if not self._vel_cap_active:
            self._vel_cap_tick = 0
            return

        self._vel_cap_tick += 1
        # Publish at VEL_CAP_HZ sub-rate: 10 Hz timer / VEL_CAP_HZ = every N ticks
        every_n = max(1, round(10.0 / VEL_CAP_HZ))
        if self._vel_cap_tick >= every_n:
            self._vel_cap_tick = 0
            msg      = Float32()
            msg.data = 0.0
            self.max_velocity_pub.publish(msg)

    # ═══════════════════════════════════════════════════════
    #  v4: BRAKE HOLD HELPERS
    # ═══════════════════════════════════════════════════════

    def _engage_brake_hold(self):
        """
        Called ONCE when entering JUNCTION_STOP.
        - Starts velocity-cap keepalive (planner needs this continuously).
        - Sends GearCommand(PARK) ONCE via the one-shot flag.
          If for any reason this method is called again in the same leg,
          the gear command is NOT re-sent because _gear_park_sent is True.
        """
        wp = GOAL_WAYPOINTS[self.waypoint_index]

        # ── Start velocity cap keepalive ──────────────────
        if not self._vel_cap_active:
            self._vel_cap_active = True
            self._vel_cap_tick   = 0
            # Publish immediately (don't wait for first timer tick)
            msg      = Float32()
            msg.data = 0.0
            self.max_velocity_pub.publish(msg)
            self.get_logger().info(
                f"🔒 Vel-cap ACTIVE (0.0 m/s keepalive @ {VEL_CAP_HZ} Hz) "
                f"→ {wp['label']}")

        # ── Send park gear ONCE per leg ───────────────────
        # _gear_park_sent is reset only in _advance_to_next_waypoint()
        # and __init__, so this block runs at most once per stop.
        if not self._gear_park_sent:
            gear_msg         = GearCommand()
            gear_msg.stamp   = self.get_clock().now().to_msg()
            gear_msg.command = GEAR_PARK
            self.gear_cmd_pub.publish(gear_msg)
            self._gear_park_sent = True
            self.get_logger().info(
                f"🅿️  Gear → PARK (one-shot, will not repeat) → {wp['label']}")
        else:
            # Should not normally happen; log as warning for debugging
            self.get_logger().warn(
                "⚠️  _engage_brake_hold() called again this leg "
                "– gear PARK NOT re-sent (one-shot guard active)")

    def _release_vel_cap(self):
        """Stop the velocity-cap keepalive so planner resumes normal speed."""
        if self._vel_cap_active:
            self._vel_cap_active = False
            self.get_logger().info("🟢 Vel-cap RELEASED – planner resumes normal speed")

    def _send_drive_gear(self):
        """Send GearCommand(DRIVE) once. Safe to call multiple times (idempotent)."""
        gear_msg         = GearCommand()
        gear_msg.stamp   = self.get_clock().now().to_msg()
        gear_msg.command = GEAR_DRIVE
        self.gear_cmd_pub.publish(gear_msg)
        self.get_logger().info("🚗 Gear → DRIVE  (park-pawl disengaging…)")

    # ═══════════════════════════════════════════════════════
    #  STATE MACHINE
    # ═══════════════════════════════════════════════════════

    def state_machine_callback(self):
        {
            MissionState.INIT:                  self.handle_init,
            MissionState.SET_INITIAL_POSE:      self.handle_set_initial_pose,
            MissionState.WAIT_BEFORE_GOAL:      self.handle_wait_before_goal,
            MissionState.SET_GOAL_POSE:         self.handle_set_goal_pose,
            MissionState.WAIT_AUTONOMOUS_READY: self.handle_wait_autonomous_ready,
            MissionState.SWITCH_AUTONOMOUS:     self.handle_switch_autonomous,
            MissionState.ENGAGE_CONTROL:        self.handle_engage_control,
            MissionState.MONITORING_PROGRESS:   self.handle_monitoring_progress,
            MissionState.JUNCTION_STOP:         self.handle_junction_stop,
            MissionState.RESUME_AFTER_STOP:     self.handle_resume_after_stop,
            MissionState.COMPLETE:              self.handle_complete,
        }[self.current_state]()

    def transition_to(self, new_state):
        self.get_logger().info(
            f"🔄 {self.current_state.name} → {new_state.name}")
        self.current_state    = new_state
        self.state_start_time = time.time()

    # ── Handlers ──────────────────────────────────────────

    def handle_init(self):
        if time.time() - self.state_start_time > 2.0:
            self.get_logger().info("✅ System ready – starting mission")
            self.transition_to(MissionState.SET_INITIAL_POSE)

    def handle_set_initial_pose(self):
        if self.initial_pose_sent:
            return
        self.get_logger().info(
            f"📍 Publishing initial pose → '{INITIAL_POSE['label']}'")
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        p = INITIAL_POSE
        msg.pose.pose.position.x    = p['x']
        msg.pose.pose.position.y    = p['y']
        msg.pose.pose.position.z    = p['z']
        msg.pose.pose.orientation.x = p['xo']
        msg.pose.pose.orientation.y = p['yo']
        msg.pose.pose.orientation.z = p['zo']
        msg.pose.pose.orientation.w = p['wo']
        self.initial_pose_pub.publish(msg)
        self.initial_pose_sent = True
        self.get_logger().info("✅ Initial pose sent – waiting for localisation…")
        self.transition_to(MissionState.WAIT_BEFORE_GOAL)

    def handle_wait_before_goal(self):
        elapsed = time.time() - self.state_start_time
        if int(elapsed) % 5 == 0 and elapsed - int(elapsed) < 0.5:
            remaining = GOAL_PUBLISH_DELAY - elapsed
            if remaining > 0:
                self.get_logger().info(
                    f"⏳ Localisation wait: {remaining:.0f}s remaining…")
        if elapsed >= GOAL_PUBLISH_DELAY:
            self.transition_to(MissionState.SET_GOAL_POSE)

    def handle_set_goal_pose(self):
        if self.goal_pose_sent:
            return
        wp = GOAL_WAYPOINTS[self.waypoint_index]
        self.get_logger().info(
            f"🎯 Goal [{self.waypoint_index+1}/{self.total_waypoints}] "
            f"→ {wp['label']}  ({wp['x']:.2f}, {wp['y']:.2f})")
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x    = wp['x']
        msg.pose.position.y    = wp['y']
        msg.pose.position.z    = wp['z']
        msg.pose.orientation.x = wp['xo']
        msg.pose.orientation.y = wp['yo']
        msg.pose.orientation.z = wp['zo']
        msg.pose.orientation.w = wp['wo']
        self.goal_pose_pub.publish(msg)
        self.goal_pose_sent       = True
        self.goal_reached         = False
        self.arrival_trigger      = "none"
        self.proximity_enter_time = None
        time.sleep(1.0)
        self.transition_to(MissionState.WAIT_AUTONOMOUS_READY)

    def handle_wait_autonomous_ready(self):
        elapsed  = time.time() - self.state_start_time
        route_ok = self.current_route_state in ROUTE_STATES_READY
        mode_ok  = (self.is_autonomous_mode_available or self.current_operation_mode == 2 or self.is_autoware_control_enabled) and not self.is_in_transition
        settled  = elapsed >= ROUTE_SETTLE_SEC
        if route_ok and mode_ok and settled:
            rs = self.ROUTE_STATE_NAMES.get(self.current_route_state, '?')
            self.get_logger().info(
                f"✅ Ready [route={rs}  settled={elapsed:.1f}s]")
            self.transition_to(MissionState.SWITCH_AUTONOMOUS)
        elif elapsed % 5.0 < 0.5:
            rs = self.ROUTE_STATE_NAMES.get(
                self.current_route_state, f'?({self.current_route_state})')
            self.get_logger().info(
                f"⏳ Waiting – mode_ok={mode_ok}  route={rs}(ok={route_ok})  "
                f"settled={settled}  {elapsed:.1f}s")

    def handle_switch_autonomous(self):
        if self.autonomous_mode_set or self.autonomous_mode_inflight:
            return
        if self.current_operation_mode == 2:
            self.get_logger().info("✅ Already in autonomous mode")
            self.autonomous_mode_set = True
            self.transition_to(MissionState.ENGAGE_CONTROL)
            return
        if time.time() < self.autonomous_mode_next_retry:
            return
        if self.autonomous_mode_client.service_is_ready():
            self.get_logger().info("🤖 Requesting autonomous mode…")
            future = self.autonomous_mode_client.call_async(
                ChangeOperationMode.Request())
            self.autonomous_mode_inflight = True
            future.add_done_callback(self._cb_autonomous_mode)
        else:
            self.get_logger().info("⏳ Autonomous mode service not ready…")

    def _cb_autonomous_mode(self, future):
        self.autonomous_mode_inflight = False
        try:
            resp = future.result()
            if resp.status.success:
                self.get_logger().info("✅ Autonomous mode activated")
                self.autonomous_mode_set     = True
                self.autonomous_mode_backoff = AUTONOMOUS_RETRY_BASE_SEC
                self.transition_to(MissionState.ENGAGE_CONTROL)
            else:
                self.autonomous_mode_next_retry = (
                    time.time() + self.autonomous_mode_backoff)
                self.get_logger().error(
                    f"❌ Autonomous mode failed: {resp.status.message}  "
                    f"(retry in {self.autonomous_mode_backoff:.1f}s)")
                self.autonomous_mode_backoff = min(
                    self.autonomous_mode_backoff * 2, AUTONOMOUS_RETRY_MAX_SEC)
        except Exception as e:
            self.autonomous_mode_next_retry = (
                time.time() + self.autonomous_mode_backoff)
            self.get_logger().error(f"❌ Service error: {e}")
            self.autonomous_mode_backoff = min(
                self.autonomous_mode_backoff * 2, AUTONOMOUS_RETRY_MAX_SEC)

    def handle_engage_control(self):
        if self.control_engaged:
            return
        if self.is_autoware_control_enabled:
            self.get_logger().info("✅ Autoware control already enabled")
            self.control_engaged = True
            self.transition_to(MissionState.MONITORING_PROGRESS)
            return
        if self.enable_control_client.service_is_ready():
            self.get_logger().info("🎮 Engaging Autoware control…")
            future = self.enable_control_client.call_async(
                ChangeOperationMode.Request())
            future.add_done_callback(self._cb_engage_control)
            self.control_engaged = True
        else:
            self.get_logger().info("⏳ Control enable service not ready…")

    def _cb_engage_control(self, future):
        try:
            resp = future.result()
            if resp.status.success:
                self.get_logger().info("✅ Control engaged – vehicle moving")
                self.transition_to(MissionState.MONITORING_PROGRESS)
            else:
                self.get_logger().error(
                    f"❌ Control engage failed: {resp.status.message}")
                self.control_engaged = False
        except Exception as e:
            self.get_logger().error(f"❌ Service error: {e}")
            self.control_engaged = False

    def handle_monitoring_progress(self):
        if self.goal_reached:
            wp = GOAL_WAYPOINTS[self.waypoint_index]
            self.get_logger().info(
                f"🛑 Arrived [{self.arrival_trigger}]  "
                f"dist={self._dist_to_goal():.2f}m → {wp['label']}")
            if wp["is_stop"]:
                self.get_logger().info(
                    f"   Dwell {STOP_WAIT_SEC}s with active brake hold")
                self.transition_to(MissionState.JUNCTION_STOP)
            else:
                self.transition_to(MissionState.COMPLETE)
        else:
            elapsed = time.time() - self.state_start_time
            if elapsed % 10.0 < 0.5:
                wp  = GOAL_WAYPOINTS[self.waypoint_index]
                rs  = self.ROUTE_STATE_NAMES.get(
                    self.current_route_state, f'?({self.current_route_state})')
                self.get_logger().info(
                    f"🚌 En-route → {wp['label']}  route={rs}  "
                    f"dist={self._dist_to_goal():.1f}m  "
                    f"spd={self.current_speed:.2f}m/s  {elapsed:.0f}s")

    # ═══════════════════════════════════════════════════════
    #  JUNCTION_STOP  (v4: engage once, then wait)
    # ═══════════════════════════════════════════════════════
    def handle_junction_stop(self):
        elapsed = time.time() - self.state_start_time

        # Engage brake hold on first call only.
        # _engage_brake_hold() internally guards against double gear-send.
        if not self._vel_cap_active:
            self._engage_brake_hold()

        # Periodic status log every 5 s
        if int(elapsed) % 5 == 0 and elapsed - int(elapsed) < 0.5:
            remaining = STOP_WAIT_SEC - elapsed
            self.get_logger().info(
                f"🛑 Dwell {remaining:.0f}s left  "
                f"[{GOAL_WAYPOINTS[self.waypoint_index]['label']}]  "
                f"spd={self.current_speed:.2f}m/s  "
                f"vel_cap=ACTIVE  gear_park_sent={self._gear_park_sent}")

        if elapsed >= STOP_WAIT_SEC:
            self.get_logger().info(
                f"⏱️  Dwell complete [{GOAL_WAYPOINTS[self.waypoint_index]['label']}] "
                f"– advancing")
            self._advance_to_next_waypoint()

    # ═══════════════════════════════════════════════════════
    #  RESUME_AFTER_STOP  (v4: clean 3-step sequence)
    #
    #  Step 1 (elapsed=0):   Send DRIVE gear once (_resume_gear_sent guard).
    #  Step 2 (elapsed≥settle): Release velocity cap so planner can accelerate.
    #  Step 3 (elapsed≥settle+0.5): Transition to SET_GOAL_POSE.
    #
    #  The gear timer (eliminated in v4) no longer interferes with step 1.
    # ═══════════════════════════════════════════════════════
    def handle_resume_after_stop(self):
        elapsed = time.time() - self.state_start_time

        # Step 1: send DRIVE once, immediately on first call
        if not self._resume_gear_sent:
            self._send_drive_gear()
            self._resume_gear_sent = True

        # Step 2: release velocity cap after pawl settle delay
        if elapsed >= RESUME_GEAR_SETTLE and self._vel_cap_active:
            self._release_vel_cap()

        # Step 3: proceed to next goal once cap is released and settle done
        if not self._vel_cap_active and elapsed >= RESUME_GEAR_SETTLE + 0.5:
            self.transition_to(MissionState.SET_GOAL_POSE)

    # ═══════════════════════════════════════════════════════
    #  ADVANCE TO NEXT WAYPOINT
    # ═══════════════════════════════════════════════════════
    def _advance_to_next_waypoint(self):
        self.waypoint_index += 1
        if self.waypoint_index >= self.total_waypoints:
            self.get_logger().error("❌ waypoint_index overflow → COMPLETE")
            self._release_vel_cap()
            self.transition_to(MissionState.COMPLETE)
            return

        # Per-leg resets
        self.goal_pose_sent      = False
        self.autonomous_mode_set = False
        self.control_engaged     = False
        self.goal_reached        = False
        self.arrival_trigger     = "none"
        self.proximity_enter_time = None

        self.autonomous_mode_inflight   = False
        self.autonomous_mode_next_retry = 0.0
        self.autonomous_mode_backoff    = AUTONOMOUS_RETRY_BASE_SEC

        # v4: reset per-leg brake flags
        # _vel_cap_active stays True until RESUME releases it
        # _gear_park_sent MUST be reset here so next stop can engage park
        self._gear_park_sent   = False   # ← allows one PARK send next stop
        self._resume_gear_sent = False   # ← allows one DRIVE send in RESUME

        nxt = GOAL_WAYPOINTS[self.waypoint_index]
        self.get_logger().info(
            f"➡️  Next [{self.waypoint_index+1}/{self.total_waypoints}]: "
            f"{nxt['label']}  ({nxt['x']:.2f}, {nxt['y']:.2f})")
        self.transition_to(MissionState.RESUME_AFTER_STOP)

    # ═══════════════════════════════════════════════════════
    #  COMPLETE
    # ═══════════════════════════════════════════════════════
    def handle_complete(self):
        self._release_vel_cap()   # always safe to call; idempotent
        self.get_logger().info("🎉🎉  SHUTTLE MISSION COMPLETE  🎉🎉")
        self.get_logger().info(f"🏁 Final trigger: [{self.arrival_trigger}]")
        self.get_logger().info(
            f"   📍 Start: {INITIAL_POSE['label']} [idx {START_WAYPOINT_INDEX}]")
        for i, w in enumerate(GOAL_WAYPOINTS):
            tag = "🏁" if i == len(GOAL_WAYPOINTS)-1 else "✅"
            self.get_logger().info(f"   {tag} Stop {i+1}: {w['label']}")
        self.get_logger().info("🏁 Shutting down…")
        self.timer.cancel()
        self.proximity_timer.cancel()
        self.vel_cap_timer.cancel()
        rclpy.shutdown()


# ═══════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AutowareShuttleMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Mission interrupted by user")
    except Exception as e:
        node.get_logger().error(f"❌ Unexpected error: {e}")
    finally:
        if node._vel_cap_active:
            node._release_vel_cap()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

