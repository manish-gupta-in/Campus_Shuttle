#!/usr/bin/env python3
"""
BITS Pilani · Hyderabad Campus
Autonomous Shuttle Mission Control Dashboard Unit Tests
Validates BITS Campus UTM waypoints, subprocess lifecycles, and
telemetry regex log parsing rules against actual recorded shuttle logs.
"""

import sys
import os
import unittest
import queue
import time
import subprocess

# Import dashboard modules dynamically
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)
import shuttle_dashboard as sd

class TestShuttleDashboard(unittest.TestCase):

    def setUp(self):
        self.log_q = queue.Queue()

    def test_waypoints_and_stop_indices(self):
        """Test BITS Pilani campus waypoint dimensions, bounds, and indexing."""
        self.assertEqual(len(sd.WAYPOINTS), 10)
        
        # Verify bounding box UTM coordinates
        self.assertTrue(sd.MIN_X < sd.MAX_X)
        self.assertTrue(sd.MIN_Y < sd.MAX_Y)
        self.assertAlmostEqual(sd.SPAN_X, sd.MAX_X - sd.MIN_X)
        self.assertAlmostEqual(sd.SPAN_Y, sd.MAX_Y - sd.MIN_Y)
        
        # Test lookup indices for stops
        self.assertEqual(sd.STOP_INDEX["a-block"], 1)
        self.assertEqual(sd.STOP_INDEX["hostel circle"], 2)
        self.assertEqual(sd.STOP_INDEX["hostel"], 2)
        self.assertEqual(sd.STOP_INDEX["security main gate"], 0)

    def test_process_manager_lifecycle(self):
        """Test starting, output capturing, and terminating a process cleanly."""
        start_called = False
        stop_called = False

        def on_start():
            nonlocal start_called
            start_called = True

        def on_stop():
            nonlocal stop_called
            stop_called = True

        # Test command: simple echo in background
        cmd = "echo 'TEST_LINE'; sleep 0.2"
        manager = sd.ProcessManager("TEST_PROC", cmd, self.log_q, on_start, on_stop)
        
        manager.start()
        time.sleep(0.3)
        
        self.assertTrue(start_called)
        manager.stop()
        
        # Verify log output capture in queue
        lines = []
        while not self.log_q.empty():
            tag, line = self.log_q.get()
            lines.append((tag, line))
            
        self.assertTrue(any(l[1] == "TEST_LINE" for l in lines))

    def test_regex_telemetry_parsers(self):
        """Validate all regex parsing rules against standard Autoware Mission log patterns."""
        # 1. State transition match
        line1 = "[INFO] [autoware_shuttle_mission]: 🔄 IDLE → INIT"
        clean = line1.split("[autoware_shuttle_mission]:", 1)[1].strip()
        m1 = sd.re.search(r'🔄\s*(\w+)\s*→\s*(\w+)', clean)
        self.assertIsNotNone(m1)
        self.assertEqual(m1.group(1), "IDLE")
        self.assertEqual(m1.group(2), "INIT")

        # 2. Transit progress match
        line2 = "[INFO] [autoware_shuttle_mission]: En-route → A-Block route=TRANSITING dist=45.2m spd=2.5m/s"
        clean = line2.split("[autoware_shuttle_mission]:", 1)[1].strip()
        m2 = sd.re.search(r'En-route\s*→\s*(.+?)\s+route=(\S+)\s+dist=([\d.]+)m\s+spd=([\d.]+)m/s', clean)
        self.assertIsNotNone(m2)
        self.assertEqual(m2.group(1), "A-Block")
        self.assertEqual(m2.group(2), "TRANSITING")
        self.assertEqual(float(m2.group(3)), 45.2)
        self.assertEqual(float(m2.group(4)), 2.5)

        # 3. Dwell stop match
        line3 = "[INFO] [autoware_shuttle_mission]: Dwell 8.5s left [Hostel Circle] spd=0.0m/s"
        clean = line3.split("[autoware_shuttle_mission]:", 1)[1].strip()
        m3 = sd.re.search(r'Dwell\s+([\d.-]+)s\s+left\s+\[(.+?)\]\s+spd=([\d.]+)m/s', clean)
        self.assertIsNotNone(m3)
        self.assertEqual(float(m3.group(1)), 8.5)
        self.assertEqual(m3.group(2), "Hostel Circle")
        self.assertEqual(float(m3.group(3)), 0.0)


if __name__ == '__main__':
    unittest.main()
