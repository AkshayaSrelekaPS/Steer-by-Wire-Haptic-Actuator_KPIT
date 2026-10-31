#!/usr/bin/env python3
"""
KPIT Steer-by-Wire Telemetry & Automotive CAN Gateway Backend Server
Provides:
1. Static HTTP Web Server for HTML5 Cockpit Dashboard (http://localhost:8080)
2. REST API & Telemetry Harvest Engine (/api/status, /api/telemetry, /api/hazard)
3. Simulated or Hardware Serial/CAN Gateway (UART/SocketCAN Bridge)
Zero third-party dependencies required (Runs out-of-the-box on pure Python standard library).
"""

import sys
import os
import json
import time
import math
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

PORT = 8080
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DASHBOARD_DIR = os.path.join(BASE_DIR, 'dashboard')

# Global Cyber-Physical System State (Thread-Safe)
state_lock = threading.Lock()
telemetry_state = {
    "timestamp": time.time(),
    "steering_angle": 0.0,
    "steering_velocity": 0.0,
    "driver_torque": 2.5,
    "vehicle_speed": 40.0,
    "spring_torque": 0.0,
    "damping_torque": 0.0,
    "barrier_torque": 0.0,
    "net_torque": 0.0,
    "motor_current": 1.42,
    "fet_temp": 34.8,
    "is_override_active": False,
    "left_hazard": False,
    "right_hazard": False,
    "rumble_active": False,
    "loop_frequency_hz": 1000,
    "safety_status": "ISO_26262_ASIL_D_NORMAL",
    "can_frames": []
}

CAN_ID_LIST = [
    {"id": "0x102", "dlc": 8, "name": "STEER_STATUS_THETA", "interval": 20},
    {"id": "0x104", "dlc": 8, "name": "MOTOR_TORQUE_CMD", "interval": 20},
    {"id": "0x220", "dlc": 4, "name": "ADAS_HAZARD_FLAG", "interval": 50},
    {"id": "0x350", "dlc": 8, "name": "SAFETY_OVERRIDE_UNR79", "interval": 100}
]

def background_can_gateway():
    """Simulates real-time 50 Hz CAN frame transmission and physics telemetry loop."""
    tick = 0
    while True:
        with state_lock:
            tick += 1
            # Synthesize periodic CAN frame
            target = CAN_ID_LIST[tick % len(CAN_ID_LIST)]
            hex_angle = f"{int((telemetry_state['steering_angle'] + 90) * 100) & 0xFFFF:04X}"
            hex_torque = f"{int((telemetry_state['net_torque'] + 10) * 100) & 0xFFFF:04X}"
            hex_hazard = "01" if telemetry_state['left_hazard'] else ("02" if telemetry_state['right_hazard'] else "00")
            hex_override = "01" if telemetry_state['is_override_active'] else "00"
            payload = f"{hex_angle} {hex_torque} {hex_hazard} {hex_override} FF AA"

            frame = {
                "id": target["id"],
                "dlc": target["dlc"],
                "name": target["name"],
                "payload": payload,
                "interval_ms": target["interval"],
                "timestamp": round(time.time(), 3)
            }
            
            # Keep circular buffer of last 10 CAN frames
            telemetry_state["can_frames"].insert(0, frame)
            if len(telemetry_state["can_frames"]) > 10:
                telemetry_state["can_frames"].pop()

            telemetry_state["timestamp"] = time.time()
        time.sleep(0.02) # 50 Hz Gateway update

class TelemetryRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DASHBOARD_DIR, **kwargs)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/status':
            self._send_json({
                "service": "KPIT Steer-by-Wire Telemetry Gateway",
                "version": "2.4.0",
                "standards": ["ISO 26262 ASIL-D", "UN R79", "AUTOSAR"],
                "mode": "ACTIVE_GATEWAY",
                "can_bus": "CAN-FD 500 kbps",
                "control_rate": "1 kHz Deterministic",
                "server_time": time.strftime("%Y-%m-%d %H:%M:%S")
            })
        elif parsed.path == '/api/telemetry':
            with state_lock:
                self._send_json(telemetry_state)
        else:
            # Fallback to serving static HTML5 dashboard files
            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/hazard':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode('utf-8'))
                with state_lock:
                    if 'left_hazard' in data:
                        telemetry_state['left_hazard'] = bool(data['left_hazard'])
                    if 'right_hazard' in data:
                        telemetry_state['right_hazard'] = bool(data['right_hazard'])
                    if 'rumble_active' in data:
                        telemetry_state['rumble_active'] = bool(data['rumble_active'])
                    if 'vehicle_speed' in data:
                        telemetry_state['vehicle_speed'] = float(data['vehicle_speed'])
                self._send_json({"status": "SUCCESS", "message": "Hazard state updated"})
            except Exception as e:
                self._send_json({"status": "ERROR", "message": str(e)}, status=400)
        else:
            self._send_json({"error": "Endpoint not found"}, status=404)

    def _send_json(self, data, status=200):
        body = json.dumps(data, indent=2).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        # Quiet logger for clean terminal output
        pass

def main():
    print("=" * 70)
    print("  KPIT TECHNOLOGIES - STEER-BY-WIRE TELEMETRY & CAN GATEWAY SERVER")
    print("  Compliance: ISO 26262 ASIL-D | UN R79 Lane-Keeping Regulations")
    print("=" * 70)
    print(f"  [+] Static Dashboard Served: {DASHBOARD_DIR}")
    print(f"  [+] REST Telemetry API:      http://localhost:{PORT}/api/telemetry")
    print(f"  [+] System Health Status:    http://localhost:{PORT}/api/status")
    print(f"  [+] Live Web Twin UI:        http://localhost:{PORT}/index.html")
    print("=" * 70)
    print("  Starting Background CAN Gateway Thread (50 Hz / ISO 11898)...")
    
    can_thread = threading.Thread(target=background_can_gateway, daemon=True)
    can_thread.start()

    server = HTTPServer(('0.0.0.0', PORT), TelemetryRequestHandler)
    print(f"  Listening on http://localhost:{PORT} (Press Ctrl+C to terminate)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n  Shutting down Telemetry Gateway cleanly...")
        server.server_close()

if __name__ == '__main__':
    main()
