import os
import csv
import time
import threading
import serial.tools.list_ports
from datetime import datetime
from flask import Flask, jsonify, render_template_string
from pymavlink import mavutil

app = Flask(__name__)

# Global dictionary holding real-time APM telemetry
apm_telemetry = {
    "connected": False,
    "system_status": "DISCONNECTED",
    "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "gps": {"lat": 0.0, "lng": 0.0, "alt": 0.0, "satellites": 0, "fix_type": "NO FIX"},
    "power": {"voltage": 0.0, "current": 0.0, "remaining": 0},
    "vfr_hud": {"airspeed": 0.0, "groundspeed": 0.0, "heading": 0, "throttle": 0, "climb": 0.0},
    "heartbeat": {"mode": "UNKNOWN", "armed": False}
}

BAUD_RATE = 115200

# Setup Flight Logging Directory and File
LOG_DIR = "flight_logs"
os.makedirs(LOG_DIR, exist_ok=True)
current_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(LOG_DIR, f"flight_log_{current_time_str}.csv")

# Initialize CSV file with headers
with open(LOG_FILE, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "timestamp", "mode", "armed", "lat", "lng", "alt_m", 
        "satellites", "fix_type", "roll_deg", "pitch_deg", "yaw_deg", 
        "groundspeed_m_s", "airspeed_m_s", "climb_m_s", "throttle_pct", 
        "voltage_v", "current_a", "battery_remaining_pct"
    ])

def log_telemetry_loop():
    """Background thread to log telemetry to CSV every 1 second."""
    while True:
        if apm_telemetry["connected"]:
            try:
                with open(LOG_FILE, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([
                        datetime.now().isoformat(),
                        apm_telemetry["heartbeat"]["mode"],
                        apm_telemetry["heartbeat"]["armed"],
                        apm_telemetry["gps"]["lat"],
                        apm_telemetry["gps"]["lng"],
                        apm_telemetry["gps"]["alt"],
                        apm_telemetry["gps"]["satellites"],
                        apm_telemetry["gps"]["fix_type"],
                        apm_telemetry["attitude"]["roll"],
                        apm_telemetry["attitude"]["pitch"],
                        apm_telemetry["attitude"]["yaw"],
                        apm_telemetry["vfr_hud"]["groundspeed"],
                        apm_telemetry["vfr_hud"]["airspeed"],
                        apm_telemetry["vfr_hud"]["climb"],
                        apm_telemetry["vfr_hud"]["throttle"],
                        apm_telemetry["power"]["voltage"],
                        apm_telemetry["power"]["current"],
                        apm_telemetry["power"]["remaining"]
                    ])
            except Exception as e:
                print(f"⚠️ Logging Error: {e}")
        time.sleep(1.0)  # Log at 1Hz rate

def auto_detect_apm_port():
    """Scans system serial ports to find the APM flight controller."""
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
        desc = port.description.lower()
        if "arduino" in desc or "ardupilot" in desc or "apm" in desc or "pixhawk" in desc:
            print(f"🎯 Auto-detected APM device on {port.device} ({port.description})")
            return port.device

    for port in ports:
        if "usb" in port.description.lower() or "com" in port.device.lower():
            print(f"🔌 Using detected serial port: {port.device} ({port.description})")
            return port.device

    return None

def read_apm_telemetry():
    """Background thread to continuously fetch MAVLink data from APM 2.8."""
    global apm_telemetry

    while True:
        serial_port = auto_detect_apm_port()
        
        if not serial_port:
            print("⚠️ No APM flight controller found. Retrying in 3 seconds...")
            apm_telemetry["connected"] = False
            apm_telemetry["system_status"] = "SEARCHING FOR PORT..."
            time.sleep(3)
            continue

        try:
            print(f"🔌 Connecting to APM on {serial_port} at {BAUD_RATE} baud...")
            master = mavutil.mavlink_connection(serial_port, baud=BAUD_RATE)
            master.wait_heartbeat(timeout=5)
            print("✅ Heartbeat received!")
            print(f"📝 Logging flight data to: {LOG_FILE}")
            apm_telemetry["connected"] = True
            apm_telemetry["system_status"] = "CONNECTED"

            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4, 1
            )

            while True:
                msg = master.recv_match(blocking=True, timeout=2)
                if not msg:
                    continue

                msg_type = msg.get_type()

                if msg_type == 'HEARTBEAT':
                    mode = mavutil.mode_string_v10(msg)
                    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    apm_telemetry["heartbeat"]["mode"] = mode
                    apm_telemetry["heartbeat"]["armed"] = is_armed

                elif msg_type == 'ATTITUDE':
                    apm_telemetry["attitude"]["roll"] = round(msg.roll * (180 / 3.14159), 1)
                    apm_telemetry["attitude"]["pitch"] = round(msg.pitch * (180 / 3.14159), 1)
                    apm_telemetry["attitude"]["yaw"] = round(msg.yaw * (180 / 3.14159), 1)

                elif msg_type == 'GPS_RAW_INT':
                    apm_telemetry["gps"]["lat"] = msg.lat / 1e7
                    apm_telemetry["gps"]["lng"] = msg.lon / 1e7
                    apm_telemetry["gps"]["alt"] = round(msg.alt / 1000.0, 1)
                    apm_telemetry["gps"]["satellites"] = msg.satellites_visible
                    fix_map = {0: "NO FIX", 1: "NO FIX", 2: "2D FIX", 3: "3D FIX", 4: "DGPS", 5: "RTK FLT", 6: "RTK FIX"}
                    apm_telemetry["gps"]["fix_type"] = fix_map.get(msg.fix_type, "UNKNOWN")

                elif msg_type == 'SYS_STATUS':
                    apm_telemetry["power"]["voltage"] = round(msg.voltage_battery / 1000.0, 2)
                    apm_telemetry["power"]["current"] = round(msg.current_battery / 100.0, 2)
                    apm_telemetry["power"]["remaining"] = msg.battery_remaining

                elif msg_type == 'VFR_HUD':
                    apm_telemetry["vfr_hud"]["airspeed"] = round(msg.airspeed, 1)
                    apm_telemetry["vfr_hud"]["groundspeed"] = round(msg.groundspeed, 1)
                    apm_telemetry["vfr_hud"]["heading"] = msg.heading
                    apm_telemetry["vfr_hud"]["throttle"] = msg.throttle
                    apm_telemetry["vfr_hud"]["climb"] = round(msg.climb, 1)

        except Exception as e:
            apm_telemetry["connected"] = False
            apm_telemetry["system_status"] = f"OFFLINE ({str(e)})"
            time.sleep(3)

# Start Threads
telemetry_thread = threading.Thread(target=read_apm_telemetry, daemon=True)
telemetry_thread.start()

logging_thread = threading.Thread(target=log_telemetry_loop, daemon=True)
logging_thread.start()

# HTML Dashboard Template with Satellite Map
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Aquavision AI - APM 2.8 Live Telemetry & Satellite Tracking</title>
    
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />

    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --accent-blue: #38bdf8;
            --accent-green: #22c55e;
            --accent-red: #ef4444;
            --text-main: #f8fafc;
            --text-sub: #94a3b8;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-main);
            margin: 0;
            padding: 20px;
        }

        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #334155;
            padding-bottom: 15px;
            margin-bottom: 25px;
        }

        .title {
            font-size: 24px;
            font-weight: bold;
            color: var(--accent-blue);
        }

        .badge {
            padding: 6px 14px;
            border-radius: 20px;
            font-weight: bold;
            font-size: 14px;
        }

        .connected { background: #166534; color: #4ade80; }
        .disconnected { background: #991b1b; color: #fca5a5; }

        .dashboard-container {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 20px;
        }

        @media (max-width: 900px) {
            .dashboard-container {
                grid-template-columns: 1fr;
            }
        }

        #map-card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3);
            border: 1px solid #334155;
            display: flex;
            flex-direction: column;
        }

        #map {
            width: 100%;
            height: 480px;
            border-radius: 8px;
            z-index: 1;
        }

        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
            gap: 15px;
        }

        .card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3);
            border: 1px solid #334155;
        }

        .card-title {
            font-size: 15px;
            color: var(--text-sub);
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 15px;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .metric-row {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px dashed #334155;
        }

        .metric-row:last-child { border-bottom: none; }

        .label { color: var(--text-sub); }
        .value { font-weight: bold; color: var(--text-main); }
        .highlight { color: var(--accent-blue); font-size: 18px; }
    </style>
</head>
<body>

    <div class="header">
        <div class="title">🚁 Aquavision - APM 2.8 Telemetry & Flight Logger</div>
        <div id="connectionStatus" class="badge disconnected">DISCONNECTED</div>
    </div>

    <div class="dashboard-container">
        <div id="map-card">
            <div class="card-title">🛰️ Live Satellite Map (NEO-M8N)</div>
            <div id="map"></div>
        </div>

        <div class="grid">
            <div class="card">
                <div class="card-title">⚙️ Flight Status</div>
                <div class="metric-row"><span class="label">Flight Mode</span><span id="flightMode" class="value highlight">--</span></div>
                <div class="metric-row"><span class="label">Armed State</span><span id="armedState" class="value">--</span></div>
            </div>

            <div class="card">
                <div class="card-title">🛰️ GPS Location</div>
                <div class="metric-row"><span class="label">Latitude</span><span id="lat" class="value">0.000000</span></div>
                <div class="metric-row"><span class="label">Longitude</span><span id="lng" class="value">0.000000</span></div>
                <div class="metric-row"><span class="label">Altitude (MSL)</span><span id="alt" class="value">0.0m</span></div>
                <div class="metric-row"><span class="label">Satellites / Fix</span><span id="gpsFix" class="value">0 (NO FIX)</span></div>
            </div>

            <div class="card">
                <div class="card-title">📐 Attitude / Orientation</div>
                <div class="metric-row"><span class="label">Roll</span><span id="roll" class="value">0.0°</span></div>
                <div class="metric-row"><span class="label">Pitch</span><span id="pitch" class="value">0.0°</span></div>
                <div class="metric-row"><span class="label">Yaw / Heading</span><span id="yaw" class="value">0.0°</span></div>
            </div>

            <div class="card">
                <div class="card-title">🚀 Flight Dynamics</div>
                <div class="metric-row"><span class="label">Ground Speed</span><span id="groundspeed" class="value">0.0 m/s</span></div>
                <div class="metric-row"><span class="label">Airspeed</span><span id="airspeed" class="value">0.0 m/s</span></div>
                <div class="metric-row"><span class="label">Climb Rate</span><span id="climb" class="value">0.0 m/s</span></div>
                <div class="metric-row"><span class="label">Throttle</span><span id="throttle" class="value">0%</span></div>
            </div>

            <div class="card">
                <div class="card-title">🔋 Power System</div>
                <div class="metric-row"><span class="label">Battery Voltage</span><span id="voltage" class="value highlight">0.00V</span></div>
                <div class="metric-row"><span class="label">Current Draw</span><span id="current" class="value">0.00A</span></div>
                <div class="metric-row"><span class="label">Battery Remaining</span><span id="remaining" class="value">0%</span></div>
            </div>
        </div>
    </div>

    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"></script>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

    <script>
        const map = L.map('map').setView([0, 0], 2);

        L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            maxZoom: 19,
            attribution: 'Tiles &copy; Esri'
        }).addTo(map);

        L.tileLayer('https://{s}.basemaps.cartocdn.com/rastertiles/voyager_only_labels/{z}/{x}/{y}{r}.png', {
            maxZoom: 19,
            subdomains: 'abcd',
            pane: 'shadowPane'
        }).addTo(map);

        let droneMarker = L.marker([0, 0]).addTo(map);
        let flightPath = L.polyline([], { color: '#38bdf8', weight: 3 }).addTo(map);
        let hasInitialCentering = false;

        async function updateTelemetry() {
            try {
                const response = await fetch('/api/telemetry');
                const data = await response.json();

                const statusBadge = document.getElementById("connectionStatus");
                if (data.connected) {
                    statusBadge.innerText = "CONNECTED";
                    statusBadge.className = "badge connected";
                } else {
                    statusBadge.innerText = data.system_status.includes("SEARCHING") ? "SEARCHING..." : "OFFLINE";
                    statusBadge.className = "badge disconnected";
                }

                document.getElementById("flightMode").innerText = data.heartbeat.mode;
                document.getElementById("armedState").innerText = data.heartbeat.armed ? "ARMED 🔴" : "DISARMED 🟢";

                document.getElementById("roll").innerText = data.attitude.roll + "°";
                document.getElementById("pitch").innerText = data.attitude.pitch + "°";
                document.getElementById("yaw").innerText = data.attitude.yaw + "°";

                const lat = data.gps.lat;
                const lng = data.gps.lng;

                document.getElementById("lat").innerText = lat.toFixed(6);
                document.getElementById("lng").innerText = lng.toFixed(6);
                document.getElementById("alt").innerText = data.gps.alt + " m";
                document.getElementById("gpsFix").innerText = `${data.gps.satellites} Sats (${data.gps.fix_type})`;

                if (lat !== 0 && lng !== 0) {
                    const newLatLng = [lat, lng];
                    droneMarker.setLatLng(newLatLng);
                    flightPath.addLatLng(newLatLng);

                    if (!hasInitialCentering) {
                        map.setView(newLatLng, 18);
                        hasInitialCentering = true;
                    }
                }

                document.getElementById("groundspeed").innerText = data.vfr_hud.groundspeed + " m/s";
                document.getElementById("airspeed").innerText = data.vfr_hud.airspeed + " m/s";
                document.getElementById("climb").innerText = data.vfr_hud.climb + " m/s";
                document.getElementById("throttle").innerText = data.vfr_hud.throttle + "%";

                document.getElementById("voltage").innerText = data.power.voltage + " V";
                document.getElementById("current").innerText = data.power.current + " A";
                document.getElementById("remaining").innerText = data.power.remaining + "%";

            } catch (err) {
                console.error("Failed to fetch telemetry:", err);
            }
        }

        setInterval(updateTelemetry, 500);
    </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route("/api/telemetry")
def get_telemetry():
    return jsonify(apm_telemetry)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)