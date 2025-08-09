# TODO:
# Ability to save location data offline

# MAC
# RUN: /Users/lucas/.pyenv/versions/3.12.2/bin/python /Users/lucas/Documents/GitHub/GPS-Tracker-2W/embedded/software/GUI/main.py

# Windows
# RUN: C:/Users/Lucas/Documents/GitHub/GPS-Tracker-2W/.venv/Scripts/python.exe "c:/Users/Lucas/Documents/GitHub/GPS-Tracker-2W/computer software/GUI/main.py"

from flask import Flask, render_template_string, jsonify, send_from_directory
import threading
import time
import serial
import re
import glob
import os
from serial.tools import list_ports
import time as pytime

app = Flask(__name__)
gps_data = {
    "local": {"lat": -33.8708, "lon": 151.2073, "last_update": pytime.time()},
    "tracker": None
}
last_serial_message = {'msg': ''}
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    crossorigin=""
    />
    <meta charset="utf-8">
    <title>GPS Tracker</title>
    <style>
    html, body {
        height: 100%;
        margin: 0;
        padding: 0;
        font-family: 'Segoe UI', Arial, sans-serif;
    }
    #map {
        height: 90vh;
        min-height: 400px;
        width: 100%;
        background: #eee;
    }
    h2 {
        text-align: center;
    }
    /* Always-visible labels above markers */
    .marker-label.leaflet-tooltip {
        background: rgba(255,255,255,0.95);
        color: #111;
        border: 1px solid #555;
        border-radius: 4px;
        padding: 4px 6px;
        box-shadow: 0 1px 3px rgba(0,0,0,0.25);
        font-size: 12px;
    }
    .route-label.leaflet-tooltip {
        background: rgba(255, 255, 200, 0.95);
        border-color: #777;
        font-weight: 600;
    }
</style>
</head>
<body>
    <h2>GPS Tracker</h2>
    <div id="map"></div>

    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
            crossorigin=""></script>

    <script>
        document.addEventListener('DOMContentLoaded', function() {
            var mapDiv = document.getElementById('map');
            var local = {{ local|tojson }};
            var tracker = {{ tracker|tojson }};
            var mapCenter = local ? [local.lat, local.lon] : [0, 0];
            var map = L.map('map').setView(mapCenter, 15);

            // Online tile layer
            var onlineLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
                attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
                maxZoom: 18
            });
            // Offline tile layer
            var offlineLayer = L.tileLayer('/tiles/{z}/{x}/{y}.png', {
                attribution: 'Offline tiles',
                maxZoom: 11,
                errorTileUrl: 'https://upload.wikimedia.org/wikipedia/commons/6/65/No-Image-Placeholder.svg'
            });

            // Add the correct layer based on connectivity
            function setTileLayer() {
                if (navigator.onLine) {
                    if (map.hasLayer(offlineLayer)) map.removeLayer(offlineLayer);
                    if (!map.hasLayer(onlineLayer)) map.addLayer(onlineLayer);
                } else {
                    if (map.hasLayer(onlineLayer)) map.removeLayer(onlineLayer);
                    if (!map.hasLayer(offlineLayer)) map.addLayer(offlineLayer);
                }
            }
            setTileLayer();
            window.addEventListener('online', setTileLayer);
            window.addEventListener('offline', setTileLayer);

            var localMarker = null;
            var trackerMarker = null;
            var lastLocalUpdate = null;
            var lastTrackerUpdate = null;
            var localAge = 0;
            var trackerAge = 0;
            var localData = null;
            var trackerData = null;
            var routeLine = null;
            var distanceTooltip = null;

            function createOrUpdateMarker(marker, lat, lon, color, label) {
                var isLocal = (color === 'blue');
                var tooltipOpts = {
                    permanent: true,
                    direction: isLocal ? 'bottom' : 'top',
                    offset: isLocal ? [0, 30] : [0, -30],
                    className: 'marker-label ' + (isLocal ? 'local' : 'tracker')
                };
                if (marker) {
                    marker.setLatLng([lat, lon]);
                    if (marker.getTooltip && marker.getTooltip()) {
                        marker.setTooltipContent(label);
                    } else {
                        marker.bindTooltip(label, tooltipOpts);
                    }
                    return marker;
                } else {
                    var icon = L.icon({
                        iconUrl: color === 'blue' ? 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-blue.png' : 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
                        shadowUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
                        iconSize: [25, 41],
                        iconAnchor: [12, 41],
                        popupAnchor: [1, -34],
                        shadowSize: [41, 41]
                    });
                    var m = L.marker([lat, lon], {icon: icon, zIndexOffset: isLocal ? 0 : 0}).addTo(map);
                    m.bindTooltip(label, tooltipOpts);
                    return m;
                }
            }

            async function updatePosition() {
                try {
                    const response = await fetch("/position");
                    const data = await response.json();
                    // Fetch and print the latest serial message only if new
                    try {
                        const msgResp = await fetch("/last_serial_message");
                        const msgData = await msgResp.json();
                        if (!window._lastSerialMsg) window._lastSerialMsg = "";
                        if (msgData.msg && msgData.msg !== window._lastSerialMsg) {
                            console.log("Serial:", msgData.msg);
                            window._lastSerialMsg = msgData.msg;
                        }
                    } catch (e) {
                        // Ignore errors
                    }
                    const now = Date.now() / 1000;
                            if (data.local) {
                                localData = data.local;
                                lastLocalUpdate = data.local.last_update;
                                localAge = Math.round(now - lastLocalUpdate);
                                let localText = `Local GPS<br>Lat: ${data.local.lat.toFixed(5)}, Lon: ${data.local.lon.toFixed(5)}`;
                                if (data.local.altitude !== undefined && data.local.fixes !== undefined) {
                                    localText += `<br>Alt: ${data.local.altitude}m<br>Fixes: ${data.local.fixes}`;
                                }
                                localText += `<br><span id='local-age'>Updated ${localAge} sec ago</span>`;
                                localMarker = createOrUpdateMarker(
                                    localMarker,
                                    data.local.lat,
                                    data.local.lon,
                                    'blue',
                                    localText
                                );
                                map.setView([data.local.lat, data.local.lon]);
                            }
                    if (data.tracker) {
                        trackerData = data.tracker;
                        lastTrackerUpdate = data.tracker.last_update;
                        trackerAge = Math.round(now - lastTrackerUpdate);
                        let popupText = `Tracker GPS<br>Lat: ${data.tracker.lat.toFixed(5)}, Lon: ${data.tracker.lon.toFixed(5)}`;
                        if (data.tracker.altitude !== undefined && data.tracker.fixes !== undefined) {
                            popupText += `<br>Alt: ${data.tracker.altitude}m<br>Fixes: ${data.tracker.fixes}`;
                        }
                        popupText += `<br><span id='tracker-age'>Updated ${trackerAge} sec ago</span>`;
                        trackerMarker = createOrUpdateMarker(
                            trackerMarker,
                            data.tracker.lat,
                            data.tracker.lon,
                            'red',
                            popupText
                        );
                    }
                    // Draw/update line and distance when both markers exist
                    if (localData && trackerData) {
                        const latlngs = [
                            [localData.lat, localData.lon],
                            [trackerData.lat, trackerData.lon]
                        ];
                        const meters = haversineDistance(localData.lat, localData.lon, trackerData.lat, trackerData.lon);
                        const label = `Distance: ${formatDistance(meters)}`;
                            const midLat = (localData.lat + trackerData.lat) / 2;
                            const midLon = (localData.lon + trackerData.lon) / 2;
                        if (routeLine) {
                            routeLine.setLatLngs(latlngs);
                            } else {
                                routeLine = L.polyline(latlngs, {color: 'yellow', weight: 3, opacity: 0.9}).addTo(map);
                            }
                            if (distanceTooltip) {
                                distanceTooltip.setLatLng([midLat, midLon]).setContent(label);
                        } else {
                                distanceTooltip = L.tooltip({permanent: true, direction: 'center', className: 'marker-label route-label'})
                                                    .setContent(label)
                                                    .setLatLng([midLat, midLon])
                                                    .addTo(map);
                        }
                    } else {
                        if (routeLine) {
                            map.removeLayer(routeLine);
                            routeLine = null;
                        }
                            if (distanceTooltip) {
                                map.removeLayer(distanceTooltip);
                                distanceTooltip = null;
                            }
                    }
                } catch (e) {
                    console.error("Failed to fetch GPS data:", e);
                }
            }

            // Haversine distance in meters between two lat/lon points
            function haversineDistance(lat1, lon1, lat2, lon2) {
                const R = 6371000; // meters
                const toRad = (v) => v * Math.PI / 180;
                const dLat = toRad(lat2 - lat1);
                const dLon = toRad(lon2 - lon1);
                const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                          Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) *
                          Math.sin(dLon/2) * Math.sin(dLon/2);
                const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
                return R * c;
            }

            function formatDistance(meters) {
                if (!isFinite(meters)) return '—';
                if (meters < 1000) return `${Math.round(meters)} m`;
                return `${(meters/1000).toFixed(2)} km`;
            }

            function updateAges() {
                const now = Date.now() / 1000;
                if (lastLocalUpdate !== null) {
                    localAge = Math.round(now - lastLocalUpdate);
                    const el = document.getElementById('local-age');
                    if (el) el.textContent = `Updated ${localAge} sec ago`;
                }
                if (lastTrackerUpdate !== null) {
                    trackerAge = Math.round(now - lastTrackerUpdate);
                    const el = document.getElementById('tracker-age');
                    if (el) el.textContent = `Updated ${trackerAge} sec ago`;
                }
            }

            setInterval(updatePosition, 1000);  // Fetch new data every 1 second
            setInterval(updateAges, 1000);      // Update age display every 1 second
        });
        window.onerror = function(message, source, lineno, colno, error) {
            console.log('JS Error:', message, 'at', source, lineno + ':' + colno);
        };
    </script>
</body>
</html>
"""
# Endpoint to serve offline tiles
@app.route('/tiles/<int:z>/<int:x>/<int:y>.png')
def serve_tile(z, x, y):
    tile_root = os.path.join(os.path.dirname(__file__), 'tiles')
    return send_from_directory(tile_root, f'{z}/{x}/{y}.png')

@app.route('/')
def map_view():
    return render_template_string(
        HTML_TEMPLATE,
        local=gps_data["local"],
        tracker=gps_data["tracker"] if gps_data["tracker"] else None,
    )

@app.route('/position')
def get_position():
    return jsonify(gps_data)

# Endpoint to get the last serial message
@app.route('/last_serial_message')
def last_serial_message_api():
    return jsonify(last_serial_message)

# --- SERIAL READER THREAD ---
def read_serial_gps():
    # Accept integers or decimals, optional minus
    num = r"-?\d+(?:\.\d+)?"
    local_full_pattern = re.compile(rf"Local GPS: Latitude:\s*({num})\s+Longitude:\s*({num})\s+Altitude:\s*({num})m\s+Fixes:\s*(\d+)")
    local_simple_pattern = re.compile(rf"Local GPS: Latitude:\s*({num})\s+Longitude:\s*({num})")
    # Accept both 'Tracker GPS' and 'Remote GPS'
    tracker_pattern = re.compile(rf"(?:Tracker|Remote) GPS: Latitude:\s*({num})\s+Longitude:\s*({num})\s+Altitude:\s*({num})m\s+Fixes:\s*(\d+)")
    ser = None
    def find_serial_port():
        try:
            ports = list_ports.comports()
            for p in ports:
                # Prefer obvious USB serial devices first
                if (p.vid is not None) or ('USB' in (p.description or '').upper()):
                    return p.device
            # Fallback to first available port if any
            if ports:
                return ports[0].device
        except Exception:
            pass
        # Fallback patterns for Unix-like systems
        for pattern in ('/dev/tty.usbmodem*', '/dev/tty.usbserial*', '/dev/ttyACM*', '/dev/ttyUSB*'):
            lst = glob.glob(pattern)
            if lst:
                return lst[0]
        return None
    while True:
        if ser is None or not ser.is_open:
            port = find_serial_port()
            if not port:
                print('No serial device found')
                time.sleep(2)
                continue
            try:
                ser = serial.Serial(port, 250000, timeout=1)
                print(f'Serial port connected: {port}')
            except Exception as e:
                print('Serial port error:', e)
                time.sleep(2)
                continue
        try:
            line_bytes = ser.readline()
            if not line_bytes:
                time.sleep(0.1)
                continue
            line = line_bytes.decode('utf-8', errors='replace').strip()
            # Skip lines that are too short to be valid
            if len(line) < 10:
                continue
            global last_serial_message
            last_serial_message['msg'] = line
            local_match_full = local_full_pattern.search(line)
            local_match_simple = local_simple_pattern.search(line)
            tracker_match = tracker_pattern.search(line)
            if local_match_full:
                gps_data["local"] = {
                    "lat": float(local_match_full.group(1)),
                    "lon": float(local_match_full.group(2)),
                    "altitude": float(local_match_full.group(3)),
                    "fixes": int(local_match_full.group(4)),
                    "last_update": pytime.time()
                }
            elif local_match_simple:
                gps_data["local"] = {
                    "lat": float(local_match_simple.group(1)),
                    "lon": float(local_match_simple.group(2)),
                    "last_update": pytime.time()
                }
            elif tracker_match:
                gps_data["tracker"] = {
                    "lat": float(tracker_match.group(1)),
                    "lon": float(tracker_match.group(2)),
                    "altitude": float(tracker_match.group(3)),
                    "fixes": int(tracker_match.group(4)),
                    "last_update": pytime.time()
                }
        except Exception as e:
            # print('Serial read error:', e)
            time.sleep(0.05)
        time.sleep(0.1)

if __name__ == '__main__':
    # Start serial reader once; disable reloader to prevent duplicate processes holding the COM port
    threading.Thread(target=read_serial_gps, daemon=True).start()
    app.run(debug=True, host='0.0.0.0', port=5050, use_reloader=False)