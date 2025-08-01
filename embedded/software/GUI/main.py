# TODO:
# Ability to save location data offline

# RUN: python /Users/lucas/Documents/GitHub/GPS-Tracker-2W/embedded/software/GUI/main.py

from flask import Flask, render_template_string, jsonify, send_from_directory
import folium
import threading
import time
import random
import serial
import io
import re
import glob
import time as pytime

app = Flask(__name__)
gps_data = {"lat": -33.8708, "lon": 151.2073}
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
                maxZoom: 12,
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

            function createOrUpdateMarker(marker, lat, lon, color, label) {
                if (marker) {
                    marker.setLatLng([lat, lon]);
                    marker.setPopupContent(label).openPopup();
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
                    var m = L.marker([lat, lon], {icon: icon}).addTo(map);
                    m.bindPopup(label).openPopup();
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
                        localMarker = createOrUpdateMarker(
                            localMarker,
                            data.local.lat,
                            data.local.lon,
                            'blue',
                            `Local GPS<br>Lat: ${data.local.lat.toFixed(5)}, Lon: ${data.local.lon.toFixed(5)}<br><span id='local-age'>Updated ${localAge} sec ago</span>`
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
                } catch (e) {
                    console.error("Failed to fetch GPS data:", e);
                }
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
    return send_from_directory('tiles', f'{z}/{x}/{y}.png')

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
    local_pattern = re.compile(r"Local GPS: Latitude:\s*(-?\d+\.\d+)\s+Longitude:\s*(-?\d+\.\d+)")
    tracker_pattern = re.compile(r"Tracker GPS: Latitude:\s*(-?\d+\.\d+)\s+Longitude:\s*(-?\d+\.\d+)\s+Altitude:\s*([\d\.]+)m\s+Fixes:\s*(\d+)")
    ser = None
    sio = None
    while True:
        if ser is None or not ser.is_open:
            port_list = glob.glob('/dev/tty.usbmodem*')
            if not port_list:
                print('No serial device found')
                time.sleep(2)
                continue
            port = port_list[0]
            try:
                ser = serial.Serial(port, 250000, timeout=1)
                sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), encoding='utf-8', errors='replace', newline='\n')
                print(f'Serial port connected: {port}')
            except Exception as e:
                print('Serial port error:', e)
                time.sleep(2)
                continue
        try:
            line = sio.readline()
            if not line:
                time.sleep(0.1)
                continue
            line = line.strip()
            # Skip lines that are too short to be valid
            if len(line) < 10:
                continue
            global last_serial_message
            last_serial_message['msg'] = line
            local_match = local_pattern.search(line)
            tracker_match = tracker_pattern.search(line)
            if local_match:
                gps_data["local"] = {
                    "lat": float(local_match.group(1)),
                    "lon": float(local_match.group(2)),
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
            # # try:
            # #     ser.close()
            # # except:
            # #     pass
            # ser = None
            # sio = None
            time.sleep(0.05)
        time.sleep(0.1)

if __name__ == '__main__':
    threading.Thread(target=read_serial_gps, daemon=True).start()
    app.run(debug=True, host='0.0.0.0', port=5050)