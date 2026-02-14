/**
 * Earth Rover Vehicle Control Station - Main Application
 * Modular JavaScript for better organization and performance
 */

// ============================================
// CONFIGURATION
// ============================================
// Get configuration from backend (environment variables) or use defaults
const VCS_CONFIG = {
    rosbridge_url: (window.VCS_BACKEND_CONFIG && window.VCS_BACKEND_CONFIG.rosbridge_url) || 'ws://192.168.1.7:9090',
    video_server_url: (window.VCS_BACKEND_CONFIG && window.VCS_BACKEND_CONFIG.video_server_url) || 'http://192.168.1.7:8080',
    update_intervals: {
        disk_space: 30000,      // 30 seconds
        recording_status: 10000, // 10 seconds
        ros_nodes: 5000,         // 5 seconds
        instruments: 50,         // 50ms (~20 FPS)
        laserscan: 100,          // 100ms (10 FPS)
        velodyne: 200           // 200ms (5 FPS)
    }
};

console.log('VCS Configuration loaded:', {
    rosbridge_url: VCS_CONFIG.rosbridge_url,
    video_server_url: VCS_CONFIG.video_server_url
});

// ============================================
// ROS CONNECTION
// ============================================
const ros = new ROSLIB.Ros({ url: VCS_CONFIG.rosbridge_url });

ros.on('connection', function() {
    console.log('Connected to ROS websocket server');
    updateStatus('Connected', 'connected');
});

ros.on('error', function(error) {
    console.error('ROS connection error:', error);
    updateStatus('Error', 'disconnected');
});

ros.on('close', function() {
    console.log('ROS connection closed');
    updateStatus('Disconnected', 'disconnected');
});

function updateStatus(text, className) {
    const statusEl = document.getElementById('ros-status');
    statusEl.textContent = text;
    statusEl.className = 'status-indicator status-' + className;
}

// ============================================
// UTILITY FUNCTIONS
// ============================================
function getCookie(name) {
    if (name === 'csrftoken') {
        const metaToken = document.querySelector('meta[name="csrf-token"]');
        if (metaToken) return metaToken.getAttribute('content');
    }
    
    let cookieValue = null;
    if (document.cookie && document.cookie !== '') {
        const cookies = document.cookie.split(';');
        for (let i = 0; i < cookies.length; i++) {
            const cookie = cookies[i].trim();
            if (cookie.substring(0, name.length + 1) === (name + '=')) {
                cookieValue = decodeURIComponent(cookie.substring(name.length + 1));
                break;
            }
        }
    }
    return cookieValue;
}

function throttle(func, delay) {
    let lastCall = 0;
    return function(...args) {
        const now = Date.now();
        if (now - lastCall >= delay) {
            lastCall = now;
            return func.apply(this, args);
        }
    };
}

// Section toggle functionality
function toggleSection(sectionId) {
    const content = document.getElementById(sectionId + '-content');
    const toggle = event.currentTarget.querySelector('.section-toggle');
    
    if (content.style.maxHeight && content.style.maxHeight !== '0px') {
        content.style.maxHeight = '0px';
        event.currentTarget.parentElement.classList.add('section-collapsed');
    } else {
        content.style.maxHeight = content.scrollHeight + 'px';
        event.currentTarget.parentElement.classList.remove('section-collapsed');
    }
}

// Initialize all sections as expanded
window.addEventListener('DOMContentLoaded', function() {
    ['instruments', 'diagnostics', 'cameras', 'sensors', 'pointcloud', 'aircraft'].forEach(function(section) {
        const content = document.getElementById(section + '-content');
        if (content) {
            content.style.maxHeight = content.scrollHeight + 'px';
        }
    });
    // Invalidate aircraft map size after layout settles (Leaflet needs this)
    setTimeout(function() {
        if (typeof aircraftMap !== 'undefined' && aircraftMap) {
            aircraftMap.invalidateSize();
        }
    }, 500);
});

// ============================================
// DISK SPACE MONITORING
// ============================================
function updateDiskSpace() {
    fetch('/api/disk-space/')
        .then(response => response.json())
        .then(data => {
            if (data.status === 'ok') {
                document.getElementById('disk-used').textContent = data.used_gb + ' GB';
                document.getElementById('disk-free').textContent = data.free_gb + ' GB';
                document.getElementById('disk-total').textContent = data.total_gb + ' GB';
                
                const diskBar = document.getElementById('disk-bar');
                diskBar.style.width = data.used_percent + '%';
                
                diskBar.className = 'disk-bar';
                if (data.used_percent < 70) {
                    diskBar.classList.add('low');
                } else if (data.used_percent < 90) {
                    diskBar.classList.add('medium');
                } else {
                    diskBar.classList.add('high');
                }
            }
        })
        .catch(error => console.error('Error fetching disk space:', error));
}

updateDiskSpace();
setInterval(updateDiskSpace, VCS_CONFIG.update_intervals.disk_space);

// ============================================
// RECORDING CONTROLS
// ============================================
function startRecording() {
    document.getElementById('start-recording-btn').disabled = true;
    
    fetch('/api/recording/start/', {
        method: 'POST',
        headers: { 'X-CSRFToken': getCookie('csrftoken') }
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            updateRecordingUI(true);
            console.log('Recording started:', data.output_file);
        } else {
            alert('Error: ' + data.message);
            document.getElementById('start-recording-btn').disabled = false;
        }
    })
    .catch(error => {
        console.error('Recording start failed:', error);
        document.getElementById('start-recording-btn').disabled = false;
    });
}

function stopRecording() {
    document.getElementById('stop-recording-btn').disabled = true;
    
    fetch('/api/recording/stop/', {
        method: 'POST',
        headers: { 'X-CSRFToken': getCookie('csrftoken') }
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'ok') {
            updateRecordingUI(false);
            console.log('Recording stopped:', data.output_file);
        } else {
            alert('Error: ' + data.message);
            document.getElementById('stop-recording-btn').disabled = false;
        }
    })
    .catch(error => {
        console.error('Recording stop failed:', error);
        document.getElementById('stop-recording-btn').disabled = false;
    });
}

function updateRecordingStatus() {
    fetch('/api/recording/status/')
        .then(response => response.json())
        .then(data => {
            if (data.status === 'ok') {
                updateRecordingUI(data.recording);
            }
        })
        .catch(error => console.error('Error fetching recording status:', error));
}

function updateRecordingUI(isRecording) {
    const startBtn = document.getElementById('start-recording-btn');
    const stopBtn = document.getElementById('stop-recording-btn');
    const statusEl = document.getElementById('recording-status');
    
    startBtn.disabled = isRecording;
    stopBtn.disabled = !isRecording;
    statusEl.textContent = isRecording ? 'Recording' : 'Idle';
    statusEl.className = 'recording-status ' + (isRecording ? 'active' : 'idle');
}

updateRecordingStatus();
setInterval(updateRecordingStatus, VCS_CONFIG.update_intervals.recording_status);

// ============================================
// ROS DIAGNOSTICS (COMPACT)
// ============================================
function escapeHtml(text) {
    const map = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#039;' };
    return text.replace(/[&<>"']/g, m => map[m]);
}

// Update ROS Nodes (compact list)
function updateRosNodes() {
    fetch('/api/ros-nodes/')
        .then(response => response.json())
        .then(data => {
            const container = document.getElementById('ros-nodes-container');
            const countBadge = document.getElementById('ros-nodes-count');
            const versionDisplay = document.getElementById('ros-version-display');
            
            if (data.status === 'ok' && data.nodes && data.nodes.length > 0) {
                countBadge.textContent = data.count;
                versionDisplay.textContent = data.ros_version || 'ROS';
                
                // Compact text list
                let html = '<div style="color: #b0b0b0;">';
                data.nodes.forEach((node, idx) => {
                    html += `<div style="padding: 2px 8px; border-left: 2px solid #4a9eff; margin-bottom: 2px; background: rgba(0,0,0,0.2);">
                        <span style="color: #00ff00; margin-right: 6px;">●</span>${escapeHtml(node)}
                    </div>`;
                });
                html += '</div>';
                container.innerHTML = html;
            } else {
                countBadge.textContent = '0';
                versionDisplay.textContent = data.ros_version || '--';
                container.innerHTML = '<div class="text-center text-muted">No nodes detected</div>';
            }
        })
        .catch(error => {
            console.error('Error fetching ROS nodes:', error);
            document.getElementById('ros-nodes-container').innerHTML = 
                '<div class="text-center text-muted">Error loading nodes</div>';
        });
}

// Update ROS Topics (compact list) - using ROSBridge
function updateRosTopics() {
    const container = document.getElementById('ros-topics-container');
    const countBadge = document.getElementById('ros-topics-count');
    
    if (!ros || !ros.isConnected) {
        container.innerHTML = '<div class="text-center text-muted">ROSBridge disconnected</div>';
        countBadge.textContent = '0';
        return;
    }
    
    // Get topics from ROSBridge
    ros.getTopics(function(topics) {
        if (topics && topics.topics && topics.topics.length > 0) {
            countBadge.textContent = topics.topics.length;
            
            // Compact text list
            let html = '<div style="color: #b0b0b0;">';
            topics.topics.forEach((topic, idx) => {
                const topicType = topics.types[idx] || 'unknown';
                html += `<div style="padding: 2px 8px; border-left: 2px solid #4a9eff; margin-bottom: 2px; background: rgba(0,0,0,0.2);">
                    <span style="color: #4a9eff; margin-right: 6px;">▸</span>${escapeHtml(topic)}
                    <span style="color: #666; font-size: 0.9em; margin-left: 8px;">[${escapeHtml(topicType)}]</span>
                </div>`;
            });
            html += '</div>';
            container.innerHTML = html;
        } else {
            countBadge.textContent = '0';
            container.innerHTML = '<div class="text-center text-muted">No topics detected</div>';
        }
    }, function(error) {
        console.error('Error fetching ROS topics:', error);
        container.innerHTML = '<div class="text-center text-muted">Error loading topics</div>';
        countBadge.textContent = '0';
    });
}

updateRosNodes();
updateRosTopics();
setInterval(updateRosNodes, VCS_CONFIG.update_intervals.ros_nodes);
setInterval(updateRosTopics, VCS_CONFIG.update_intervals.ros_nodes);

// ============================================
// CAMERA MANAGEMENT
// ============================================
const cameras = {
    'left-stereo': { topic: '/stereo/left/image_raw', active: false, quality: 'medium' },
    'right-stereo': { topic: '/stereo/right/image_raw', active: false, quality: 'medium' },
    'fisheye': { topic: '/camera/fisheye1/image_raw', active: false, quality: 'medium' },
    'spectrometer-plot': { topic: '/spectrometer_plot', active: false, quality: 'medium' }
};

const qualitySettings = {
    'high': '?quality=80&width=1280&height=720',
    'medium': '?quality=60&width=640&height=480',
    'low': '?quality=40&width=320&height=240',
    'snapshot': '?quality=60&width=640&height=480'
};

function getCameraStreamUrl(cameraId) {
    const camera = cameras[cameraId];
    const quality = qualitySettings[camera.quality] || qualitySettings['medium'];
    
    if (camera.quality === 'snapshot') {
        return `${VCS_CONFIG.video_server_url}/snapshot${quality}&topic=${camera.topic}`;
    } else {
        return `${VCS_CONFIG.video_server_url}/stream${quality}&topic=${camera.topic}`;
    }
}

function startCamera(cameraId) {
    const camera = cameras[cameraId];
    if (camera.active) return;
    
    const container = document.getElementById(cameraId + '-container');
    const statusEl = document.getElementById(cameraId + '-status');
    
    statusEl.textContent = 'Loading...';
    statusEl.className = 'camera-status loading';
    
    const img = document.createElement('img');
    img.alt = cameraId + ' Camera';
    img.style.width = '100%';
    
    img.onload = function() {
        statusEl.textContent = 'Live';
        statusEl.className = 'camera-status live';
        camera.active = true;
        updateCameraButton(cameraId, true);
    };
    
    img.onerror = function() {
        statusEl.textContent = 'Error';
        statusEl.className = 'camera-status paused';
    };
    
    img.src = getCameraStreamUrl(cameraId);
    container.innerHTML = '';
    container.appendChild(img);
    camera.img = img;
}

function stopCamera(cameraId) {
    const camera = cameras[cameraId];
    if (!camera.active) return;
    
    const container = document.getElementById(cameraId + '-container');
    const statusEl = document.getElementById(cameraId + '-status');
    
    container.innerHTML = `<div class="camera-placeholder" onclick="startCamera('${cameraId}')">
        <div class="camera-placeholder-icon">📷</div>
        <div>Click to Start</div>
    </div>`;
    
    camera.img = null;
    camera.active = false;
    statusEl.textContent = 'Paused';
    statusEl.className = 'camera-status paused';
    updateCameraButton(cameraId, false);
}

function toggleCamera(cameraId) {
    if (cameras[cameraId].active) {
        stopCamera(cameraId);
    } else {
        startCamera(cameraId);
    }
}

function updateCameraQuality(cameraId) {
    const select = document.getElementById(cameraId + '-quality');
    cameras[cameraId].quality = select.value;
    
    if (cameras[cameraId].active) {
        stopCamera(cameraId);
        setTimeout(() => startCamera(cameraId), 100);
    }
}

function updateCameraButton(cameraId, isActive) {
    const button = document.querySelector(`#${cameraId}-container`).parentElement.querySelector('.camera-btn');
    if (button) {
        button.textContent = isActive ? '⏸ Pause' : '▶ Play';
    }
}

// ============================================
// UNIFIED SITUATIONAL AWARENESS MAP
// (Rover position + ADS-B aircraft on one map)
// ============================================

const aircraftMap = (function() {
    const mapEl = document.getElementById('aircraft-map');
    if (!mapEl) return null;
    // Default center: Phoenix metro area
    const m = L.map('aircraft-map').setView([33.45, -112.07], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap'
    }).addTo(m);
    return m;
})();

// --- Rover position on the unified map ---
const roverIcon = L.divIcon({
    className: 'rover-marker-icon',
    html: '<div style="width: 14px; height: 14px; background: #00e676; border: 2px solid #fff; border-radius: 50%; box-shadow: 0 0 8px rgba(0,230,118,0.7);"></div>',
    iconSize: [14, 14],
    iconAnchor: [7, 7],
    popupAnchor: [0, -10]
});
let roverMarker = null;
const roverPathCoords = [];
let roverPolyline = null;
let roverMapCentered = false;
let lastRoverGpsUpdate = 0;

if (aircraftMap) {
    roverPolyline = L.polyline(roverPathCoords, {
        color: '#00e676',
        weight: 3,
        opacity: 0.7
    }).addTo(aircraftMap);
}

// Subscribe to mavros GPS for rover position
const roverGpsListener = new ROSLIB.Topic({
    ros: ros,
    name: '/mavros/global_position/raw/fix',
    messageType: 'sensor_msgs/NavSatFix'
});

roverGpsListener.subscribe(function(message) {
    if (!aircraftMap) return;

    const now = Date.now();
    if (now - lastRoverGpsUpdate < 2000) return;  // Throttle to 0.5 Hz
    lastRoverGpsUpdate = now;

    const lat = message.latitude;
    const lon = message.longitude;

    if (lat === 0 && lon === 0) return;

    // Update rover path
    roverPathCoords.push([lat, lon]);
    roverPolyline.setLatLngs(roverPathCoords);

    // Update or create rover marker
    if (roverMarker) {
        roverMarker.setLatLng([lat, lon]);
    } else {
        roverMarker = L.marker([lat, lon], {
            icon: roverIcon,
            zIndexOffset: 2000  // Above aircraft markers
        }).addTo(aircraftMap);
        roverMarker.bindPopup(
            '<div style="font-family: monospace; font-size: 12px;">' +
            '<div style="color: #00e676; font-weight: bold;">Earth Rover</div>' +
            '</div>',
            { className: 'aircraft-popup', closeButton: false }
        );
    }

    // Update rover popup with current coords
    roverMarker.setPopupContent(
        '<div style="font-family: monospace; font-size: 12px;">' +
        '<div style="color: #00e676; font-weight: bold; margin-bottom: 4px;">Earth Rover</div>' +
        '<div style="color: #b0b0b0;">' + lat.toFixed(6) + ', ' + lon.toFixed(6) + '</div>' +
        '</div>'
    );

    // Update position display under the map
    const posDisplay = document.getElementById('rover-position-display');
    if (posDisplay) {
        posDisplay.textContent = lat.toFixed(5) + ', ' + lon.toFixed(5);
    }

    // Center map on rover on first fix (if no aircraft has centered it yet)
    if (!roverMapCentered) {
        aircraftMap.setView([lat, lon], 11);
        roverMapCentered = true;
    }
});

// --- ADS-B estimated receiver position ---
const estimatedPosIcon = L.divIcon({
    className: 'estimated-pos-icon',
    html: '<div style="width: 20px; height: 20px; border: 2px dashed #ff9800; border-radius: 50%; background: rgba(255,152,0,0.15); box-shadow: 0 0 12px rgba(255,152,0,0.4);"></div>',
    iconSize: [20, 20],
    iconAnchor: [10, 10],
    popupAnchor: [0, -12]
});
let estimatedPosMarker = null;
let estimatedPosCircle = null;

const estimatedPosListener = new ROSLIB.Topic({
    ros: ros,
    name: '/adsb/rtl_adsb_decoder_node/estimated_position',
    messageType: 'std_msgs/String'
});

estimatedPosListener.subscribe(function(message) {
    if (!aircraftMap) return;
    try {
        const data = JSON.parse(message.data);
        const lat = data.lat;
        const lon = data.lon;
        const count = data.aircraft_used;
        const confidence = data.confidence;

        if (!lat || !lon) return;

        // Estimate accuracy radius: lower confidence = larger circle
        // Rough heuristic: at confidence=1.0 → ~10km, at 0.0 → ~100km
        const radiusKm = 10 + (1.0 - confidence) * 90;
        const radiusM = radiusKm * 1000;

        // Update or create marker
        if (estimatedPosMarker) {
            estimatedPosMarker.setLatLng([lat, lon]);
            estimatedPosCircle.setLatLng([lat, lon]);
            estimatedPosCircle.setRadius(radiusM);
        } else {
            estimatedPosMarker = L.marker([lat, lon], {
                icon: estimatedPosIcon,
                zIndexOffset: 1500
            }).addTo(aircraftMap);

            estimatedPosCircle = L.circle([lat, lon], {
                radius: radiusM,
                color: '#ff9800',
                weight: 1,
                dashArray: '6 4',
                fillColor: '#ff9800',
                fillOpacity: 0.05
            }).addTo(aircraftMap);
        }

        // Update popup
        estimatedPosMarker.bindPopup(
            '<div style="font-family: monospace; font-size: 12px;">' +
            '<div style="color: #ff9800; font-weight: bold; margin-bottom: 4px;">Estimated RTL-SDR Position</div>' +
            '<div style="color: #b0b0b0;">' + lat.toFixed(5) + ', ' + lon.toFixed(5) + '</div>' +
            '<div style="color: #888; margin-top: 4px;">From ' + count + ' aircraft</div>' +
            '<div style="color: #888;">Confidence: ' + Math.round(confidence * 100) + '%</div>' +
            '<div style="color: #888;">Est. radius: ~' + radiusKm.toFixed(0) + ' km</div>' +
            '</div>',
            { className: 'aircraft-popup', closeButton: false }
        );

        // Update display under the map
        const estDisplay = document.getElementById('estimated-pos-display');
        if (estDisplay) {
            estDisplay.textContent = lat.toFixed(5) + ', ' + lon.toFixed(5) +
                ' (' + Math.round(confidence * 100) + '%, ' + count + ' ac)';
        }

    } catch (e) {
        console.error('Error parsing estimated position:', e);
    }
});

// State for tracked aircraft
const aircraftMarkers = {};   // ICAO -> { marker, data, lastSeen }
const aircraftData = {};       // ICAO -> { callsign, alt, speed, heading, lat, lon }
const AIRCRAFT_STALE_MS = 60000;  // Remove after 60s (1 minute) without update
let aircraftMapCentered = false;

// Create airplane icon for a given heading
function createAircraftIcon(heading, isSelected) {
    // ADS-B heading: 0°=N, 90°=E, 180°=S, 270°=W  (CW from true north)
    // The ✈ glyph (U+2708) points east (right) by default, so subtract
    // 90° to align it: heading 0° → rotate(-90°) → points north.
    const rotation = (heading || 0) - 90;
    const color = isSelected ? '#ff4444' : '#00e5ff';
    return L.divIcon({
        className: 'aircraft-marker-icon',
        html: `<div style="transform: rotate(${rotation}deg); color: ${color}; font-size: 22px; text-align: center; line-height: 1;">&#9992;</div>`,
        iconSize: [24, 24],
        iconAnchor: [12, 12],
        popupAnchor: [0, -14]
    });
}

// Build popup content for an aircraft
function aircraftPopupContent(icao, data) {
    let html = `<div style="min-width: 140px;">`;
    html += `<div style="color: #4a9eff; font-weight: bold; font-size: 13px; margin-bottom: 4px;">${icao}</div>`;
    if (data.callsign) {
        html += `<div style="color: #00e676; font-size: 14px; font-weight: bold; margin-bottom: 4px;">${data.callsign}</div>`;
    }
    if (data.alt != null) {
        html += `<div>ALT: <span style="color: #ffab40;">${data.alt.toFixed(0)} ft</span></div>`;
    }
    if (data.speed != null) {
        html += `<div>SPD: <span style="color: #e0e0e0;">${data.speed.toFixed(0)} kts</span></div>`;
    }
    if (data.heading != null) {
        html += `<div>HDG: <span style="color: #b0b0b0;">${data.heading.toFixed(0)}&deg;</span></div>`;
    }
    if (data.lat != null && data.lon != null) {
        html += `<div style="color: #888; font-size: 10px; margin-top: 4px;">${data.lat.toFixed(4)}, ${data.lon.toFixed(4)}</div>`;
    }
    html += `</div>`;
    return html;
}

// Subscribe to individual aircraft positions (NavSatFix)
const aircraftPositionListener = new ROSLIB.Topic({
    ros: ros,
    name: '/adsb/rtl_adsb_decoder_node/aircraft',
    messageType: 'sensor_msgs/NavSatFix'
});

aircraftPositionListener.subscribe(function(message) {
    if (!aircraftMap) return;

    // Extract ICAO from frame_id (format: "aircraft_ICAO")
    const frameId = message.header.frame_id || '';
    const icao = frameId.replace('aircraft_', '').toUpperCase();
    if (!icao) return;

    const lat = message.latitude;
    const lon = message.longitude;
    const alt = message.altitude;

    if (lat === 0 && lon === 0) return;

    // Update aircraft data
    if (!aircraftData[icao]) {
        aircraftData[icao] = {};
    }
    aircraftData[icao].lat = lat;
    aircraftData[icao].lon = lon;
    aircraftData[icao].alt = alt;

    const heading = aircraftData[icao].heading || 0;

    // Update or create marker
    if (aircraftMarkers[icao]) {
        aircraftMarkers[icao].marker.setLatLng([lat, lon]);
        aircraftMarkers[icao].marker.setIcon(createAircraftIcon(heading, false));
        aircraftMarkers[icao].marker.setPopupContent(aircraftPopupContent(icao, aircraftData[icao]));
        aircraftMarkers[icao].lastSeen = Date.now();
    } else {
        const marker = L.marker([lat, lon], {
            icon: createAircraftIcon(heading, false),
            zIndexOffset: 1000
        }).addTo(aircraftMap);
        marker.bindPopup(aircraftPopupContent(icao, aircraftData[icao]), {
            className: 'aircraft-popup',
            closeButton: false,
            autoPan: false
        });
        aircraftMarkers[icao] = { marker: marker, lastSeen: Date.now() };
    }

    // Center map on first aircraft only if rover hasn't centered it yet
    if (!aircraftMapCentered && !roverMapCentered) {
        aircraftMap.setView([lat, lon], 9);
        aircraftMapCentered = true;
    }
});

// Subscribe to aircraft list summary (String)
const aircraftListListener = new ROSLIB.Topic({
    ros: ros,
    name: '/adsb/rtl_adsb_decoder_node/aircraft_list',
    messageType: 'std_msgs/String'
});

let lastAircraftListUpdate = 0;

aircraftListListener.subscribe(function(message) {
    const now = Date.now();
    if (now - lastAircraftListUpdate < 2000) return;  // Throttle to 0.5 Hz
    lastAircraftListUpdate = now;

    const data = message.data || '';

    // Parse: "Tracked Aircraft (N): ICAO1 (CALL) @ALTft SPDkts hdg HDG°, ICAO2 ..."
    const countMatch = data.match(/Tracked Aircraft \((\d+)\)/);
    const count = countMatch ? parseInt(countMatch[1]) : 0;

    // Update count badges
    const countBadge = document.getElementById('aircraft-count-badge');
    const mapCount = document.getElementById('aircraft-map-count');
    if (countBadge) countBadge.textContent = count;
    if (mapCount) mapCount.textContent = count;

    // Parse individual aircraft entries
    const listPart = data.replace(/Tracked Aircraft \(\d+\):\s*/, '');
    const entries = listPart.split(',').map(e => e.trim()).filter(e => e.length > 0);

    // Parse each entry and update aircraftData
    const seenIcaos = new Set();
    entries.forEach(function(entry) {
        // Format: "ICAO (CALLSIGN) @ALTft SPEEDkts hdg HEADING°"
        const icaoMatch = entry.match(/^([A-F0-9]{6})/i);
        if (!icaoMatch) return;

        const icao = icaoMatch[1].toUpperCase();
        seenIcaos.add(icao);
        if (!aircraftData[icao]) aircraftData[icao] = {};
        aircraftData[icao]._listAge = Date.now();

        const callMatch = entry.match(/\(([^)]+)\)/);
        if (callMatch) aircraftData[icao].callsign = callMatch[1];

        const altMatch = entry.match(/@([\d.]+)ft/);
        if (altMatch) aircraftData[icao].alt = parseFloat(altMatch[1]);

        const spdMatch = entry.match(/([\d.]+)kts/);
        if (spdMatch) aircraftData[icao].speed = parseFloat(spdMatch[1]);

        const hdgMatch = entry.match(/hdg\s+([\d.]+)/);
        if (hdgMatch) {
            aircraftData[icao].heading = parseFloat(hdgMatch[1]);
            // Update marker icon rotation if marker exists
            if (aircraftMarkers[icao]) {
                aircraftMarkers[icao].marker.setIcon(createAircraftIcon(aircraftData[icao].heading, false));
            }
        }
    });

    // Remove map markers for aircraft the ROS node has already evicted
    // (they no longer appear in the aircraft_list).
    Object.keys(aircraftMarkers).forEach(function(icao) {
        if (!seenIcaos.has(icao)) {
            if (aircraftMap) aircraftMap.removeLayer(aircraftMarkers[icao].marker);
            delete aircraftMarkers[icao];
            delete aircraftData[icao];
        }
    });

    // Update aircraft list UI
    updateAircraftListUI(entries);
});

function updateAircraftListUI(entries) {
    const container = document.getElementById('aircraft-list-container');
    if (!container) return;

    if (entries.length === 0) {
        container.innerHTML = '<div class="text-center text-muted" style="padding: 40px 0;">No aircraft detected</div>';
        return;
    }

    let html = '<table class="aircraft-table"><thead><tr>';
    html += '<th>ICAO</th><th>Callsign</th><th>Alt (ft)</th><th>Speed</th><th>Hdg</th>';
    html += '</tr></thead><tbody>';

    entries.forEach(function(entry) {
        const icaoMatch = entry.match(/^([A-F0-9]{6})/i);
        if (!icaoMatch) return;
        const icao = icaoMatch[1].toUpperCase();
        const d = aircraftData[icao] || {};

        html += '<tr>';
        html += `<td class="icao-cell">${icao}</td>`;
        html += `<td class="callsign-cell">${d.callsign || '--'}</td>`;
        html += `<td class="alt-cell">${d.alt != null ? d.alt.toFixed(0) : '--'}</td>`;
        html += `<td class="speed-cell">${d.speed != null ? d.speed.toFixed(0) + ' kts' : '--'}</td>`;
        html += `<td class="heading-cell">${d.heading != null ? d.heading.toFixed(0) + '&deg;' : '--'}</td>`;
        html += '</tr>';
    });

    html += '</tbody></table>';
    container.innerHTML = html;
}

// Periodic cleanup of stale aircraft markers (every 10 seconds)
setInterval(function() {
    const now = Date.now();
    // Remove stale markers (no position update in 60 seconds)
    Object.keys(aircraftMarkers).forEach(function(icao) {
        if (now - aircraftMarkers[icao].lastSeen > AIRCRAFT_STALE_MS) {
            if (aircraftMap) aircraftMap.removeLayer(aircraftMarkers[icao].marker);
            delete aircraftMarkers[icao];
            delete aircraftData[icao];
        }
    });
    // Also clean up orphaned aircraftData entries that have no marker
    // and haven't been refreshed via the aircraft_list topic
    Object.keys(aircraftData).forEach(function(icao) {
        if (!aircraftMarkers[icao] && aircraftData[icao]._listAge != null) {
            if (now - aircraftData[icao]._listAge > AIRCRAFT_STALE_MS) {
                delete aircraftData[icao];
            }
        }
    });
}, 10000);

// ============================================
// COMPACT INSTRUMENTS (Simplified versions)
// ============================================
let currentPitch = 0, currentRoll = 0, currentHeading = 0;
let currentAltitude = 0, currentLaserRange = 0;
let instrumentsNeedUpdate = true;

// Subscribe to IMU data
const imuListener = new ROSLIB.Topic({
    ros: ros,
    name: '/mavros/imu/data',
    messageType: 'sensor_msgs/Imu'
});

imuListener.subscribe(throttle(function(message) {
    const q = message.orientation;
    
    // Convert quaternion to Euler
    currentRoll = Math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y)) * (180/Math.PI);
    const sinp = 2*(q.w*q.y - q.z*q.x);
    currentPitch = (Math.abs(sinp) >= 1 ? Math.sign(sinp)*90 : Math.asin(sinp)) * (180/Math.PI);
    const yaw = Math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)) * (180/Math.PI);
    currentHeading = (yaw + 360) % 360;
    
    document.getElementById('pitch-value').textContent = `P: ${currentPitch.toFixed(1)}° R: ${currentRoll.toFixed(1)}°`;
    document.getElementById('heading-value').textContent = currentHeading.toFixed(1) + '°';
    instrumentsNeedUpdate = true;
}, 100));

// Subscribe to altitude
const altitudeListener = new ROSLIB.Topic({
    ros: ros,
    name: '/mavros/global_position/rel_alt',
    messageType: 'std_msgs/Float64'
});

altitudeListener.subscribe(throttle(function(message) {
    currentAltitude = message.data;
    document.getElementById('altitude-value').textContent = currentAltitude.toFixed(1) + ' m';
    instrumentsNeedUpdate = true;
}, 100));

// Subscribe to laser range data from serial port
const laserRangeListener = new ROSLIB.Topic({
    ros: ros,
    name: '/serial_data',
    messageType: 'std_msgs/String'
});

laserRangeListener.subscribe(throttle(function(message) {
    try {
        // Parse the string data to float (laser distance in meters)
        const rangeValue = parseFloat(message.data);
        if (!isNaN(rangeValue)) {
            currentLaserRange = rangeValue;
            document.getElementById('laser-value').textContent = currentLaserRange.toFixed(2) + ' m';
            instrumentsNeedUpdate = true;
        }
    } catch (error) {
        console.error('Error parsing laser range data:', error);
    }
}, 100));

// ============================================
// COMPACT INSTRUMENT DRAWING
// ============================================

// Get canvas contexts
const attitudeCanvas = document.getElementById('attitude-mini');
const headingCanvas = document.getElementById('heading-mini');
const altitudeCanvas = document.getElementById('altitude-mini');
const laserCanvas = document.getElementById('laser-mini');

const attitudeCtx = attitudeCanvas ? attitudeCanvas.getContext('2d') : null;
const headingCtx = headingCanvas ? headingCanvas.getContext('2d') : null;
const altitudeCtx = altitudeCanvas ? altitudeCanvas.getContext('2d') : null;
const laserCtx = laserCanvas ? laserCanvas.getContext('2d') : null;

// Draw attitude indicator (artificial horizon)
function drawAttitude(ctx, pitch, roll) {
    if (!ctx) return;
    
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.min(width, height) / 2 - 10;
    
    ctx.clearRect(0, 0, width, height);
    
    // Save context
    ctx.save();
    
    // Create circular clipping region
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.clip();
    
    // Rotate and translate based on roll and pitch
    ctx.translate(cx, cy);
    ctx.rotate(-roll * Math.PI / 180);
    
    const pitchPixels = (pitch / 90) * radius;
    
    // Draw sky (blue)
    ctx.fillStyle = '#4a90e2';
    ctx.fillRect(-radius * 2, -radius * 2 - pitchPixels, radius * 4, radius * 2);
    
    // Draw ground (brown)
    ctx.fillStyle = '#8b4513';
    ctx.fillRect(-radius * 2, -pitchPixels, radius * 4, radius * 2);
    
    // Draw horizon line
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-radius * 1.5, -pitchPixels);
    ctx.lineTo(radius * 1.5, -pitchPixels);
    ctx.stroke();
    
    // Draw pitch ladder
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.font = '10px monospace';
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    
    for (let angle = -30; angle <= 30; angle += 10) {
        if (angle === 0) continue;
        const y = -pitchPixels - (angle / 90) * radius;
        const lineWidth = angle % 20 === 0 ? 40 : 20;
        
        ctx.beginPath();
        ctx.moveTo(-lineWidth, y);
        ctx.lineTo(lineWidth, y);
        ctx.stroke();
        
        if (angle % 20 === 0) {
            ctx.fillText(angle.toString(), lineWidth + 15, y + 4);
            ctx.fillText(angle.toString(), -lineWidth - 15, y + 4);
        }
    }
    
    ctx.restore();
    
    // Draw center marker (fixed)
    ctx.strokeStyle = '#ffff00';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(cx - 30, cy);
    ctx.lineTo(cx - 10, cy);
    ctx.moveTo(cx + 10, cy);
    ctx.lineTo(cx + 30, cy);
    ctx.stroke();
    
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, 2 * Math.PI);
    ctx.fillStyle = '#ffff00';
    ctx.fill();
    
    // Draw outer ring
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.stroke();
}

// Draw heading indicator (compass rose)
function drawHeading(ctx, heading) {
    if (!ctx) return;
    
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.min(width, height) / 2 - 10;
    
    ctx.clearRect(0, 0, width, height);
    
    // Draw dark background circle
    ctx.fillStyle = '#1a1a2e';
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.fill();
    
    // Draw outer blue ring
    ctx.strokeStyle = '#4a9eff';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.stroke();
    
    // Draw inner compass circle
    ctx.strokeStyle = '#6ab7ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, radius - 8, 0, 2 * Math.PI);
    ctx.stroke();
    
    // Rotate the compass card
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-heading * Math.PI / 180);
    
    // Draw tick marks and labels
    for (let i = 0; i < 72; i++) {
        const angle = i * 5;
        ctx.save();
        ctx.rotate(angle * Math.PI / 180);
        
        const isMajor = angle % 30 === 0;
        const isMedium = angle % 10 === 0;
        
        // Draw tick marks
        if (isMajor) {
            // Major tick every 30 degrees
            ctx.strokeStyle = '#4a9eff';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(0, -radius + 12);
            ctx.lineTo(0, -radius + 28);
            ctx.stroke();
        } else if (isMedium) {
            // Medium tick every 10 degrees
            ctx.strokeStyle = '#4a9eff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(0, -radius + 12);
            ctx.lineTo(0, -radius + 22);
            ctx.stroke();
        } else {
            // Minor tick every 5 degrees
            ctx.strokeStyle = '#4a9eff';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(0, -radius + 12);
            ctx.lineTo(0, -radius + 18);
            ctx.stroke();
        }
        
        // Draw labels
        if (angle % 90 === 0) {
            // Cardinal directions
            const labels = ['N', 'E', 'S', 'W'];
            const cardinalLabel = labels[angle / 90];
            
            ctx.save();
            ctx.rotate(-angle * Math.PI / 180 + heading * Math.PI / 180);
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 28px Arial';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(cardinalLabel, 0, -radius + 48);
            ctx.restore();
        } else if (angle % 30 === 0) {
            // Degree numbers every 30 degrees
            const degreeLabels = {
                30: '03', 60: '06', 120: '12', 150: '15',
                210: '21', 240: '24', 300: '30', 330: '33'
            };
            const label = degreeLabels[angle];
            
            ctx.save();
            ctx.rotate(-angle * Math.PI / 180 + heading * Math.PI / 180);
            ctx.fillStyle = '#4a9eff';
            ctx.font = 'bold 18px Arial';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(label, 0, -radius + 45);
            ctx.restore();
        }
        
        ctx.restore();
    }
    
    ctx.restore();
    
    // Draw fixed white triangle pointer at top (lubber line)
    ctx.fillStyle = '#ffffff';
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy - radius + 5);
    ctx.lineTo(cx - 12, cy - radius + 25);
    ctx.lineTo(cx + 12, cy - radius + 25);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    
    // Draw center dot
    ctx.fillStyle = '#4a9eff';
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
    ctx.fill();
    
    // Draw digital heading readout box at bottom
    const boxWidth = 70;
    const boxHeight = 22;
    const boxY = cy + radius - 30;
    
    ctx.fillStyle = 'rgba(0, 0, 0, 0.9)';
    ctx.fillRect(cx - boxWidth/2, boxY, boxWidth, boxHeight);
    
    ctx.strokeStyle = '#4a9eff';
    ctx.lineWidth = 2;
    ctx.strokeRect(cx - boxWidth/2, boxY, boxWidth, boxHeight);
    
    // Display heading value
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 16px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const hdgText = Math.round(heading).toString().padStart(3, '0') + '°T';
    ctx.fillText(hdgText, cx, boxY + boxHeight/2);
}

// Draw altitude indicator - Vertical tape style
function drawAltitude(ctx, altitude) {
    if (!ctx) return;
    
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.min(width, height) / 2 - 10;
    
    ctx.clearRect(0, 0, width, height);
    
    // Draw dark background circle
    ctx.fillStyle = '#0a0a0a';
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.fill();
    
    // Draw outer ring
    ctx.strokeStyle = '#4a9eff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.stroke();
    
    // Create vertical tape display
    const tapeHeight = radius * 1.6;
    const pixelsPerMeter = tapeHeight / 50; // Show 50m range
    
    ctx.save();
    
    // Clip to circle
    ctx.beginPath();
    ctx.arc(cx, cy, radius - 5, 0, 2 * Math.PI);
    ctx.clip();
    
    // Calculate visible altitude range
    const minAlt = Math.floor(altitude - 25);
    const maxAlt = Math.ceil(altitude + 25);
    
    // Draw altitude tape
    for (let alt = Math.floor(minAlt / 10) * 10; alt <= maxAlt; alt += 2) {
        const y = cy - (alt - altitude) * pixelsPerMeter;
        
        // Skip if outside visible area
        if (y < cy - radius || y > cy + radius) continue;
        
        const isMajor = alt % 10 === 0;
        const lineLength = isMajor ? 25 : 12;
        const lineWidth = isMajor ? 2 : 1;
        
        // Draw tick mark
        ctx.strokeStyle = '#4a9eff';
        ctx.lineWidth = lineWidth;
        ctx.beginPath();
        ctx.moveTo(cx - lineLength, y);
        ctx.lineTo(cx, y);
        ctx.stroke();
        
        // Draw labels for major marks
        if (isMajor) {
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 12px monospace';
            ctx.textAlign = 'right';
            ctx.textBaseline = 'middle';
            ctx.fillText(alt.toString(), cx - lineLength - 5, y);
        }
    }
    
    ctx.restore();
    
    // Draw center reference line/box
    ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
    const boxWidth = 50;
    const boxHeight = 24;
    ctx.fillRect(cx - boxWidth/2, cy - boxHeight/2, boxWidth, boxHeight);
    
    ctx.strokeStyle = '#4a9eff';
    ctx.lineWidth = 2;
    ctx.strokeRect(cx - boxWidth/2, cy - boxHeight/2, boxWidth, boxHeight);
    
    // Draw current altitude value
    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 14px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(altitude.toFixed(1), cx, cy);
    
    // Draw reference triangles
    ctx.fillStyle = '#4a9eff';
    ctx.beginPath();
    ctx.moveTo(cx - boxWidth/2, cy);
    ctx.lineTo(cx - boxWidth/2 - 8, cy - 8);
    ctx.lineTo(cx - boxWidth/2 - 8, cy + 8);
    ctx.closePath();
    ctx.fill();
    
    ctx.beginPath();
    ctx.moveTo(cx + boxWidth/2, cy);
    ctx.lineTo(cx + boxWidth/2 + 8, cy - 8);
    ctx.lineTo(cx + boxWidth/2 + 8, cy + 8);
    ctx.closePath();
    ctx.fill();
}

// Draw range indicator (laser) - Fixed arc gauge
function drawRange(ctx, range) {
    if (!ctx) return;
    
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.min(width, height) / 2 - 10;
    
    ctx.clearRect(0, 0, width, height);
    
    // Draw dark background circle
    ctx.fillStyle = '#0a0a0a';
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.fill();
    
    // Draw outer ring
    ctx.strokeStyle = '#4a9eff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2 * Math.PI);
    ctx.stroke();
    
    // Configuration
    const maxRange = 100; // meters
    const startAngle = -0.75 * Math.PI; // -135 degrees
    const sweepAngle = 1.5 * Math.PI;   // 270 degrees sweep
    
    // Draw background arc (gray)
    ctx.strokeStyle = '#2a2a2a';
    ctx.lineWidth = 12;
    ctx.beginPath();
    ctx.arc(cx, cy, radius - 15, startAngle, startAngle + sweepAngle);
    ctx.stroke();
    
    // Draw tick marks
    ctx.save();
    ctx.translate(cx, cy);
    
    for (let i = 0; i <= 10; i++) {
        const angle = startAngle + (i / 10) * sweepAngle;
        const isMajor = i % 2 === 0;
        const lineLength = isMajor ? 18 : 10;
        const lineWidth = isMajor ? 2 : 1;
        
        ctx.strokeStyle = '#4a9eff';
        ctx.lineWidth = lineWidth;
        
        const x1 = Math.cos(angle) * (radius - 30);
        const y1 = Math.sin(angle) * (radius - 30);
        const x2 = Math.cos(angle) * (radius - 30 + lineLength);
        const y2 = Math.sin(angle) * (radius - 30 + lineLength);
        
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
        
        // Draw labels at major ticks
        if (isMajor) {
            const label = (i * 10).toString();
            const labelRadius = radius - 35;
            const lx = Math.cos(angle) * labelRadius;
            const ly = Math.sin(angle) * labelRadius;
            
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 11px monospace';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(label, lx, ly);
        }
    }
    
    ctx.restore();
    
    // Draw range arc (colored based on distance)
    // Fixed: Changed condition to handle range === 0 and include full range
    if (range >= 0 && range <= maxRange) {
        // Color coding: green (far), yellow (medium), red (close)
        let arcColor;
        if (range > 20) {
            arcColor = '#00ff00'; // Green - safe distance
        } else if (range > 5) {
            arcColor = '#ffaa00'; // Orange - caution
        } else {
            arcColor = '#ff0000'; // Red - warning
        }
        
        const arcAngle = (Math.min(range, maxRange) / maxRange) * sweepAngle;
        
        ctx.strokeStyle = arcColor;
        ctx.lineWidth = 12;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.arc(cx, cy, radius - 15, startAngle, startAngle + arcAngle);
        ctx.stroke();
    }
    
    // Draw center digital readout
    const boxWidth = 60;
    const boxHeight = 26;
    ctx.fillStyle = 'rgba(0, 0, 0, 0.9)';
    ctx.fillRect(cx - boxWidth/2, cy - boxHeight/2, boxWidth, boxHeight);
    
    ctx.strokeStyle = '#4a9eff';
    ctx.lineWidth = 2;
    ctx.strokeRect(cx - boxWidth/2, cy - boxHeight/2, boxWidth, boxHeight);
    
    // Display range value or NO DATA
    if (range >= 0 && range <= maxRange) {
        ctx.fillStyle = '#00ff00';
        ctx.font = 'bold 16px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(range.toFixed(1) + 'm', cx, cy);
    } else {
        ctx.fillStyle = '#ff3333';
        ctx.font = 'bold 11px Arial';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('NO DATA', cx, cy);
    }
    
    // Draw "RANGE" label at bottom
    ctx.fillStyle = '#4a9eff';
    ctx.font = 'bold 10px Arial';
    ctx.textAlign = 'center';
    ctx.fillText('RANGE', cx, cy + radius - 10);
}

// Main drawing loop
function drawCompactInstruments() {
    if (instrumentsNeedUpdate) {
        drawAttitude(attitudeCtx, currentPitch, currentRoll);
        drawHeading(headingCtx, currentHeading);
        drawAltitude(altitudeCtx, currentAltitude);
        drawRange(laserCtx, currentLaserRange);
        instrumentsNeedUpdate = false;
    }
    requestAnimationFrame(drawCompactInstruments);
}

// Start drawing loop
drawCompactInstruments();

// ============================================
// SENSOR VISUALIZATIONS
// ============================================

// Laser scan visualization
const laserscanCanvas = document.getElementById('laserscan-canvas');
const laserscanCtx = laserscanCanvas.getContext('2d');

function resizeLaserscanCanvas() {
    laserscanCanvas.width = laserscanCanvas.parentElement.clientWidth - 20;
    laserscanCanvas.height = 300;
}
resizeLaserscanCanvas();
window.addEventListener('resize', resizeLaserscanCanvas);

const laserscanListener = new ROSLIB.Topic({
    ros: ros,
    name: '/scan',
    messageType: 'sensor_msgs/LaserScan'
});

laserscanListener.subscribe(throttle(function(message) {
    const width = laserscanCanvas.width;
    const height = laserscanCanvas.height;
    const centerX = width / 2;
    const centerY = height / 2;
    const maxRange = message.range_max || 10.0;
    const scale = Math.min(width, height) / 2 / maxRange;
    
    laserscanCtx.fillStyle = '#1a1a1a';
    laserscanCtx.fillRect(0, 0, width, height);
    
    // Draw robot
    laserscanCtx.fillStyle = '#00ff00';
    laserscanCtx.beginPath();
    laserscanCtx.arc(centerX, centerY, 5, 0, 2*Math.PI);
    laserscanCtx.fill();
    
    // Draw points
    laserscanCtx.fillStyle = '#00ffff';
    const ranges = message.ranges;
    let validPoints = 0;
    
    for (let i = 0; i < ranges.length; i++) {
        const range = ranges[i];
        if (isNaN(range) || range < message.range_min || range > message.range_max) continue;
        
        validPoints++;
        const angle = message.angle_min + i * message.angle_increment;
        const x = centerX + Math.sin(angle) * range * scale;
        const y = centerY - Math.cos(angle) * range * scale;
        
        laserscanCtx.beginPath();
        laserscanCtx.arc(x, y, 2, 0, 2*Math.PI);
        laserscanCtx.fill();
    }
    
    document.getElementById('laserscan-points').textContent = validPoints;
    document.getElementById('laserscan-range').textContent = maxRange.toFixed(1);
}, VCS_CONFIG.update_intervals.laserscan));

// Velodyne 3D Point Cloud
const velodyneContainer = document.getElementById('velodyne-container');
const velodyneScene = new THREE.Scene();
velodyneScene.background = new THREE.Color(0x1a1a1a);

const velodyneCamera = new THREE.PerspectiveCamera(
    75, velodyneContainer.clientWidth / velodyneContainer.clientHeight, 0.1, 1000
);
velodyneCamera.position.set(0, 0, 50);

const velodyneRenderer = new THREE.WebGLRenderer({ antialias: true });
velodyneRenderer.setSize(velodyneContainer.clientWidth, velodyneContainer.clientHeight);
velodyneContainer.appendChild(velodyneRenderer.domElement);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
velodyneScene.add(ambientLight);

const velodyneGeometry = new THREE.BufferGeometry();
const velodyneMaterial = new THREE.PointsMaterial({ color: 0x00ffff, size: 0.1 });
const velodynePoints = new THREE.Points(velodyneGeometry, velodyneMaterial);
velodyneScene.add(velodynePoints);

function animateVelodyne() {
    requestAnimationFrame(animateVelodyne);
    velodyneRenderer.render(velodyneScene, velodyneCamera);
}
animateVelodyne();

const velodyneListener = new ROSLIB.Topic({
    ros: ros,
    name: '/velodyne_points',
    messageType: 'sensor_msgs/PointCloud2'
});

velodyneListener.subscribe(throttle(function(message) {
    // Simplified point cloud parsing
    const points = [];
    // Parse PointCloud2 data (simplified version)
    // Full implementation would include proper field parsing
    
    if (points.length > 0) {
        velodyneGeometry.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
        document.getElementById('velodyne-points').textContent = (points.length / 3).toLocaleString();
    }
}, VCS_CONFIG.update_intervals.velodyne));

function resetVelodyneView() {
    velodyneCamera.position.set(0, 0, 50);
    velodyneCamera.lookAt(0, 0, 0);
}

let velodyneAutoRotate = false;
function toggleVelodyneAutoRotate() {
    velodyneAutoRotate = !velodyneAutoRotate;
    document.getElementById('auto-rotate-status').textContent = velodyneAutoRotate ? 'On' : 'Off';
}

console.log('Earth Rover VCS initialized');

