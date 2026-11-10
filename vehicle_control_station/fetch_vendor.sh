#!/bin/bash
# Download the browser libraries the dashboard used to load from CDNs.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V="$HERE/vehicle_control_station/static/vehicle-control-station/vendor"
mkdir -p "$V/leaflet-1.7.1/images"
get() { curl -fsSL --retry 3 -o "$2" "$1"; }
get https://unpkg.com/leaflet@1.7.1/dist/leaflet.js  "$V/leaflet-1.7.1/leaflet.js"
get https://unpkg.com/leaflet@1.7.1/dist/leaflet.css "$V/leaflet-1.7.1/leaflet.css"
for i in layers.png layers-2x.png marker-icon.png marker-icon-2x.png marker-shadow.png; do
    get "https://unpkg.com/leaflet@1.7.1/dist/images/$i" "$V/leaflet-1.7.1/images/$i"
done
get https://unpkg.com/leaflet@1.7.1/LICENSE "$V/leaflet-1.7.1/LICENSE"
get https://d3js.org/d3.v6.min.js "$V/d3.v6.min.js"
get https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js "$V/three-r128.min.js"
cat > "$V/README.md" <<'EOF'
# Vendored browser libraries

Fetched by `vehicle_control_station/fetch_vendor.sh` so the dashboard loads with
no internet on the rover.

| File | Version | License | Source |
|---|---|---|---|
| `leaflet-1.7.1/` | Leaflet 1.7.1 | BSD-2-Clause (`leaflet-1.7.1/LICENSE`) | unpkg.com/leaflet@1.7.1/dist |
| `d3.v6.min.js` | d3 v6 | ISC (header of the file) | d3js.org |
| `three-r128.min.js` | three.js r128 | MIT (header of the file) | cdnjs three.js/r128 |
EOF
ls -la "$V" "$V/leaflet-1.7.1" "$V/leaflet-1.7.1/images"
