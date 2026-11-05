#!/bin/bash
# Self-contained ArduCopter SITL flight test of the inspection drone, headless, without the rest of the stack:
#   1. derives a test world from webots-agri/worlds/dos_fields_dron.wbt with the Husky without controller (so
#      Webots does not wait for its bridge);
#   2. headless Webots on port 1299, drone_bridge with Flight.Backend = "ardupilot", ArduCopter SITL;
#   3. test_flight.py: GUIDED take-off to 3 m, hover, 90 deg yaw, LAND; prints PASS/FAIL;
#   4. stops only what it started.
# usage: sitl_selftest.sh [--fast] [test_flight.py args...]     --fast = Webots in fast mode (same physics, quicker)
# Needs: ~/robocomp/ardupilot built (build/sitl/bin/arducopter) and ~/robocomp/ardupilot-venv (pymavlink).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
WORK="${SITL_DIR:-$HOME/.cache/inspection_drone_sitl}"; mkdir -p "$WORK"
MODE=realtime; [ "${1:-}" = "--fast" ] && { MODE=fast; shift; }
PY="${ARDUPILOT_VENV:-$HOME/robocomp/ardupilot-venv}/bin/python"
WORLD="$REPO/webots-agri/worlds/_sitl_selftest.wbt"          # must live next to its EXTERNPROTO paths

# dos_fields_dron.wbt already has basicTimeStep 2 and externalPropulsion TRUE; only the Husky loses its controller.
sed -e 's/^DEF husky HuskyDroneCarrier {$/DEF husky HuskyDroneCarrier {\n  controller "<none>"/' \
    "$REPO/webots-agri/worlds/dos_fields_dron.wbt" > "$WORLD"
sed -e 's/^Backend = "internal"/Backend = "ardupilot"/' \
    -e 's/^MediaPlaneDDS = "tcp -p 12110"/MediaPlaneDDS = "tcp -p 12120"/' \
    -e 's/^FullPoseEstimation = "tcp -p 12111"/FullPoseEstimation = "tcp -p 12121"/' \
    "$HERE/etc/config.toml" > "$WORK/drone_bridge_selftest.toml"

PIDS=()
cleanup() {
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
    sleep 2; rm -f "$WORLD"
}
trap cleanup EXIT

(cd "$REPO/webots-agri/worlds" && exec webots --batch --mode=$MODE --minimize --no-rendering --port=1299 \
    --stdout --stderr "$WORLD" > "$WORK/webots.log" 2>&1) & PIDS+=($!)
sleep 8
(cd "$HERE" && WEBOTS_CONTROLLER_URL=ipc://1299/inspection_drone exec bin/drone_bridge "$WORK/drone_bridge_selftest.toml" \
    > "$WORK/drone_bridge.log" 2>&1) & PIDS+=($!)
sleep 3
(SITL_DIR="$WORK/sitl" exec "$HERE/scripts/run_sitl.sh" --wipe > "$WORK/sitl.log" 2>&1) & PIDS+=($!)
sleep 3
"$PY" "$HERE/scripts/test_flight.py" "$@"
RC=$?
echo "logs: $WORK"
exit $RC
