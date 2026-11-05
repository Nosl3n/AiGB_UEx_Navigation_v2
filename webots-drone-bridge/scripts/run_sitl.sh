#!/bin/bash
# Start ArduCopter SITL for the inspection drone. drone_bridge (Flight.Backend = "ardupilot") is its physics:
# SITL sends servo frames to UDP 9002 and waits, lockstep, for the state of each Webots step.
#   MAVLink: tcp:127.0.0.1:5760 (SERIAL0)   — GCS / mission scripts connect here
#
# usage: run_sitl.sh [--wipe]        --wipe = forget parameters saved by a previous run (eeprom.bin)
# env:   ARDUPILOT_HOME (default ~/robocomp/ardupilot)
#        SITL_HOME      lat,lon,alt_m,heading_deg of the take-off point (default: Badajoz)
#        SITL_DIR       working dir for eeprom.bin and logs (default ~/.cache/inspection_drone_sitl)
#        SITL_EXTRA_PARM  one more .parm file loaded last (experiments; overrides the two above)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AP="${ARDUPILOT_HOME:-$HOME/robocomp/ardupilot}"
HOME_LOC="${SITL_HOME:-38.8790,-6.9707,190,0}"
RUN_DIR="${SITL_DIR:-$HOME/.cache/inspection_drone_sitl}"
BIN="$AP/build/sitl/bin/arducopter"
[ -x "$BIN" ] || { echo "no $BIN — build it: cd $AP && ./waf configure --board sitl && ./waf copter"; exit 1; }
mkdir -p "$RUN_DIR" && cd "$RUN_DIR"
WIPE=""; [ "$1" = "--wipe" ] && WIPE="--wipe"
# Not exec'd: SITL answers SIGTERM by raising an exit flag it only checks between physics steps, so once the
# physics (Webots / drone_bridge) is gone it waits for ever and never exits. Stay in front and turn the launcher's
# SIGTERM (or a Ctrl-C) into SIGINT, which ends it at once. Nothing is lost: every launch starts with --wipe.
# env --default-signal=INT: a background job of a non-interactive shell (or of a launcher started in the
# background) inherits SIGINT IGNORED, and then the SIGINT below would do nothing.
# --serial0 tcp:0: listen on 5760 WITHOUT waiting — SITL's default (tcp:0:wait) blocks the autopilot until a ground
# station connects, and here the joystick pilot uses SERIAL1 (5762).
env --default-signal=INT "$BIN" --model JSON:127.0.0.1 --speedup 1 --home "$HOME_LOC" $WIPE --serial0 tcp:0 \
     --defaults "$AP/Tools/autotest/default_params/copter.parm,$HERE/etc/ardupilot/inspection_drone.parm${SITL_EXTRA_PARM:+,$SITL_EXTRA_PARM}" &
CHILD=$!
trap 'kill -INT "$CHILD" 2>/dev/null' TERM INT HUP
wait "$CHILD"; wait "$CHILD" 2>/dev/null
exit 0
