#!/usr/bin/env bash
# apple_human.bash — launch the whole Husky A300 stack in the APPLE ORCHARD world with the keyboard-controlled
# farmer 'granjero' (webots-agri/worlds/husky_apple_human.wbt).
#
#   ./apple_human.bash            Webots + sensorimotor layer + cognitive layer, each launcher in its own terminal
#   ./apple_human.bash --route    ... and, once the agents are up, drive the fixed test route (utils/run_route.py)
#   ./apple_human.bash --stop     stop everything gracefully: cognitive layer, then sensorimotor layer, then Webots
#   ./apple_human.bash --dry-run  print what would be run, run nothing
#
# Order (README.md, "Lanzar"): Webots with the world -> subcognitive.py sub_husky.toml (bridge, helios, IMU, ZED,
# joystick; it also starts rcnode/IceStorm if it is not running) -> cognitive.py cognitive_husky.toml (DSR agents).
# Stop with Ctrl+C in the COGNITIVE terminal first, then in the sensorimotor one — or ./apple_human.bash --stop.
# Never kill -9 an agent: its nodes would stay in the shared graph.
#
# The components must be built (cmake -B build && make -C build -j8 in each, see README.md) and the repo must live
# at ~/robocomp/AiGB_UEx_Navigation_v2: the launcher TOMLs point their working directories there.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS="$REPO/active_inference/utils"
WORLD="$REPO/webots-agri/worlds/husky_apple_human.wbt"
WEBOTS="${WEBOTS:-/usr/local/bin/webots}"
SUB_WAIT_S=25      # sensorimotor layer up (bridge connected to Webots, sensors publishing) before the cognitive one
ROUTE_WAIT_S=45    # agents Operating before the test route starts

ROUTE=0
DRY=0
STOP=0
for arg in "$@"; do
    case "$arg" in
        --route)   ROUTE=1 ;;
        --dry-run) DRY=1 ;;
        --stop)    STOP=1 ;;
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg (see --help)" >&2; exit 2 ;;
    esac
done

run() { if (( DRY )); then echo "[dry-run] $*"; else "$@"; fi; }
say() { echo "[apple_human] $*"; }

# ── --stop: the graceful order, waiting for each layer to finish its own cleanup ─────────────────────────────────
wait_gone() {   # wait_gone <pgrep -f pattern> <seconds>
    for _ in $(seq 1 "$2"); do pgrep -f "$1" >/dev/null || return 0; sleep 1; done
    return 1
}
if (( STOP )); then
    if pid=$(pgrep -f "^python3 cognitive.py" | head -1); then
        say "stopping the cognitive layer (SIGINT to cognitive.py $pid)"
        run kill -INT "$pid"
        (( DRY )) || wait_gone "bin/(robot_concept|openfield_concept|residual_concept|controller|retina|human_concept|plant_concept|row_metaconcept|plot_metaconcept|field_metaconcept) " 40 \
            || say "some agents are still running after 40 s"
    fi
    if pid=$(pgrep -f "^python3 subcognitive.py" | head -1); then
        say "stopping the sensorimotor layer (SIGINT to subcognitive.py $pid)"
        run kill -INT "$pid"
        (( DRY )) || wait_gone "bin/(Webots2Robocomp|lidar3d_dds|imu_dds|zed_camera) " 40 \
            || say "some drivers are still running after 40 s"
    fi
    if pid=$(pgrep -x webots-bin | head -1); then
        say "closing Webots ($pid)"
        run kill -TERM "$pid"
        # A simulator holds no graph nodes, so forcing it is safe; a dialog or a slow exit must not hang --stop.
        if (( ! DRY )) && ! wait_gone "webots-bin" 15; then
            say "Webots did not close in 15 s — forcing it"
            kill -KILL "$pid" 2>/dev/null || true
        fi
    fi
    say "stopped (rcnode/IceStorm is left running, as the launchers leave it)"
    exit 0
fi

# ── Checks ──────────────────────────────────────────────────────────────────────────────────────────────────────
if [[ "$REPO" != "$HOME/robocomp/AiGB_UEx_Navigation_v2" ]]; then
    say "WARNING: the repo is at $REPO, but the launcher TOMLs expect ~/robocomp/AiGB_UEx_Navigation_v2"
fi
[[ -x "$WEBOTS" ]] || { say "Webots not found at $WEBOTS (set WEBOTS=/path/to/webots)"; exit 1; }
[[ -f "$WORLD" ]]  || { say "world not found: $WORLD"; exit 1; }
missing=()
for bin in webots-bridge/bin/Webots2Robocomp \
           robocomp-robolab/components/hardware/laser/lidar3d_dds/bin/lidar3d_dds \
           robocomp-robolab/components/hardware/imu/imu_dds/bin/imu_dds \
           robocomp-robolab/components/hardware/camera/zed_camera/bin/zed_camera \
           active_inference/robot_concept/bin/robot_concept \
           active_inference/openfield_concept/bin/openfield_concept \
           active_inference/residual_concept/bin/residual_concept \
           active_inference/controller/bin/controller \
           active_inference/plant_concept/bin/plant_concept \
           active_inference/row_metaconcept/bin/row_metaconcept \
           active_inference/plot_metaconcept/bin/plot_metaconcept \
           active_inference/retina/bin/retina \
           active_inference/human_concept/bin/human_concept; do
    [[ -x "$REPO/$bin" ]] || missing+=("$bin")
done
if (( ${#missing[@]} )); then
    say "not built yet (cmake -B build && make -C build -j8 in each component):"
    printf '         %s\n' "${missing[@]}"
    exit 1
fi
if pgrep -x webots-bin >/dev/null; then
    say "Webots is already running — close it first (or ./apple_human.bash --stop): the launcher would reuse whatever world it has open"
    exit 1
fi
if pgrep -f "^python3 (sub)?cognitive.py" >/dev/null; then
    say "a launcher is already running — stop it first (./apple_human.bash --stop)"
    exit 1
fi

# ── A terminal per launcher: they are interactive consoles (status table, keyboard) ─────────────────────────────
open_terminal() {   # open_terminal <title> <command>
    local title="$1" cmd="$2; echo; echo '[$1] finished — this window can be closed'; exec bash"
    if command -v gnome-terminal >/dev/null; then
        run gnome-terminal --title="$title" -- bash -c "$cmd"
    elif command -v konsole >/dev/null; then
        run konsole -p tabtitle="$title" -e bash -c "$cmd" &
    elif command -v xterm >/dev/null; then
        run xterm -T "$title" -e bash -c "$cmd" &
    else
        say "no terminal emulator found (gnome-terminal / konsole / xterm)"; exit 1
    fi
}

# ── Launch ──────────────────────────────────────────────────────────────────────────────────────────────────────
say "1/3 Webots with $(basename "$WORLD")"
if (( DRY )); then
    echo "[dry-run] $WEBOTS --batch --mode=realtime $WORLD &"
else
    # --batch: no blocking pop-ups (e.g. "save the modified world?" on exit, which made it ignore --stop).
    ( cd "$REPO/webots-agri/worlds" && nohup "$WEBOTS" --batch --mode=realtime "$WORLD" >/dev/null 2>&1 & )
    for _ in $(seq 1 30); do pgrep -x webots-bin >/dev/null && break; sleep 1; done
    pgrep -x webots-bin >/dev/null || { say "Webots did not start"; exit 1; }
    sleep 8   # the world loads its PROTOs (the first time it also downloads Webots' own ones)
fi

say "2/3 sensorimotor layer (bridge, helios, IMU, ZED, joystick) — terminal 'Husky · sensorimotor'"
open_terminal "Husky · sensorimotor" "cd '$UTILS' && python3 subcognitive.py sub_husky.toml"
(( DRY )) || sleep "$SUB_WAIT_S"

say "3/3 cognitive layer (DSR agents) — terminal 'Husky · cognitive'"
open_terminal "Husky · cognitive" "cd '$UTILS' && python3 cognitive.py cognitive_husky.toml"

if (( ROUTE )); then
    say "test route in ${ROUTE_WAIT_S} s (utils/run_route.py; needs Controller.ArmOnStart = true)"
    (( DRY )) || sleep "$ROUTE_WAIT_S"
    run bash -c "cd '$UTILS' && python3 run_route.py"
fi

say "up. Web monitor: http://127.0.0.1:8080 · goal: python3 $UTILS/send_goal.py X Y · stop: ./apple_human.bash --stop"
