# utils — how to launch the Husky A300 stack

There are **two launchers**, one per layer, each with its own TOML. Details of the web monitor and its
architecture: [netmon/README.md](netmon/README.md).

## Start-up

```bash
cd ~/robocomp/AiGB_UEx_Navigation_v2/active_inference/utils

# Terminal 1 — sensorimotor layer (Webots + bridge + LiDAR + IMU + ZED + gamepad)
python3 subcognitive.py sub_husky.toml

# Terminal 2 — cognitive layer (DSR agents), once the table in terminal 1 is green
python3 cognitive.py cognitive_husky.toml
```

- Web monitor: <http://127.0.0.1:8080>
- Webots must have **`webots-agri/worlds/husky_apple.wbt`** open (`../../apple.bash` opens it for you).
  The launcher starts Webots if it is not running, but it opens **the last world used**: check it.
- Launch **from `utils/`**: both scripts import the `netmon/` package in this folder.
- Without an argument, each launcher uses its Husky TOML.
- Navigation goal from the terminal: `python3 send_goal.py X Y` (metres in the field frame, ENU: x east,
  y north).

## The two launchers

| Script | Layer | What it starts | TOML |
|---|---|---|---|
| `subcognitive.py` | sensorimotor | Webots (if needed), rcnode/icebox, bridge, `lidar3d_dds` (helios), `imu_dds`, `zed_camera`, Xbox gamepad | `sub_husky.toml` |
| `cognitive.py` | cognitive | `robot_concept`, `openfield_concept`, `residual_concept`, `controller`, `retina`, `human_concept`, `vegetation_concept`, `trunk_concept` (`row_metaconcept` exists but is disabled) | `cognitive_husky.toml` |

Always **`subcognitive.py` first, then `cognitive.py`**. The agents do not start Webots and need the sensors
already publishing. The order *inside* `cognitive_husky.toml` does not matter: every agent waits for its
`required_agent_names`.

Both share the web monitor: the first one to start serves it and the other only registers its state.

> ⚠ **Do not pass the other launcher's TOML.** `subcognitive.py cognitive_husky.toml` starts anyway, but in the
> wrong layer. For example, `cognitive.py` never starts Webots.

To check that every TOML entry has its binary and its config:

```bash
python3 - <<'E'
import toml, os, glob
for f in sorted(glob.glob('*.toml')):
    for c in toml.load(f).get('components', []):
        cwd = os.path.expanduser(c.get('cwd', '')); cmd = c.get('cmd', '').split()
        if cmd and cmd[0] == 'env':
            cmd = [x for x in cmd[1:] if '=' not in x]
        miss = [x for x, p in (('bin', cmd[0]), ('cfg', cmd[1] if len(cmd) > 1 else None))
                if p and not os.path.exists(os.path.join(cwd, p))]
        if miss: print(f, c['name'], miss)
E
```

## Logs

Without `tmux`, each component writes to:

```
~/.local/logs/<name>.out     # stdout
~/.local/logs/<name>.err     # stderr
```

`<name>` is the TOML `name` field (`bridge`, `controller`, `robot_concept`…). **They are overwritten on every
start**: copy a log **after** stopping the component and **before** restarting it.

The Webots console goes to no log (the launcher sends it to `/dev/null`); it is shown in the Webots window itself.

With `tmux` installed, each component runs in its own session and the web monitor shows its terminal.

## Stopping

- **Ctrl+C in each terminal**: `cognitive.py` first, then `subcognitive.py`. The launcher sends SIGTERM to each
  component, the agents delete their nodes from the DSR graph and, if one does not finish within 5 s, it is killed
  with SIGKILL.
- **Never `kill -9`** an agent: its nodes are left dangling in the shared graph.
- Relaunching with old processes still alive also kills them with **SIGKILL** (`_remove_existing`). Stop with
  Ctrl+C first and relaunch afterwards.
- Webots is not closed when `subcognitive.py` stops: it is reused on the next start.

## Useful flags

| Flag | Script | Effect |
|---|---|---|
| `--no-webots` | `subcognitive.py` | Do not start Webots (ignored if the TOML has `[general] start_webots`) |
| `--no-rcnode` | `subcognitive.py` | Do not check/start rcnode (icebox) |
| `--no-battery` | `subcognitive.py` | No Victron battery monitor (`/dev/ttyUSB0`) |
| `--no-web` | both | Do not serve the web monitor |
| `--web-port N` | both | Web port (the same in both; 8080 by default) |
| `--no-bw` | both | No ICE bandwidth capture |

## Other things in this folder

| Path | What it is |
|---|---|
| `netmon/` | Package shared by both launchers: web monitor, registry in `/tmp/robocomp_netmon/`, DDS statistics bridge |
| `dds_preflight/` | Checks that DDS discovery over shared memory works (domain 7). The launchers run it at start-up. If it fails, stale segments are almost always left over: stop everything and `rm -f /dev/shm/fastdds_*`. Symptom without it: the media plane at 0.0 Hz while Ice works. |
| `send_goal.py` | Sends a navigation goal to the controller: `python3 send_goal.py X Y` |
| `run_route.py` | Drives a fixed route of goals (by default, out and back along three corridors of the orchard, away from the workers) and moves to the next one at 0.6 m: reproducible runs for measuring. Needs `Controller.ArmOnStart = true` |
