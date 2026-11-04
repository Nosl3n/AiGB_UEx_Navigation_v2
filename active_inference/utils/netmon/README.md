# netmon — RoboComp interconnection web monitor

Launches the components/agents of a RoboComp stack, monitors them (state, CPU, memory) and serves a **web map** of
how they are interconnected: ICE proxies (RPC), IceStorm publish/subscribe, DDS publications (with the real
measured bandwidth, not an estimate), DSR graph spheres, bandwidth per connection and a live video window for the
mediamtx RTSP/WebRTC stream.

The Python side uses only the **stdlib** (no FastAPI/scapy): the web server is `http.server`, capture uses a raw
socket. The only non-stdlib piece is an optional C++ binary (`dds_stats_bridge/`) that measures the real DDS
bandwidth — see below.

---

## Launching

From `~/robocomp/AiGB_UEx_Navigation_v2/active_inference/utils`:

```bash
# Sensorimotor layer only — hosts the web at http://0.0.0.0:8080 (reachable from the network)
python3 subcognitive.py sub_husky.toml

# Add the cognitive layer (agents + DSR sphere) — in another terminal, any order
python3 cognitive.py cognitive_husky.toml
```

Open **http://\<machine-ip\>:8080** (or `http://127.0.0.1:8080` locally). Stop each terminal with `Ctrl+C` (it ends
its processes, releases the monitor lock and deletes its registry entry).

### Flags

| Flag | Effect |
|------|--------|
| `--web-host IP` | Web bind address (default `0.0.0.0`, every interface) |
| `--web-port N` | Web port (both launchers must use the same one; default `8080`) |
| `--no-web` | Do not host the web (console table + `/tmp` registry only) |
| `--no-bw` | Disable the ICE (loopback) bandwidth capture |
| `--no-webots` | (`subcognitive.py`) Do not auto-start Webots — only applies if the TOML has no `[general]`, see below |

### Starting Webots / rcnode from the config

Instead of (or as well as) the CLI flags, any component TOML can carry a `[general]` table that overrides the
command line:

```toml
[general]
start_webots = false   # e.g. real hardware, no simulator
start_rcnode = true    # starts rcnode (icebox) if it is not already running
```

Both starts are idempotent: if the process is already running (Webots, or `icebox` for rcnode) it is reused
instead of relaunched.

### ICE bandwidth per edge (optional)

The `KB/s` on the RPC edges only appear if the interpreter has capture permission. **Once**:

```bash
sudo setcap cap_net_raw,cap_net_admin+eip $(readlink -f $(which python3))
```

Without it the map works the same (state, topology, edges); only the ICE bytes/s are missing. The **DDS** bandwidth
does not depend on this — see its own section below.

---

## Architecture

Each launcher writes its state to a shared registry in `/tmp`; the first one to start takes the web-server role
and reads **every** registry. If it dies, another one takes over.

```
subcognitive.py ─┐                          /tmp/robocomp_netmon/
cognitive.py ────┼─▶ netmon/launcher.py ──▶   agents/subcognitive.json
                  │      (each one               agents/cognitive.json
                  │       updates 1×/s)          monitor.lock  ◀─ a single web server
                  └─▶ the holder of monitor.lock serves the web with the union of registries
```

- **Registry** (`registry.py`): `agents/<launcher>.json` with `{name, status, cpu, mem, pid, cwd, cmd, layer,
  domain}`. Entries not updated for more than 5 s are ignored.
- **Lock** (`monitor.lock`): create-exclusive per PID; a stale host (dead or old PID) is taken over automatically.

---

## The web view

**Bottom-up hierarchical** layout:

```
        (top)      ● DSR·dN            shared DSR graph sphere
                   ⬭ agents            concept agents (cognitive) — ellipses
                   ◆ IceStorm          ICE topic broker
                   ⬡ DDS·dN            DDS domain broker (orange hexagon)
                   ▢ components        sensorimotor components with dependencies
        (bottom)   ▢ bridge / sources  hubs that consume nothing
```

- **Nodes** — colour by state: `alive` green · `up (ICE ✗)` yellow · `stopped` red. Shapes: `box` component ·
  `ellipse` DSR agent · `dot` DSR sphere · `diamond` IceStorm broker · `hexagon` DDS broker · dashed `box` =
  external (not managed, e.g. `kinovaarm:12333`).
- **Edges**:
  - **RPC** (solid line, `:port interface`): thickens and turns green with traffic (`KB/s`).
  - **ICE pub/sub** (dotted purple): `pub`/`sub` + topic, through IceStorm. Several topics of the same pair are
    grouped into one multi-line edge.
  - **DDS** (dotted orange): component → `DDS·dN` broker, one or more topics per component (e.g.
    `RGBTopic`/`DepthTopic`). Thickens and turns green with real measured traffic — see the DDS section.
  - **DSR** (cyan, no arrow): agent ↔ shared graph (read/write).
- **Tooltip** of each node: what it implements / requires / publishes / subscribes / DDS (domain + topics), and
  CPU/mem/state.
- **Layer buttons** (top bar): `RPC` / `IceStorm` / `DDS` toggle the visibility of each group of edges without
  touching the rest of the graph (useful to untangle a busy graph).
- You can **drag** any node with the mouse; once dropped it stays free for good (it no longer returns to its
  level band), so you can place nodes by hand without fighting the automatic layout.
- **View** button: cycles **Organic by level** (default: force-directed with each node anchored to its level band)
  → **Hierarchical** → **Free organic**. **Re-layout** re-arranges; **Table** shows/hides the side panel. Physics
  freezes once stable, so the map does not move with the state refreshes.

### Live view (top bar)

The **🎥 Live view** button opens/closes a floating, resizable panel with an `<iframe>` pointing at the URL typed
in the text field (remembered in `localStorage`). It is meant for the HTML page with an embedded player that
**mediamtx** serves on its WebRTC port (`http://<host>:8889/<path>`) — it needs no extra frontend library, it is
just a generic iframe, so an HLS URL or any other video page works too. Closing the panel sets the iframe to
`about:blank` so it stops decoding/consuming bandwidth in the background.

### Process control (Table panel)

Each table row carries actions on its component:

- **↻ restart** · **⏹ stop** · **📄 logs**.
- Stop/restart run safely: the monitor does **not** kill processes it does not own; it drops a command in
  `/tmp/robocomp_netmon/commands/` that **the launcher owning** the process (the one that started it) claims and
  executes. Only `stop`/`restart` on known components are accepted. External processes (Webots, rcnode,
  `dds_stats_bridge`) cannot be controlled from the web.
- **📄 logs** opens a bottom panel with the `tail` of `~/.local/logs/<name>.{err,out}` (stdout/stderr switchable),
  refreshed every 1.5 s and auto-scrolled to the end.

---

## Real DDS bandwidth (`dds_stats_bridge`)

The `*_dds` components (e.g. `lidar3d_dds`) publish with `SharedMemoryOnly = true`: the data travels through shared
memory between processes on the same host, **never over the network** — `bandwidth.py`'s sniffing (raw socket on
`lo`) structurally cannot see it. That traffic is measured with **Fast DDS's native statistics module**, which
reports the real throughput per *DataWriter* regardless of the transport.

### How it works

1. `netmon/launcher.py` finds, from the `[DDS]` **and** `[Media]` sections of each component's config (parsed by
   `topology.component_dds`/`component_media`), which ones publish or consume through DDS and on which domain. It
   injects the environment variable
   `FASTDDS_STATISTICS=_fastdds_statistics_publication_throughput;_fastdds_statistics_subscription_throughput`
   into those processes when starting them (and when restarting them from the web) — without touching their source
   code. Each one then publishes, besides its normal data, its own write *and read* throughput (instantaneous
   bytes/s) on two reserved internal Fast DDS topics — so the real bandwidth on the **consumer** side (e.g.
   `retina` subscribed to the media plane) is obtained too, without instrumenting the component.
2. For each DDS domain found among the components, the launcher starts (idempotently, like Webots/rcnode) an
   instance of `netmon/dds_stats_bridge/build/dds_stats_bridge --domain N --out /tmp/robocomp_netmon/dds_stats_dN.json`.
   This standalone C++ binary subscribes to both statistics topics, resolves writer/reader GUID → real topic name
   through normal DDS discovery, and dumps `{"<topic>": bytes_per_second, ...}` to JSON every 1 s (atomic write).
   If the same topic has a producer and a consumer active at once, they share a single key — the JSON cannot tell
   "published" from "received", nor separate several consumers of the same topic (both are known limits, not
   bugs; see the comment in `main.cpp`).
3. `netmon/server.py` merges every `dds_stats_d*.json` it finds and exposes them as `dds_bw` in `GET /api/state`.
4. `app.js` adds up the `dds_bw` of the topics of each DDS edge of the graph and animates it like the RPC edges
   (thickness + green with traffic).

### Building the binary (once, on the machine where it will run)

```bash
cd netmon/dds_stats_bridge
mkdir -p build && cd build
cmake .. && make -j8
```

It needs Fast DDS installed (`fastdds`/`fastcdr` through `find_package`) **and** a checkout of the Fast-DDS source
matching the installed version, because the code generated from `fastdds/statistics/types.idl` is not shipped in
the public headers — it is taken directly from `<checkout>/src/cpp/statistics/types/`. The path is set with the CMake
variable `FASTDDS_SRC_TREE` (default `$HOME/software/Fast-DDS`).

If the binary is missing from `dds_stats_bridge/build/`, the launcher detects it, warns on the console
(`dds_stats_bridge binary missing...`) and keeps working normally — just without DDS bandwidth (the rest of the map
is unaffected).

### Notes / limitations

- A component with neither `[DDS] Domain = N` nor `[Media] domain_id = N` in its config simply does not take part
  (it gets no environment variable and does not count when deciding which domains need a bridge).
- A component can have several DDS topics (`RGBTopic`/`DepthTopic`, etc.) — any `[DDS]` key ending in `Topic`, or
  `[Media]` key ending in `_topic`, counts.
- A media-plane consumer that resolves its domain/topic **at runtime** through the DSR graph instead of declaring it
  in its `etc/*.toml` (see `topology.py`) has no static `[Media] domain_id` to read — it gets no environment
  variable and does not show as an edge in the graph. This is a limit of "reading configs cold", not something the
  bridge can fix on its own.
- There is no throughput until the real *DataWriter* writes at least one sample: if the component is not publishing
  yet (e.g. waiting for a physical camera), its topic simply does not appear in `dds_bw` — not a bridge failure.
- The value is **instantaneous** (bytes since that writer's last sample / elapsed time), not a smoothed average —
  it can oscillate between 1 s readings.

---

## Configuration

The launchers read a TOML with the list of components:

```toml
[general]
start_webots = false
start_rcnode = true

[[components]]
name     = "bridge"
cwd      = "~/robocomp/AiGB_UEx_Navigation_v2/webots-bridge"
cmd      = "bin/Webots2Robocomp etc/config_husky"
ice_name = "webots2robocomp:tcp -h localhost -p 10006"   # optional: proxy for the state ping
```

- `ice_name` is used **only** for the launcher's state ping. The **map topology is NOT derived from it**, but from
  each component's real `etc/config` (`Endpoints.*` = implements, `Proxies.*` = requires, `*Prefix`/`*Topic` = ICE
  pub/sub, `[DDS] Domain`/`*Topic` = DDS pub).
- **ICE identities are lowercase** (`lidar3d`, `webots2robocomp`); the ping is case-sensitive. The port is the one of
  the server's `Endpoints.<Iface>`.
- Publishers without an RPC endpoint (e.g. `python_xbox_controller`) use `ice_name = ""`.
- **Cognitive agents** are grouped into a `DSR·dN` sphere by the `[Agent] domain` of their `config.toml`. Several
  domains ⇒ several spheres.

---

## Files

| File | Role |
|------|------|
| `subcognitive.py`, `cognitive.py` | thin wrappers (in `utils/`) |
| `netmon/launcher.py` | launch + collector + registry + monitor-role takeover + Webots/rcnode/dds_stats_bridge |
| `netmon/registry.py` | shared `/tmp` registry + monitor lock |
| `netmon/topology.py` | graph from the configs (RPC, ICE pub/sub, DDS, DSR spheres) |
| `netmon/bandwidth.py` | ICE loopback capture (raw AF_PACKET) + per-connection attribution |
| `netmon/server.py` | web server (http.server) + JSON API (including `dds_bw`) |
| `netmon/static/` | vis-network frontend (`index.html`, `app.js`) |
| `netmon/dds_stats_bridge/` | standalone C++ binary: real Fast DDS statistics → JSON |

API: `GET /api/topology` (static graph) · `GET /api/state` (state + `edges_bw` + `dds_bw`, 1×/s) ·
`GET /api/logs?name=&stream=&lines=` · `POST /api/action` (`{action, name}`).

---

## Common problems

- **Everything shows `Up (ICE ✗)` in yellow** → the process is alive but the ICE ping fails: check that the TOML's
  `ice_name` (lowercase identity + port of `Endpoints.<Iface>`) matches the component's real `etc/config`.
- **A component shows `Stopped`** → it died on its own (see `~/.local/logs/<name>.err`); typically missing hardware
  (camera, `/dev/input/js0`) or a port still held by an old instance.
- **`bw: no capture permission`** → apply the `setcap` above (it only affects ICE, not DDS).
- **The web does not show up** → check that no old process holds the port; the live lock is in
  `/tmp/robocomp_netmon/monitor.lock`.
- **No bandwidth on a DDS edge** → either the `dds_stats_bridge` binary is not built (check the launcher console at
  start-up), or the component has not published any real sample yet (e.g. camera not connected), or its config
  never actually enabled DDS (look in `~/.local/logs/<name>.out` for `PublishDDS not found, using default false` or
  similar — a problem of the component's own config, not of the bridge).
