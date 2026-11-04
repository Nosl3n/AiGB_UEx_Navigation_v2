#!/usr/bin/env python3
"""Drive the robot along a FIXED route of goals through the DSR graph, without the GUI — reproducible runs.

    python3 run_route.py                       # the default orchard route (all four corridors, serpentine)
    python3 run_route.py --route "1.25,-9.5 1.25,9 3.75,9" --tol 0.5 --timeout 90

Same mechanism as send_goal.py (a `goal` node of type `target` under the world frame, linked robot
-goto_action-> goal), but the goal MOVES to the next waypoint once the robot is within --tol of it, or once
--timeout seconds have passed for that waypoint (logged as a timeout, and the route goes on). Needs the
controller armed without a click: Controller.ArmOnStart = true (controller/etc/config_husky.toml).

The default route is for webots-agri/worlds/husky_apple.wbt (rows at x = -5 ... 5, 2.5 m
apart, corridors at x = +-1.25, +-3.75), starting from the robot's spawn in the south headland; see DEFAULT_ROUTE
for why it is shaped around the workers. Run it a few seconds after cognitive.py, once the agents are Operating.

Agent id 98, like send_goal.py (CONCEPT_AGENT_RECIPE.md registry): run one of them at a time.
"""
import argparse
import math
import signal
import time

from pydsr import DSRGraph, Edge, Node, inner_api, rt_api

AGENT_ID = 98
# Out and back along three corridors, changing corridor through the SOUTH headland. It keeps clear of where the
# two workers walk (they are 0.8 m forbidden discs to the planner, and rightly so): worker 1 paces the x = 1.25
# corridor (y -4 .. 3) and worker 2 the north headland (y = 9.5, x -4 .. 4). The turn at y = 8.0 sits between the
# row ends (last trunk y = 6.88, + its 0.6 m disc) and worker 2's band; the x = 1.25 corridor is still seen
# from its neighbours.
DEFAULT_ROUTE = "3.75,-9.5 3.75,8 3.75,-9.5 -1.25,-9.5 -1.25,8 -1.25,-9.5 -3.75,-9.5 -3.75,8 -3.75,-9.5"


def find_world(g):
    for t in ("field", "room"):
        nodes = g.get_nodes_by_type(t)
        if nodes:
            return nodes[0]
    return None


def robot_xy(g, world, robot):
    """Robot position in the world frame, or None while the transform cannot be resolved.

    ★A FRESH inner_api EVERY CALL. An InnerEigenAPI caches its ts==0 transforms and invalidates them from Qt
    queued slots (active_inference/CLAUDE.md, DSR graph thread-safety); this script has no Qt event loop, so one
    long-lived instance kept answering the robot's pose of the moment it was first asked — measured: the robot
    stood 0.23 m from a waypoint while the script still read it metres away, and every waypoint "timed out"."""
    try:
        v = inner_api(g).transform(world.name, robot.name)
        if v is not None and len(v) >= 2 and all(math.isfinite(c) for c in v[:2]):
            return float(v[0]), float(v[1])
    except Exception:
        pass
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default=DEFAULT_ROUTE, help='waypoints "x,y x,y ..." in world metres (ENU)')
    ap.add_argument("--tol", type=float, default=0.6, help="reached when the robot is this close (m)")
    ap.add_argument("--timeout", type=float, default=120.0, help="seconds allowed per waypoint")
    a = ap.parse_args()
    route = [tuple(float(c) for c in wp.split(",")) for wp in a.route.split()]

    g = DSRGraph(0, "run_route", AGENT_ID)
    deadline = time.time() + 20
    world = robots = None
    while time.time() < deadline:
        world, robots = find_world(g), g.get_nodes_by_type("robot")
        if world is not None and robots:
            break
        time.sleep(0.5)
    if world is None or not robots:
        raise SystemExit("no world frame (field/room) or robot node in the graph — is the stack up?")
    robot = robots[0]

    stale = [n for n in g.get_nodes_by_type("target") if n.name == "goal" or n.agent_id == AGENT_ID]
    for n in stale:
        g.delete_node(n.id)
    if stale:
        time.sleep(0.5)

    rt = rt_api(g)
    x, y = route[0]
    goal_id = g.insert_node(Node(AGENT_ID, "target", "goal"))
    rt.insert_or_assign_edge_RT(world, goal_id, [x, y, 0.0], [0.0, 0.0, 0.0])
    g.insert_or_assign_edge(Edge(goal_id, robot.id, "goto_action", AGENT_ID))

    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    t_route = time.time()
    reached = 0
    for k, (x, y) in enumerate(route):
        t0 = time.time()
        print(f"[route] waypoint {k + 1}/{len(route)} -> ({x:.2f}, {y:.2f})", flush=True)
        outcome = "timeout"
        tick = 0
        p = None
        while not stop and time.time() - t0 < a.timeout:
            # Re-stamp at 10 Hz: the controller reads the target at the scan stamp (see send_goal.py).
            rt.insert_or_assign_edge_RT(world, goal_id, [x, y, 0.0], [0.0, 0.0, 0.0], int(time.time() * 1000))
            if tick % 5 == 0:                       # the robot's pose at 2 Hz (a fresh inner_api each time)
                p = robot_xy(g, world, robot)
                if p is not None and math.hypot(p[0] - x, p[1] - y) < a.tol:
                    outcome = "reached"
                    break
            if tick % 100 == 0 and p is not None:   # where it is, every 10 s
                print(f"[route]   robot at ({p[0]:.2f}, {p[1]:.2f}), {math.hypot(p[0] - x, p[1] - y):.1f} m to go",
                      flush=True)
            tick += 1
            time.sleep(0.1)
        if stop:
            break
        reached += outcome == "reached"
        print(f"[route] waypoint {k + 1} {outcome} after {time.time() - t0:.1f} s", flush=True)
    print(f"[route] done: {reached}/{len(route)} waypoints reached in {time.time() - t_route:.1f} s", flush=True)
    g.delete_node(goal_id)
    time.sleep(0.5)


if __name__ == "__main__":
    main()
