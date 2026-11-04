#!/usr/bin/env python3
"""Send the controller a navigation goal through the DSR graph, without the GUI.

    python3 send_goal.py X Y [--hold SECONDS]

Creates a node `goal` (type `target`) placed by an RT edge under the world frame (`field` outdoors,
`room` indoors) at (X, Y) metres, and links it robot -goto_action-> goal: what the controller reads as its
target (controller etc/config_husky.toml, Target.EdgeType). On exit (Ctrl-C or --hold elapsed) the goal
node is deleted, which also removes its edges.

Agent id 98 (CONCEPT_AGENT_RECIPE.md registry). Run one at a time.
"""
import argparse
import signal
import time

from pydsr import DSRGraph, Edge, Node, rt_api

AGENT_ID = 98


def find_world(g):
    for t in ("field", "room"):
        nodes = g.get_nodes_by_type(t)
        if nodes:
            return nodes[0]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--hold", type=float, default=0.0, help="seconds to keep the goal (0 = until Ctrl-C)")
    a = ap.parse_args()

    g = DSRGraph(0, "send_goal", AGENT_ID)
    deadline = time.time() + 10
    world = robots = None
    while time.time() < deadline:
        world, robots = find_world(g), g.get_nodes_by_type("robot")
        if world is not None and robots:
            break
        time.sleep(0.5)
    if world is None or not robots:
        raise SystemExit("no world frame (field/room) or robot node in the graph — is the stack up?")
    robot = robots[0]

    # Clear every leftover goal (a crashed run leaves its node behind, and the DSR then RENAMES a new node
    # whose name is taken — so the stale one would keep the name and none of the edges).
    stale = [n for n in g.get_nodes_by_type("target") if n.name == "goal" or n.agent_id == AGENT_ID]
    for n in stale:
        g.delete_node(n.id)
    if stale:
        print(f"removed {len(stale)} stale goal node(s)", flush=True)
        time.sleep(0.5)

    goal_id = g.insert_node(Node(AGENT_ID, "target", "goal"))
    rt = rt_api(g)
    rt.insert_or_assign_edge_RT(world, goal_id, [a.x, a.y, 0.0], [0.0, 0.0, 0.0])
    g.insert_or_assign_edge(Edge(goal_id, robot.id, "goto_action", AGENT_ID))
    print(f"goal ({a.x:.2f}, {a.y:.2f}) in '{world.name}' sent to '{robot.name}' (node id {goal_id})", flush=True)

    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    t0 = time.time()
    # Re-stamp the goal's RT edge at 10 Hz. The controller asks for the target AT THE SCAN STAMP with an
    # interpolated RT query, and a ring holding one sample from the moment of creation has nothing to
    # interpolate at any later instant — the goal would be invisible. A live producer re-publishes anyway.
    while not stop and (a.hold <= 0 or time.time() - t0 < a.hold):
        rt.insert_or_assign_edge_RT(world, goal_id, [a.x, a.y, 0.0], [0.0, 0.0, 0.0], int(time.time() * 1000))
        time.sleep(0.1)
    g.delete_node(goal_id)
    time.sleep(0.5)   # let the deletion reach the peers
    print("goal removed", flush=True)


if __name__ == "__main__":
    main()
