#!/usr/bin/env python3
"""
prepare_human_motions.py — turn raw motion-capture clips into the granjero's ready-to-play motion library.

Source clips: the CMU Graphics Lab Motion Capture Database in Bruce Hahne's BVH conversion (cgspeed), the same
skeleton as Webots' own walk.bvh (projects/humans/skin_animated_humans/motions). CMU places no restrictions on the
use of the data; please credit "mocap.cs.cmu.edu" (see controllers/granjero/motions/README.md).

For every clip in CLIPS this script (all decisions are computed from the data, not hand-tuned per clip):
  1. cuts the requested frame segment (frame 0 of a cgspeed file is a T-pose and is never played);
  2. HEADING-NORMALISES it: every subject was recorded facing a different way, so the root's heading is removed —
     a constant one (the median heading of the frames where the subject stands upright) for actions, or the
     time-varying one for turn-in-place clips (whose turning is then done by the Robot node, at the measured rate);
  3. treats the root translation according to the clip's role:
       vertical   — horizontal motion removed, vertical kept (idle, harvest)
       full       — horizontal motion kept, relative to the standing reference frame (pick up, sit, stand up)
       locomotion — horizontal travel removed and MEASURED (walk): the body is moved by the controller at the
                    clip's own speed, so the feet do not slide
       turn       — horizontal motion removed, heading progression MEASURED (rad/s)
  4. for looping clips, finds the most seamless loop (the pair of frames, at least min_loop apart, whose poses are
     closest), so the cycle never pops;
  5. writes <name>.bvh (T-pose frame + segment) and motions.json with frame counts, loop range, reference frame,
     speed / yaw rate and play mode.

Usage:  prepare_human_motions.py <dir with the downloaded CMU .bvh files> <output dir>
"""
import json
import math
import os
import sys

import numpy as np

WEBOTS_WALK = "/usr/local/webots/projects/humans/skin_animated_humans/motions/walk.bvh"
UNIT = 1.0 / 20.0   # BVH units -> metres (Webots' bvh_util default scale for this dataset)

# name: source file, frame segment [a, b], play mode, root treatment, options
CLIPS = {
    "idle":       dict(src="111_28.bvh", seg=(40, 1900), mode="loop", root="vertical", min_loop=600),
    "walk":       dict(src=WEBOTS_WALK, seg=(1, 327), mode="loop", root="locomotion", min_loop=100),
    "turn_left":  dict(src="69_16.bvh", seg=(60, 840), mode="loop", root="turn", min_loop=200),
    "turn_right": dict(src="69_18.bvh", seg=(60, 800), mode="loop", root="turn", min_loop=200),
    "pick":       dict(src="111_17.bvh", seg=(1, 420), mode="once", root="full"),
    "harvest":    dict(src="14_20.bvh", seg=(1740, 2600), mode="loop", root="vertical", min_loop=250),
    "stand_up":   dict(src="111_06.bvh", seg=(700, 1320), mode="once", root="full"),
    "sit_down":   dict(src="111_06.bvh", seg=(700, 1320), mode="once", root="full", reverse=True),
}


# ── BVH I/O ─────────────────────────────────────────────────────────────────────────────────────────────────────
def read_bvh(path):
    text = open(path).read().replace("\r", "")
    head, motion = text.split("MOTION", 1)
    lines = [l for l in motion.strip().split("\n") if l.strip()]
    ft = float(lines[1].split()[-1])
    data = np.array([[float(v) for v in l.split()] for l in lines[2:]])
    joints, stack = [], []
    for l in head.split("\n"):
        t = l.split()
        if not t:
            continue
        if t[0] in ("ROOT", "JOINT", "End"):
            joints.append({"name": t[1] if t[0] != "End" else "_end", "parent": stack[-1] if stack else -1,
                           "ch": [], "offset": None})
            cur = len(joints) - 1
        elif t[0] == "{":
            stack.append(cur)
        elif t[0] == "}":
            stack.pop()
        elif t[0] == "OFFSET":
            joints[stack[-1]]["offset"] = np.array([float(v) for v in t[1:4]])
        elif t[0] == "CHANNELS":
            joints[stack[-1]]["ch"] = t[2:]
    return head.rstrip() + "\n", joints, data, ft


def write_bvh(path, head, data, ft):
    with open(path, "w") as f:
        f.write(head)
        f.write("MOTION\n")
        f.write(f"Frames: {len(data)}\n")
        f.write(f"Frame Time: {ft:.7f}\n")
        for row in data:
            f.write(" ".join(f"{v:.5f}" for v in row) + "\n")


# ── rotations (BVH: Y up; root channels Xpos Ypos Zpos Zrot Yrot Xrot  ⇒  R = Rz·Ry·Rx) ─────────────────────────
def R_axis(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return {"X": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "Y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "Z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def root_R(row):
    return R_axis("Z", row[3]) @ R_axis("Y", row[4]) @ R_axis("X", row[5])


def zyx_from_R(R):
    b = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    a = math.atan2(R[1, 0], R[0, 0])
    c = math.atan2(R[2, 1], R[2, 2])
    return math.degrees(a), math.degrees(b), math.degrees(c)


def heading(row):
    f = root_R(row) @ np.array([0.0, 0.0, 1.0])
    return math.atan2(f[0], f[2])          # from +Z toward +X; + = turning left once in Webots (see controller)


def circ_median(angles):
    s = np.median(np.sin(angles)), np.median(np.cos(angles))
    return math.atan2(*s)


def pose_distance(a, b):
    """Joint-rotation distance between two frames (root heading and translation excluded)."""
    d = (a[6:] - b[6:] + 180.0) % 360.0 - 180.0
    return float(np.sum(d * d) + 400.0 * (a[1] - b[1]) ** 2)   # + the hips' height (BVH units)


def best_loop(frames, min_len):
    """Most seamless (a, b) with b - a >= min_len, searched on a coarse grid then refined."""
    n = len(frames)
    best = (float("inf"), 0, n - 1)
    step = max(1, n // 200)
    for a in range(0, n - min_len, step):
        for b in range(a + min_len, n, step):
            d = pose_distance(frames[a], frames[b])
            if d < best[0]:
                best = (d, a, b)
    _, a0, b0 = best
    for a in range(max(0, a0 - step), min(n, a0 + step + 1)):
        for b in range(max(a + min_len, b0 - step), min(n, b0 + step + 1)):
            d = pose_distance(frames[a], frames[b])
            if d < best[0]:
                best = (d, a, b)
    return best


# ── one clip ────────────────────────────────────────────────────────────────────────────────────────────────────
def prepare(name, spec, src_dir, out_dir):
    path = spec["src"] if os.path.isabs(spec["src"]) else os.path.join(src_dir, spec["src"])
    head, joints, data, ft = read_bvh(path)
    tpose = data[0].copy()
    a, b = spec["seg"]
    seg = data[a:b + 1].copy()
    if spec.get("reverse"):
        seg = seg[::-1].copy()
    hips = seg[:, 1]
    upright = hips >= 0.95 * hips.max()
    # the standing reference: first upright frame (last one for a reversed clip, which ends standing)
    if spec.get("reverse"):
        ref = int(np.nonzero(upright)[0][-1])
    else:
        ref = int(np.nonzero(upright)[0][0])
    hd = np.array([heading(r) for r in seg])
    info = {"file": f"{name}.bvh", "mode": spec["mode"], "root": spec["root"], "frame_time": ft,
            "frames": len(seg), "ref_frame": ref + 1}

    if spec["root"] == "turn":
        unwrapped = np.unwrap(hd)
        # smooth the heading (the stepping wobbles it) and remove it frame by frame
        k = 25
        smooth = np.convolve(np.pad(unwrapped, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), mode="same")[k:-k]
        info["yaw_rate"] = float((smooth[-1] - smooth[0]) / (len(seg) * ft))
        remove = smooth
    else:
        h0 = circ_median(hd[upright])
        remove = np.full(len(seg), h0)
        info["heading_removed_deg"] = math.degrees(h0)

    xz_ref = seg[ref, [0, 2]].copy()
    if spec["root"] == "locomotion":
        travel = np.linalg.norm(seg[-1, [0, 2]] - seg[0, [0, 2]]) * UNIT
        info["speed"] = float(travel / (len(seg) * ft))
    for i, r in enumerate(seg):
        Ry = R_axis("Y", -math.degrees(remove[i]))
        z, y, x = zyx_from_R(Ry @ root_R(r))
        r[3], r[4], r[5] = z, y, x
        if spec["root"] == "full":
            d = Ry @ np.array([r[0] - xz_ref[0], 0.0, r[2] - xz_ref[1]])
            r[0], r[2] = xz_ref[0] + d[0], xz_ref[1] + d[2]
        else:
            r[0], r[2] = xz_ref
    if spec["mode"] == "loop":
        d, la, lb = best_loop(seg, spec["min_loop"])
        info["loop"] = [la + 1, lb + 1]
        info["loop_mismatch"] = round(d, 1)
    write_bvh(os.path.join(out_dir, f"{name}.bvh"), head, np.vstack([tpose, seg]), ft)
    return info


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    src_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    lib = {}
    for name, spec in CLIPS.items():
        lib[name] = prepare(name, spec, src_dir, out_dir)
        print(f"{name:10s} {json.dumps(lib[name])}")
    with open(os.path.join(out_dir, "motions.json"), "w") as f:
        json.dump(lib, f, indent=2)


if __name__ == "__main__":
    main()
