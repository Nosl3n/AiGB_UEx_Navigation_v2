#!/usr/bin/env python3
"""
granjero.py — keyboard-driven farmer ('granjero', FarmerSkin / Robert) with motion-captured actions.

Controls (click inside the Webots 3D view first so it receives keyboard focus):
  UP / DOWN            walk forward / backward
  LEFT / RIGHT         turn left / right (stepping on the spot, or while walking)
  SHIFT + LEFT/RIGHT   step sideways
  R                    recoger   — bend down and pick something up from the ground
  C                    recolectar — reach up into the tree and drop the fruit in a basket (a few cycles)
  S                    sentarse / levantarse — sit down on the ground, or get up again

Animation (see motions/README.md and webots-agri/tools/prepare_human_motions.py):
  * every motion is a motion-capture clip, heading-normalised and with its seamless loop pre-computed;
  * the body moves AT THE CLIP'S OWN SPEED (walk: measured m/s, turn: measured rad/s) and the clip is played
    faster or slower in proportion, so the feet never slide;
  * switching motion cross-fades every bone (quaternion slerp), so nothing pops;
  * the feet follow the ground: the terrain's ElevationGrid is read and the farmer stands on it.

Environment (optional, for tests):
  GRANJERO_LOG=<file>   log every motion change, the position and any error to <file>
  GRANJERO_DEMO=1       play a scripted tour of every motion once at start (no keyboard needed)
  GRANJERO_SNAP_DIR=<d> save a screenshot of the 3D view in <d> during each motion (tests)
"""

import ctypes
import json
import math
import optparse
import os
import time
import traceback

from controller import Keyboard, Supervisor

LIB = "/usr/local/webots/projects/humans/skin_animated_humans/libraries/bvh_util/libbvh_util.so"
HERE = os.path.dirname(os.path.abspath(__file__))
MOTIONS = os.path.join(HERE, "motions")
SKIP_BONES = {15, 16, 17, 24, 25, 26}   # finger bones the BVH skeleton does not drive (Webots' own example)


# ── quaternions (w, x, y, z) ↔ Webots axis-angle (x, y, z, angle) ───────────────────────────────────────────────
def aa_to_q(a):
    x, y, z, ang = a
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    s = math.sin(ang / 2.0) / n
    return (math.cos(ang / 2.0), x * s, y * s, z * s)


def q_to_aa(q):
    w, x, y, z = q
    w = max(-1.0, min(1.0, w))
    ang = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    return [x / s, y / s, z / s, ang]


def slerp(q0, q1, t):
    d = sum(a * b for a, b in zip(q0, q1))
    if d < 0.0:
        q1, d = tuple(-v for v in q1), -d
    if d > 0.9995:
        r = tuple(a + t * (b - a) for a, b in zip(q0, q1))
    else:
        th = math.acos(d)
        s0, s1 = math.sin((1 - t) * th) / math.sin(th), math.sin(t * th) / math.sin(th)
        r = tuple(s0 * a + s1 * b for a, b in zip(q0, q1))
    n = math.sqrt(sum(v * v for v in r)) or 1.0
    return tuple(v / n for v in r)


def lerp3(a, b, t):
    return [x + t * (y - x) for x, y in zip(a, b)]


# ── one motion-capture clip, retargeted to the skin by Webots' bvh_util ─────────────────────────────────────────
class Clip:
    def __init__(self, lib, skin, name, meta, scale=20.0):
        self.lib, self.skin, self.name, self.meta = lib, skin, name, meta
        self.motion = lib.wbu_bvh_read_file(os.path.join(MOTIONS, meta["file"]).encode())
        if not self.motion:
            raise RuntimeError(f"cannot read {meta['file']}")
        self.frames = lib.wbu_bvh_get_frame_count(self.motion) - 1   # playable frames 1..frames
        self.dt = meta["frame_time"]
        names = {lib.wbu_bvh_get_joint_name(self.motion, j).decode(): j
                 for j in range(lib.wbu_bvh_get_joint_count(self.motion))}
        self.map = []
        self.root_bone = -1
        for i in range(skin.getBoneCount()):
            bone = skin.getBoneName(i)
            if bone == "Hips":
                self.root_bone = i
            if i in SKIP_BONES or bone not in names:
                continue
            j = names[bone]
            self.map.append((i, j))
            lib.wbu_bvh_set_model_t_pose(self.motion, skin.getBoneOrientation(i, True), j, True)
            lib.wbu_bvh_set_model_t_pose(self.motion, skin.getBoneOrientation(i, False), j, False)
        lib.wbu_bvh_set_scale(self.motion, scale)
        # ★CAPTURE THE BVH T-POSE. bvh_util takes each joint's reference ("T") orientation from FRAME 0, and only when
        # that joint's rotation is queried while the motion sits on frame 0 (bvh_util.c, wbu_bvh_get_joint_rotation).
        # Never querying it leaves the identity as the reference — but this dataset's T-pose has seven joints at up to
        # ±21°, so every pose came out offset by them: the legs closed until they crossed when standing.
        lib.wbu_bvh_goto_frame(self.motion, 0)
        for _, j in self.map:
            lib.wbu_bvh_get_joint_rotation(self.motion, j)
        self.ref_root = self._root(meta.get("ref_frame", 1))
        self.loop = meta.get("loop")

    def _root(self, f):
        self.lib.wbu_bvh_goto_frame(self.motion, int(f))
        t = self.lib.wbu_bvh_get_root_translation(self.motion)
        return [t[0], t[1], t[2]]

    def _bones(self, f):
        self.lib.wbu_bvh_goto_frame(self.motion, int(f))
        out = {}
        for i, j in self.map:
            r = self.lib.wbu_bvh_get_joint_rotation(self.motion, j)
            out[i] = aa_to_q((r[0], r[1], r[2], r[3]))
        return out

    def pose(self, frame, skin_root_rest):
        """Bones (quaternions) and root position at a FRACTIONAL frame (interpolated between the two neighbours)."""
        f0 = max(1, min(self.frames, int(math.floor(frame))))
        f1 = max(1, min(self.frames, f0 + 1))
        t = max(0.0, min(1.0, frame - math.floor(frame)))
        b0, b1 = self._bones(f0), self._bones(f1)
        bones = {i: slerp(b0[i], b1[i], t) for i in b0}
        r = lerp3(self._root(f0), self._root(f1), t)
        root = [skin_root_rest[k] + r[k] - self.ref_root[k] for k in range(3)]
        return bones, root

    def wrap(self, frame):
        """Keep a looping clip's cursor inside its seamless loop; clamp a one-shot clip."""
        if self.loop:
            a, b = self.loop
            span = b - a
            while frame >= b:
                frame -= span
            while frame < a:
                frame += span
            return frame
        return max(1.0, min(float(self.frames), frame))


# ── the ground under the farmer: the terrain's ElevationGrid (FieldTerrain's FIELD_GRID), bilinear ─────────────
class Ground:
    def __init__(self, sup):
        self.grid = None
        root = sup.getRoot().getField("children")
        for k in range(root.getCount()):
            n = root.getMFNode(k)
            if n.getTypeName() != "FieldTerrain":
                continue
            g = n.getFromProtoDef("FIELD_GRID")
            if g is None:
                continue
            size = n.getField("size").getSFVec2f()
            tr = n.getField("translation").getSFVec3f()
            self.grid = g
            self.height = g.getField("height")
            self.nx = g.getField("xDimension").getSFInt32()
            self.ny = g.getField("yDimension").getSFInt32()
            self.dx = g.getField("xSpacing").getSFFloat()
            self.dy = g.getField("ySpacing").getSFFloat()
            self.x0, self.y0, self.z0 = tr[0] - 0.5 * size[0], tr[1] - 0.5 * size[1], tr[2]
            print(f"[granjero] following the ground of '{n.getTypeName()}' ({self.nx} x {self.ny} heights)")
            break
        if self.grid is None:
            print("[granjero] no FieldTerrain in the world: walking on z = 0")

    def z(self, x, y):
        if self.grid is None:
            return 0.0
        fx, fy = (x - self.x0) / self.dx, (y - self.y0) / self.dy
        i, j = int(math.floor(fx)), int(math.floor(fy))
        if not (0 <= i < self.nx - 1 and 0 <= j < self.ny - 1):
            return self.z0
        tx, ty = fx - i, fy - j
        h = lambda a, b: self.height.getMFFloat(b * self.nx + a)
        return self.z0 + ((h(i, j) * (1 - tx) + h(i + 1, j) * tx) * (1 - ty)
                          + (h(i, j + 1) * (1 - tx) + h(i + 1, j + 1) * tx) * ty)


class Granjero(Supervisor):
    def __init__(self):
        super().__init__()
        self.step_ms = int(self.getBasicTimeStep()) or 32
        p = optparse.OptionParser()
        p.add_option("--speed", type=float, default=0.0, help="walking speed m/s (0 = the clip's own, 1.17)")
        p.add_option("--turn-speed", type=float, default=0.0, help="turning speed rad/s (0 = the clip's own)")
        p.add_option("--skin", type=str, default="granjero_skin")
        p.add_option("--blend", type=float, default=0.25, help="cross-fade time between motions (s)")
        p.add_option("--harvest-cycles", type=int, default=2)
        opt, _ = p.parse_args()
        self.opt = opt

        self.kb = self.getKeyboard()
        self.kb.enable(self.step_ms)
        self.node = self.getSelf()
        self.f_trans = self.node.getField("translation")
        self.f_rot = self.node.getField("rotation")
        self.skin = self.getDevice(opt.skin)
        self.ground = Ground(self)

        lib = ctypes.CDLL(LIB)
        lib.wbu_bvh_read_file.argtypes = [ctypes.c_char_p]
        lib.wbu_bvh_read_file.restype = ctypes.c_void_p
        for fn, args, res in [("wbu_bvh_get_joint_count", [ctypes.c_void_p], ctypes.c_int),
                              ("wbu_bvh_get_joint_name", [ctypes.c_void_p, ctypes.c_int], ctypes.c_char_p),
                              ("wbu_bvh_get_frame_count", [ctypes.c_void_p], ctypes.c_int),
                              ("wbu_bvh_goto_frame", [ctypes.c_void_p, ctypes.c_int], ctypes.c_bool),
                              ("wbu_bvh_set_scale", [ctypes.c_void_p, ctypes.c_double], None),
                              ("wbu_bvh_get_root_translation", [ctypes.c_void_p], ctypes.POINTER(ctypes.c_double)),
                              ("wbu_bvh_get_joint_rotation", [ctypes.c_void_p, ctypes.c_int],
                               ctypes.POINTER(ctypes.c_double)),
                              ("wbu_bvh_set_model_t_pose", [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double),
                                                            ctypes.c_int, ctypes.c_bool], None)]:
            getattr(lib, fn).argtypes = args
            getattr(lib, fn).restype = res
        meta = json.load(open(os.path.join(MOTIONS, "motions.json")))
        self.clips = {name: Clip(lib, self.skin, name, m) for name, m in meta.items()}
        rb = self.clips["idle"].root_bone
        self.root_bone = rb
        # getBonePosition returns a C POINTER (not a list): read exactly three values — list() on it never ends.
        srp = self.skin.getBonePosition(rb, False) if rb >= 0 else None
        self.skin_root_rest = [srp[0], srp[1], srp[2]] if srp is not None else [0.0, 0.0, 0.0]

        self.walk_speed = opt.speed or self.clips["walk"].meta["speed"]
        self.turn_speed = opt.turn_speed or abs(self.clips["turn_left"].meta["yaw_rate"])
        self.state = "idle"          # idle | walk | turn_left | turn_right | pick | harvest | sit_down | sitting | stand_up
        self.clip = self.clips["idle"]
        self.frame = float(self.clip.loop[0])
        self.shown = None            # (bones, root) currently on the skin
        self.blend_from, self.blend_t = None, 1.0
        self.cycles_left = 0
        self.prev_keys = set()
        self.log_path = os.environ.get("GRANJERO_LOG", "")
        self.log(f"start: {len(self.clips)} clips, walk {self.walk_speed:.2f} m/s, turn {self.turn_speed:.2f} rad/s, "
                 f"ground {'FieldTerrain' if self.ground.grid else 'flat'}")
        # Scripted tour (GRANJERO_DEMO=1): (seconds, held keys, pressed key) — the same inputs a person would give.
        U, L, Rt = Keyboard.UP, Keyboard.LEFT, Keyboard.RIGHT
        self.demo = [] if os.environ.get("GRANJERO_DEMO", "") != "1" else [
            (2.0, set(), None), (4.0, {U}, None), (1.5, set(), None), (3.0, {L}, None), (3.0, {Rt}, None),
            (1.0, set(), None), (6.0, set(), ord("R")), (12.0, set(), ord("C")), (8.0, set(), ord("S")),
            (4.0, set(), None), (8.0, set(), ord("S")), (3.0, {U, L}, None), (2.0, set(), None)]
        self.demo_t = 0.0
        self.last_state = None
        self.state_t = 0.0
        self.snap_dir = os.environ.get("GRANJERO_SNAP_DIR", "")
        self.snapped = set()

    def log(self, msg):
        line = f"[granjero] {msg}"
        print(line)
        if self.log_path:
            with open(self.log_path, "a") as f:
                f.write(f"{time.strftime('%H:%M:%S')} {line}\n")

    # ── motion switching ────────────────────────────────────────────────────────────────────────────────────────
    def play(self, name, frame=None):
        if self.clip.name == name and frame is None:
            return
        self.blend_from, self.blend_t = self.shown, 0.0
        self.clip = self.clips[name]
        self.frame = float(frame if frame is not None else (self.clip.loop[0] if self.clip.loop else 1))
        self.state = name

    def show(self, dt):
        bones, root = self.clip.pose(self.frame, self.skin_root_rest)
        if self.blend_from is not None and self.blend_t < 1.0:
            self.blend_t = min(1.0, self.blend_t + dt / max(1e-3, self.opt.blend))
            w = self.blend_t * self.blend_t * (3 - 2 * self.blend_t)   # smoothstep
            fb, fr = self.blend_from
            bones = {i: slerp(fb[i], q, w) if i in fb else q for i, q in bones.items()}
            root = lerp3(fr, root, w)
        for i, q in bones.items():
            self.skin.setBoneOrientation(i, q_to_aa(q), False)
        if self.root_bone >= 0:
            self.skin.setBonePosition(self.root_bone, root, False)
        self.shown = (bones, root)

    # ── keyboard: held keys (motion) and pressed edges (actions) ─────────────────────────────────────────────────
    def demo_keys(self, dt):
        """The scripted tour's input at this instant: held keys and the key pressed on entering a step."""
        t, start = self.demo_t, 0.0
        self.demo_t += dt
        for dur, held, key in self.demo:
            if t < start + dur:
                pressed = {key} if key is not None and t < start + dt else set()
                return held | pressed, False, pressed
            start += dur
        return set(), False, set()

    def read_keys(self):
        held, k = set(), self.kb.getKey()
        while k != -1:
            held.add((k & Keyboard.KEY, bool(k & Keyboard.SHIFT)))
            k = self.kb.getKey()
        base = {b for b, _ in held}
        shift = any(s for _, s in held)
        pressed = base - self.prev_keys
        self.prev_keys = base
        return base, shift, pressed

    def yaw(self):
        r = self.f_rot.getSFRotation()
        return math.atan2(math.sin(r[3]) * r[2] * 1.0, math.cos(r[3])) if abs(r[2]) > 0.5 else 0.0

    def run(self):
        print("[granjero] flechas: caminar/girar · SHIFT+flechas: paso lateral · R: recoger · C: recolectar · "
              "S: sentarse/levantarse (haz clic en la vista 3D primero)")
        yaw = self.yaw()
        pos = list(self.f_trans.getSFVec3f())
        while self.step(self.step_ms) != -1:
            dt = self.step_ms / 1000.0
            base, shift, pressed = self.demo_keys(dt) if self.demo else self.read_keys()
            fwd = (1.0 if Keyboard.UP in base else 0.0) - (1.0 if Keyboard.DOWN in base else 0.0)
            side = turn = 0.0
            lr = (1.0 if Keyboard.LEFT in base else 0.0) - (1.0 if Keyboard.RIGHT in base else 0.0)
            if shift:
                side = lr
            else:
                turn = lr

            busy = self.state in ("pick", "sit_down", "stand_up")
            # ── actions (on the key PRESS) ──
            if ord("S") in pressed and not busy:
                if self.state == "sitting":
                    self.play("stand_up")
                else:
                    self.play("sit_down")
            elif ord("R") in pressed and not busy and self.state != "sitting":
                self.play("pick")
            elif ord("C") in pressed and not busy and self.state != "sitting":
                self.play("harvest")
                self.cycles_left = self.opt.harvest_cycles

            # ── advance the current motion ──
            st = self.state
            clip = self.clip
            if st in ("pick", "sit_down", "stand_up"):
                self.frame += dt / clip.dt
                if self.frame >= clip.frames:
                    self.frame = float(clip.frames)
                    if st == "sit_down":
                        self.state = "sitting"            # hold the last frame: seated on the ground
                    else:
                        self.play("idle")
            elif st == "sitting":
                pass
            elif st == "harvest" and not (fwd or side or turn):
                before = self.frame
                self.frame += dt / clip.dt
                if clip.loop and before < clip.loop[1] <= self.frame:
                    self.cycles_left -= 1
                if self.cycles_left <= 0:
                    self.play("idle")
                else:
                    self.frame = clip.wrap(self.frame)
            else:
                # ── locomotion: the body moves at the clip's measured speed; the clip follows the body ──
                if fwd or side:
                    if st != "walk":
                        self.play("walk")
                    v = self.walk_speed * (fwd if fwd else abs(side))
                    rate = v / self.clips["walk"].meta["speed"]
                    self.frame = self.clip.wrap(self.frame + math.copysign(1.0, fwd or 1.0) * abs(rate) * dt / self.clip.dt)
                    if turn:
                        yaw += turn * self.turn_speed * dt
                    dx = (fwd * math.cos(yaw) - side * math.sin(yaw)) * self.walk_speed * dt
                    dy = (fwd * math.sin(yaw) + side * math.cos(yaw)) * self.walk_speed * dt
                    pos[0] += dx
                    pos[1] += dy
                elif turn:
                    name = "turn_left" if turn > 0 else "turn_right"
                    if st != name:
                        self.play(name)
                    clip_rate = abs(self.clip.meta["yaw_rate"])
                    self.frame = self.clip.wrap(self.frame + (self.turn_speed / clip_rate) * dt / self.clip.dt)
                    yaw += turn * self.turn_speed * dt
                else:
                    if st != "idle":
                        self.play("idle")
                    self.frame = self.clip.wrap(self.frame + dt / self.clip.dt)

            yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
            pos[2] = self.ground.z(pos[0], pos[1])
            self.f_trans.setSFVec3f(pos)
            self.f_rot.setSFRotation([0.0, 0.0, 1.0, yaw])
            self.show(dt)
            if self.state != self.last_state:
                self.last_state = self.state
                self.state_t = 0.0
                self.log(f"{self.state} at ({pos[0]:.2f}, {pos[1]:.2f}, z {pos[2]:.3f}) yaw {math.degrees(yaw):.0f} deg")
            self.state_t += dt
            # Test screenshots (GRANJERO_SNAP_DIR): one per motion, taken mid-motion.
            when = {"idle": 1.0, "walk": 1.0, "turn_left": 1.0, "pick": 1.6, "harvest": 1.2, "sitting": 1.0}
            if self.snap_dir and self.state in when and self.state_t >= when[self.state] \
                    and self.state not in self.snapped:
                self.snapped.add(self.state)
                path = os.path.join(self.snap_dir, f"granjero_{self.state}.jpg")
                self.exportImage(path, 85)
                self.log(f"screenshot {path}")


if __name__ == "__main__":
    try:
        Granjero().run()
    except Exception:
        err = traceback.format_exc()
        print(err)
        if os.environ.get("GRANJERO_LOG"):
            with open(os.environ["GRANJERO_LOG"], "a") as f:
                f.write(err)
        raise
