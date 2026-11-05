#!/usr/bin/env python3
"""Basic ArduCopter SITL flight test for the inspection drone: GUIDED take-off, hover, land.

Talks MAVLink exactly as a ground station or a mission node would; nothing here touches Webots.
usage: test_flight.py [--alt 3] [--hold 15] [--url tcp:127.0.0.1:5760]
Run with the python of ~/robocomp/ardupilot-venv (pymavlink).
"""
import argparse
import math
import statistics
import sys
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--alt", type=float, default=3.0)
ap.add_argument("--hold", type=float, default=15.0)
args = ap.parse_args()


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


m = mavutil.mavlink_connection(args.url, source_system=250)
m.wait_heartbeat(timeout=60)
log(f"heartbeat from system {m.target_system}")
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)


def statustexts(timeout=0.0):
    t_end = time.time() + timeout
    while True:
        msg = m.recv_match(type="STATUSTEXT", blocking=timeout > 0, timeout=max(0.0, t_end - time.time()))
        if msg is None:
            return
        log(f"  autopilot: {msg.text}")


def wait_ready(timeout=150):
    """Arming checks passing (SYS_STATUS prearm bit) AND the EKF holding an absolute horizontal position
    (EKF_STATUS_REPORT), which GUIDED needs: the prearm bit alone can come before the EKF uses the GPS."""
    t0 = time.time()
    prearm = pos_abs = False
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["SYS_STATUS", "STATUSTEXT", "EKF_STATUS_REPORT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            log(f"  autopilot: {msg.text}")
        elif msg.get_type() == "SYS_STATUS":
            prearm = bool(msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
        else:
            pos_abs = bool(msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS)
        if prearm and pos_abs:
            return time.time() - t0
    return None


def arm(timeout=20):
    t0 = time.time()
    m.arducopter_arm()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            log(f"  autopilot: {msg.text}")
        elif msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                log(f"  arm rejected (result {msg.result}), retrying")
                time.sleep(1)
                m.arducopter_arm()
        elif msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == m.target_system \
                and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
    return False


ready = wait_ready()
if ready is None:
    log("FAIL: pre-arm checks never passed")
    sys.exit(1)
log(f"pre-arm checks OK after {ready:.1f} s")

m.set_mode("GUIDED")
if not arm():
    log("FAIL: could not arm")
    sys.exit(1)
log("armed")
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                        0, 0, 0, 0, 0, 0, 0, args.alt)

# climb
t0 = time.time()
alt = 0.0
while time.time() - t0 < 30:
    msg = m.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT"], blocking=True, timeout=1)
    if msg is None:
        continue
    if msg.get_type() == "STATUSTEXT":
        log(f"  autopilot: {msg.text}")
        continue
    alt = msg.relative_alt / 1000.0
    if alt > 0.95 * args.alt:
        break
log(f"climb: reached {alt:.2f} m in {time.time() - t0:.1f} s")
if alt < 0.9 * args.alt:
    log("FAIL: did not reach the take-off altitude")
    sys.exit(1)

# hover: altitude error, horizontal drift and attitude while holding
xs, ys, zs, rolls, pitches = [], [], [], [], []
t0 = time.time()
while time.time() - t0 < args.hold:
    msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=1)
    if msg is None:
        continue
    if msg.get_type() == "LOCAL_POSITION_NED":
        xs.append(msg.x); ys.append(msg.y); zs.append(-msg.z)
    elif msg.get_type() == "ATTITUDE":
        rolls.append(math.degrees(msg.roll)); pitches.append(math.degrees(msg.pitch))
    else:
        log(f"  autopilot: {msg.text}")
if zs:
    drift = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    log(f"hover {args.hold:.0f} s: alt mean {statistics.mean(zs):.2f} m (min {min(zs):.2f}, max {max(zs):.2f}), "
        f"horizontal spread {drift:.2f} m, |roll| max {max(map(abs, rolls)):.1f} deg, "
        f"|pitch| max {max(map(abs, pitches)):.1f} deg")

# yaw 90 deg clockwise in place (checks the sign of the rotors' reaction torque: a wrong sign diverges)
def heading():
    msg = m.recv_match(type="VFR_HUD", blocking=True, timeout=2)
    return msg.heading if msg else None
h0 = heading()
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                        0, 90, 30, 1, 1, 0, 0, 0)          # 90 deg, 30 deg/s, clockwise, relative
t0 = time.time()
h = h0
while time.time() - t0 < 8:
    h = heading()
turned = ((h - h0 + 540) % 360) - 180 if h is not None and h0 is not None else None
log(f"yaw: heading {h0} -> {h} deg (turned {turned:+.0f} deg, asked +90)")
if turned is None or abs(turned - 90) > 10:
    log("FAIL: yaw did not reach +90 deg")
    sys.exit(1)

# land
m.set_mode("LAND")
t0 = time.time()
while time.time() - t0 < 60:
    hb = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
    if hb is None:
        continue
    if hb.get_type() == "STATUSTEXT":
        log(f"  autopilot: {hb.text}")
    elif hb.get_srcSystem() == m.target_system and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        log(f"landed and disarmed {time.time() - t0:.1f} s after LAND")
        break
else:
    log("FAIL: did not disarm within 60 s of LAND")
    sys.exit(1)
log("PASS")
