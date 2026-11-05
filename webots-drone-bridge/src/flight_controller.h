#pragma once
// FlightController — the drone's stabiliser: turns a pilot's VELOCITY command into four rotor speeds.
//
// The InspectionDrone (webots-agri/protos) has no autopilot of its own; without this, a joystick cannot fly it.
// Cascade, every Webots step:
//   stick -> velocity setpoint (heading frame) -> tilt setpoint -> attitude PD -> rotor mix
//   stick -> climb-rate setpoint -> altitude setpoint (integrated) -> altitude PID -> collective thrust
// Rotor mix and signs are those of Cyberbotics' Mavic 2 Pro example controller (the drone proto is derived
// from that model and keeps its propeller layout and thrust-constant signs).
//
// Frames (Webots drone body, FLU): +X forward, +Y left, +Z up. Inertial-unit roll is about +X (positive =
// right side DOWN), pitch about +Y (positive = nose DOWN), yaw about +Z (positive = counter-clockwise).
//
// Arming: the rotors stay OFF while the drone is parked and not selected. Selected (the joystick's A button)
// and parked, they SPOOL at an idle speed well below lift-off — the simulated rotors take seconds to spin up
// from rest (measured: >2.4 s from 0, 1.3 s from idle), so a drone that starts from idle answers the stick.
// It arms when the pilot pushes the climb stick up, and disarms after a sustained descent command produces no
// descent (it is resting on something). No automatic take-off and no automatic landing.

#include <algorithm>
#include <array>
#include <cmath>

namespace drone
{

struct FlightGains
{
    // Hover: the official Mavic (0.9 kg) hovers at 68.6 rad/s (measured). Scale s: mass x s^2, thrust x s^3,
    // so hover x sqrt(1/s) -> 97 rad/s at s = 0.5 (measured: 97).
    double hover_omega   = 97.0;   // rad/s, feed-forward rotor speed that holds the drone
    double max_omega     = 576.0;  // rad/s, the proto's RotationalMotor.maxVelocity
    double k_z_p         = 10.0;   // rad/s per m of altitude error
    double k_z_d         = 10.0;   // rad/s per m/s of climb-rate error
    double k_z_i         = 1.0;    // rad/s per m·s of integrated altitude error (absorbs a wrong hover_omega)
    double z_i_limit     = 15.0;   // rad/s, cap of the integral contribution
    double z_i_band      = 0.2;    // m, integrate only this close to the setpoint (anti wind-up while parked)
    double k_v           = 0.15;   // rad of tilt per m/s of horizontal velocity error
    double max_tilt      = 0.25;   // rad
    double tilt_tau      = 0.3;    // s, low-pass on the tilt setpoint
    // Attitude: Cyberbotics' mavic2pro.c gains, which fly this airframe (verified at s = 1 and s = 0.5).
    double k_roll_p      = 50.0;   // rad/s per rad
    double k_pitch_p     = 30.0;   // rad/s per rad
    double k_rate        = 1.0;    // rad/s per rad/s of body rate
    double k_yaw         = 2.0;    // rad/s per rad/s of yaw-rate error
    double max_z_lead    = 0.5;    // m, how far the altitude setpoint may run ahead of the drone
    double idle_omega    = 60.0;   // rad/s, spool speed while selected and parked (lift-off needs ~100)
    double collective_rate = 20.0; // rad/s per s, below the rotors' ~45 so attitude control keeps its margin
    double arm_vz        = 0.2;    // m/s, a climb command above this arms the rotors
    double disarm_vz     = -0.2;   // m/s, a descent command below this ...
    double disarm_still  = 0.05;   // m/s, ... that yields less descent than this ...
    double disarm_after  = 1.5;    // s,   ... for this long, means: landed -> rotors off
};

struct FlightCommand            // pilot setpoints, heading frame (yaw only, no tilt)
{
    double vx = 0.0;            // m/s forward
    double vy = 0.0;            // m/s left
    double vz = 0.0;            // m/s up
    double yaw_rate = 0.0;      // rad/s counter-clockwise
};

struct FlightState              // measured
{
    double roll = 0.0, pitch = 0.0, yaw = 0.0;      // rad (inertial unit)
    double p = 0.0, q = 0.0, r = 0.0;               // rad/s body rates (gyro)
    double z = 0.0;                                 // m, altitude
    double vxw = 0.0, vyw = 0.0, vzw = 0.0;         // m/s, world-frame velocity
};

// Rotor order: front-left, front-right, rear-left, rear-right. Returned with the motor's own sign applied
// (front-right and rear-left spin the other way, as in the Mavic example).
using RotorSpeeds = std::array<double, 4>;

class FlightController
{
public:
    explicit FlightController(const FlightGains& g = {}) : g_(g) {}

    [[nodiscard]] bool armed() const noexcept { return armed_; }
    [[nodiscard]] double altitude_setpoint() const noexcept { return z_ref_; }

    // `spool`: the pilot has selected this drone — idle the rotors while parked.
    RotorSpeeds step(const FlightCommand& c, const FlightState& s, double dt, bool spool = false)
    {
        const RotorSpeeds idle = {g_.idle_omega, -g_.idle_omega, -g_.idle_omega, g_.idle_omega};
        if (not armed_)
        {
            if (c.vz > g_.arm_vz)
            {
                armed_ = true;
                z_ref_ = s.z;
                z_int_ = 0.0;
                still_t_ = 0.0;
                pitch_ref_ = roll_ref_ = 0.0;
                collective_ = spool ? g_.idle_omega : 0.0;   // continue from what the rotors are doing
            }
            else
                return spool ? idle : RotorSpeeds{0.0, 0.0, 0.0, 0.0};
        }

        // Landed? A descent command that produces no descent, held long enough.
        if (c.vz < g_.disarm_vz and std::abs(s.vzw) < g_.disarm_still)
            still_t_ += dt;
        else
            still_t_ = 0.0;
        if (still_t_ > g_.disarm_after)
        {
            armed_ = false;
            return spool ? idle : RotorSpeeds{0.0, 0.0, 0.0, 0.0};
        }

        // Altitude: the setpoint integrates the climb command but may not run away from the drone.
        z_ref_ = std::clamp(z_ref_ + c.vz * dt, s.z - g_.max_z_lead, s.z + g_.max_z_lead);
        const double ez = z_ref_ - s.z;
        // Integrate only near the setpoint: while the drone is still resting on the deck the error is large
        // and an integral would wind up into an overshoot on lift-off.
        if (std::abs(ez) < g_.z_i_band)
            z_int_ = std::clamp(z_int_ + ez * dt, -g_.z_i_limit / g_.k_z_i, g_.z_i_limit / g_.k_z_i);
        // The simulated rotors are acceleration-limited (measured ~45 rad/s per s). While all four are accelerating
        // at that limit there is no margin left for a DIFFERENTIAL, i.e. no attitude control — which is how a
        // take-off on a fast-rising collective tipped the drone over. So the collective may only change at a rate
        // the rotors can follow with room to spare; the differential (attitude) is never rate-limited.
        const double target = g_.hover_omega + g_.k_z_p * ez + g_.k_z_d * (c.vz - s.vzw) + g_.k_z_i * z_int_;
        const double step_max = g_.collective_rate * dt;
        collective_ += std::clamp(target - collective_, -step_max, step_max);
        const double collective = collective_;

        // Horizontal velocity in the heading frame.
        const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
        const double vx = cy * s.vxw + sy * s.vyw;
        const double vy = -sy * s.vxw + cy * s.vyw;
        // Forward needs nose DOWN (pitch > 0); leftward needs right side UP (roll < 0). The tilt setpoint is
        // low-pass filtered: a stick step must not become an attitude step (that excited the pitch loop).
        const double a = std::clamp(dt / g_.tilt_tau, 0.0, 1.0);
        pitch_ref_ += a * (std::clamp(g_.k_v * (c.vx - vx), -g_.max_tilt, g_.max_tilt) - pitch_ref_);
        roll_ref_  += a * (std::clamp(-g_.k_v * (c.vy - vy), -g_.max_tilt, g_.max_tilt) - roll_ref_);
        const double pitch_ref = pitch_ref_, roll_ref = roll_ref_;

        const double roll_in  = g_.k_roll_p * std::clamp(s.roll - roll_ref, -1.0, 1.0) + g_.k_rate * s.p;
        const double pitch_in = g_.k_pitch_p * std::clamp(s.pitch - pitch_ref, -1.0, 1.0) + g_.k_rate * s.q;
        const double yaw_in   = g_.k_yaw * (c.yaw_rate - s.r);

        const auto lim = [this](double w) { return std::clamp(w, 0.0, g_.max_omega); };
        const double fl = lim(collective - roll_in + pitch_in - yaw_in);
        const double fr = lim(collective + roll_in + pitch_in + yaw_in);
        const double rl = lim(collective - roll_in - pitch_in + yaw_in);
        const double rr = lim(collective + roll_in - pitch_in - yaw_in);
        return {fl, -fr, -rl, rr};
    }

    void disarm() noexcept { armed_ = false; }

private:
    FlightGains g_;
    bool armed_ = false;
    double z_ref_ = 0.0;
    double z_int_ = 0.0;
    double pitch_ref_ = 0.0;
    double roll_ref_ = 0.0;
    double collective_ = 0.0;
    double still_t_ = 0.0;
};

}  // namespace drone
