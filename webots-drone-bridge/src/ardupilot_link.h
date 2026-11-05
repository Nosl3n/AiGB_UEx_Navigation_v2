// ArduPilot SITL link — the physics side of ArduPilot's JSON simulator interface
// (ardupilot/libraries/SITL/SIM_JSON.cpp, examples/JSON/readme.md).
//
// Lockstep, one exchange per Webots step:
//   SITL  --(binary servo packet: magic, frame_rate, frame_count, pwm[16|32])-->  UDP :9002 (this side)
//   SITL  <--(one JSON line: timestamp, imu.gyro, imu.accel_body, position, velocity, quaternion)--  reply
// SITL does not advance until it gets the reply, so the autopilot's clock IS the simulation clock.
//
// Frames. Webots world: ENU — x east, y north, z up (the orchard worlds set WorldInfo.northDirection 0 1 0, so
// the drone's autopilot and the Husky's GPS agree on geography). Drone body (InspectionDrone.proto): x forward,
// y left, z up (FLU). ArduPilot wants earth NED and body FRD: ENU->NED = P (x,y,z)->(y,x,-z), FLU->FRD = S =
// diag(1,-1,-1), so R_ned_frd = P · R_enu_flu · S.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <netinet/in.h>

namespace drone
{

struct ServoFrame
{
    std::uint16_t frame_rate = 0;
    std::uint32_t frame_count = 0;
    std::array<std::uint16_t, 16> pwm{};
};

struct PhysicsState                                // everything in WEBOTS frames (world ENU, body FLU)
{
    double time_s = 0.0;                           // simulation time
    std::array<double, 3> gyro_flu{};              // rad/s, body
    std::array<double, 3> accel_flu{};             // m/s^2, body, specific force (+9.81 z at rest)
    std::array<double, 3> pos_enu{};               // m, world, relative to the start point
    std::array<double, 3> vel_enu{};               // m/s, world
    std::array<double, 9> R_enu_flu{};             // row-major body -> world rotation (Webots getOrientation())
    std::optional<double> range_down_m;            // downward rangefinder, if any
};

class ArduPilotLink
{
public:
    ~ArduPilotLink();
    bool open(int port);                           // bind 0.0.0.0:port (UDP)
    // Latest servo frame, waiting up to timeout_ms for one (0 = poll). Drains older queued frames.
    std::optional<ServoFrame> receive(int timeout_ms);
    // Reply to whoever sent the last servo frame. No-op before the first frame.
    void send(const PhysicsState& s);
    [[nodiscard]] bool has_peer() const { return has_peer_; }

private:
    int fd_ = -1;
    sockaddr_in peer_{};
    bool has_peer_ = false;
    std::string line_;
};

}  // namespace drone
