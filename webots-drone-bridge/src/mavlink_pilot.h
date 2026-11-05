// MavlinkPilot — the joystick, spoken to ArduCopter the way a real pilot app speaks to a real drone: MAVLink
// commands to its autopilot, never motor speeds. Used with Flight.Backend = "ardupilot".
//
// Gamepad (python_xbox_controller, "mode 2"), only while the drone is selected (A; B gives the sticks back):
//   left stick up, drone on the ground  -> GUIDED + arm + take off to Pilot.TakeoffAltitude
//   sticks in flight                    -> velocity setpoints in the heading frame (SET_POSITION_TARGET_LOCAL_NED,
//                                          MAV_FRAME_BODY_OFFSET_NED): forward, side, climb, yaw rate
//   sticks released                     -> zero velocity: the autopilot holds its position (GPS/EKF)
//   left stick held down near the ground (relative altitude < Pilot.LandAltitude) -> LAND mode: the autopilot
//                                          lands and disarms by itself (its land detector, not ours)
//   left stick up during LAND           -> back to GUIDED (abort the landing)
//   B (deselect) in flight              -> one zero-velocity setpoint: it hovers where it is
// Talks to SITL's SERIAL1 (tcp:5762) so SERIAL0 (tcp:5760) stays free for a ground station or mission scripts.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "flight_controller.h"   // drone::FlightCommand (vx forward, vy left, vz up, yaw_rate CCW)

namespace drone
{

struct PilotInput
{
    FlightCommand cmd;           // already zeroed by the caller if the joystick went quiet
    bool selected = false;       // the joystick's control target is this drone
};

struct PilotConfig
{
    std::string host = "127.0.0.1";
    int    port = 5762;
    double takeoff_alt_m = 1.0;  // GUIDED take-off height above the take-off point
    double land_alt_m    = 0.5;  // below this relative altitude a held "down" stick hands over to LAND
    double stick_arm     = 0.3;  // m/s of climb command that means "take off" (and "abort landing")
    double stick_land    = 0.3;  // m/s of descent command that means "land" near the ground
};

struct PilotStatus
{
    bool connected = false, armed = false;
    std::uint32_t mode = 0;      // ArduCopter custom mode (4 GUIDED, 9 LAND, ...)
    int landed = 0;              // MAV_LANDED_STATE: 1 on ground, 2 in air, 3 taking off, 4 landing
    double rel_alt_m = 0.0;
};

class MavlinkPilot
{
public:
    using InputFn = std::function<PilotInput()>;
    ~MavlinkPilot() { stop(); }
    void start(const PilotConfig& cfg, InputFn input);
    void stop();
    [[nodiscard]] PilotStatus status() const;

private:
    void run();
    bool connect();
    void send(const void* msg);                       // const mavlink_message_t*
    void handle(const void* msg);
    void tick(const PilotInput& in, std::chrono::steady_clock::time_point now);
    void command_long(std::uint16_t command, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0,
                      float p5 = 0, float p6 = 0, float p7 = 0);
    void set_mode(std::uint32_t custom_mode);
    void velocity(double fwd, double right, double down, double yaw_rate_cw);

    PilotConfig cfg_;
    InputFn input_;
    std::thread thread_;
    std::atomic_bool stop_{false};
    int fd_ = -1;
    std::uint8_t target_sys_ = 1, target_comp_ = 1;
    mutable std::mutex mtx_;
    PilotStatus st_;
    bool takeoff_sent_ = false, was_selected_ = false;
    std::chrono::steady_clock::time_point last_arm_try_{}, last_mode_try_{};
    std::chrono::steady_clock::time_point last_state_msg_{}, last_stream_req_{};   // EXTENDED_SYS_STATE stream
};

}  // namespace drone
