#include "ardupilot_link.h"

#include <cmath>
#include <cstring>
#include <format>

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace drone
{
namespace
{
constexpr std::uint16_t kMagic16 = 18458;
constexpr std::uint16_t kMagic32 = 29569;

// Rotation matrix (row-major) -> unit quaternion (w, x, y, z), Shepperd's method.
std::array<double, 4> quat_from_matrix(const std::array<double, 9>& m)
{
    const double tr = m[0] + m[4] + m[8];
    double w, x, y, z;
    if (tr > 0.0)
    {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s; x = (m[7] - m[5]) / s; y = (m[2] - m[6]) / s; z = (m[3] - m[1]) / s;
    }
    else if (m[0] > m[4] and m[0] > m[8])
    {
        const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0;
        w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s;
    }
    else if (m[4] > m[8])
    {
        const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0;
        w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0;
        w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s;
    }
    return {w, x, y, z};
}
}  // namespace

ArduPilotLink::~ArduPilotLink()
{
    if (fd_ >= 0) ::close(fd_);
}

bool ArduPilotLink::open(int port)
{
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(static_cast<std::uint16_t>(port));
    return ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
}

std::optional<ServoFrame> ArduPilotLink::receive(int timeout_ms)
{
    std::optional<ServoFrame> out;
    pollfd p{fd_, POLLIN, 0};
    int wait = timeout_ms;
    while (::poll(&p, 1, wait) > 0)
    {
        std::uint8_t buf[512];
        sockaddr_in from{};
        socklen_t len = sizeof(from);
        const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &len);
        wait = 0;                                      // got one: only drain what is already queued
        if (n < 8) continue;
        std::uint16_t magic;
        std::memcpy(&magic, buf, 2);
        const int channels = magic == kMagic16 ? 16 : magic == kMagic32 ? 32 : 0;
        if (channels == 0 or n < 8 + 2 * channels) continue;
        ServoFrame f;
        std::memcpy(&f.frame_rate, buf + 2, 2);
        std::memcpy(&f.frame_count, buf + 4, 4);
        std::memcpy(f.pwm.data(), buf + 8, 2 * 16);    // first 16 channels are all a quad needs
        peer_ = from;
        has_peer_ = true;
        out = f;
    }
    return out;
}

void ArduPilotLink::send(const PhysicsState& s)
{
    if (not has_peer_) return;
    // World ENU (x east, y north, z up: the orchard worlds set northDirection 0 1 0 so Webots' GPS gives real
    // WGS84) -> NED is P: (x, y, z) -> (y, x, -z). Body FLU -> FRD is S = diag(1, -1, -1). R_ned_frd = P·R·S.
    auto P = [](const std::array<double, 3>& v) { return std::array<double, 3>{v[1], v[0], -v[2]}; };
    auto S = [](const std::array<double, 3>& v) { return std::array<double, 3>{v[0], -v[1], -v[2]}; };
    const auto& r = s.R_enu_flu;
    // P·R swaps rows 0 and 1 and negates row 2; ·S then negates columns 1 and 2.
    const std::array<double, 9> R_ned_frd = { r[3], -r[4], -r[5],
                                              r[0], -r[1], -r[2],
                                             -r[6],  r[7],  r[8]};
    const auto q = quat_from_matrix(R_ned_frd);
    const auto g = S(s.gyro_flu), a = S(s.accel_flu), p = P(s.pos_enu), v = P(s.vel_enu);
    line_ = std::format(
        "{{\"timestamp\":{:.6f},\"imu\":{{\"gyro\":[{:.6f},{:.6f},{:.6f}],\"accel_body\":[{:.6f},{:.6f},{:.6f}]}},"
        "\"position\":[{:.6f},{:.6f},{:.6f}],\"velocity\":[{:.6f},{:.6f},{:.6f}],"
        "\"quaternion\":[{:.8f},{:.8f},{:.8f},{:.8f}]",
        s.time_s, g[0], g[1], g[2], a[0], a[1], a[2], p[0], p[1], p[2], v[0], v[1], v[2], q[0], q[1], q[2], q[3]);
    if (s.range_down_m) line_ += std::format(",\"rng_1\":{:.4f}", *s.range_down_m);
    line_ += "}\n";
    ::sendto(fd_, line_.data(), line_.size(), 0, reinterpret_cast<const sockaddr*>(&peer_), sizeof(peer_));
}

}  // namespace drone
