// openfield_variants.h — HOW a GPS fix is fused, and the filter variants E1 compares side by side.
//
// PURE Eigen (no DSR, no Qt, no Ice), like openfield_ekf.h.
//
// ── THE RECEIVER STATUS (optional) ──────────────────────────────────────────────────────────────────────────
//   A real RTK receiver reports, beside each fix, its own sigma, the fix type (RTK fixed / float / single) and
//   when the fix was measured. In simulation the bridge writes that report to a file ([GnssDegradation]
//   StatusFile); [Gps] StatusFile here reads it. Without it every fix is "sigma = Ekf.GpsSigmaM, now".
//
// ── GpsModel: the switches ([Ekf] in etc/config.toml) ───────────────────────────────────────────────────────
//   receiver_sigma  R = the receiver's reported sigma^2 instead of the fixed Ekf.GpsSigmaM^2
//   student         Student-t likelihood (nu) instead of a Gaussian: multipath jumps are down-weighted
//   lag_comp        the fix is moved to NOW by the filter's own displacement since it was MEASURED
//                   (the latency the receiver stamps), instead of being applied as if it were current
//   All false = the filter as it always was.
//
// ── Shadow filters ([Experiment] ShadowFilters) ─────────────────────────────────────────────────────────────
//   Extra filters fed the SAME odometry, gyro, IMU yaw and fixes as the published one, never published, only
//   logged ([Diag] ExperimentCsv) — so E1 compares configurations on identical data:
//     A  "current"   the current filter: fixed sigma, Gaussian, fix applied on arrival
//     B  "receiver"  receiver sigma + Student-t + latency compensation
//     C  "no_gnss"   initialised like the others, then never fused a fix (dead reckoning + IMU yaw)
//     D  "no_yaw"    like A but without the IMU's absolute yaw
#pragma once

#include "openfield_ekf.h"

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace rc::openfield
{

struct GpsFix
{
    std::uint64_t seq = 0;
    std::int64_t meas_ms = 0, deliver_ms = 0;   // receiver clock (simulation ms in simulation); 0 = unknown
    bool valid = true;
    Eigen::Vector2d xy = Eigen::Vector2d::Zero();
    double sigma = 0.0;                         // reported per-axis 1 sigma; 0 = not reported
    int fix_type = -1, n_vis = -1;              // GGA quality (4 fixed, 5 float, 1 single, 0 none); -1 = unknown
    double mp = 0.0, err = 0.0;                 // simulation truth carried by the status file (logs only)
    bool has_status = false;
};

struct GpsModel
{
    bool receiver_sigma = false;
    bool student = false;
    double nu = 4.0;
    bool lag_comp = false;
};

struct GpsDiag { double d2 = 0.0, w = 1.0, sigma = 0.0, lag_ms = 0.0; };

// Predicted antenna position by filter clock: what lag compensation subtracts.
class AntennaHistory
{
public:
    void push(std::int64_t clock_ms, const Eigen::Vector2d &xy);
    [[nodiscard]] std::optional<Eigen::Vector2d> at(std::int64_t clock_ms) const;
    void clear() { h_.clear(); }
private:
    std::deque<std::pair<std::int64_t, Eigen::Vector2d>> h_;
};

// Fuse one fix into `ekf` the way `m` says. now_clock_ms = the filter's clock (last propagated IMU frame).
GpsDiag fuse_gps(OpenFieldEkf &ekf, const GpsModel &m, const GpsFix &fix, double fixed_sigma,
                 const AntennaHistory &hist, std::int64_t now_clock_ms);

struct Shadow
{
    std::string name;
    OpenFieldEkf ekf;
    GpsModel gps;
    bool use_gps = true, use_yaw = true;
    AntennaHistory antenna;
    std::int64_t last_yaw_clock_ms = 0;
    GpsDiag diag;
    // (stamp ms, state, covariance): graded at the ground truth's own stamp, like the published pose.
    std::deque<std::tuple<std::int64_t, Eigen::Vector3d, Eigen::Matrix3d>> history;
    void record(std::int64_t stamp_ms);
    [[nodiscard]] std::optional<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> at(std::int64_t stamp_ms) const;
};

std::vector<Shadow> make_shadows(const EkfParams &p, double nu);

} // namespace rc::openfield
