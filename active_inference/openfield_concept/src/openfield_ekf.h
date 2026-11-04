// openfield_ekf.h — SE(2) EKF for the open field: wheels + gyro predict, GPS + IMU yaw correct.
//
// PURE Eigen: no DSR, no Qt, no Ice, so it can be unit-tested in isolation (self_test()).
//
// ── FRAMES (the ones every agent here uses; see active_inference/FRAMES.md) ─────────────────────
//   * World frame ("world" node; was "field" until 2026-11-03): x = EAST, y = NORTH, metres, origin = WorldInfo.gpsReference. It is
//     the GPS local frame the bridge serves (GPS.getPos), and in simulation also the Webots world frame.
//   * Robot body frame: +Y FORWARD, +X RIGHT, +Z up.
//   * State [x, y, theta]: theta is the rotation field<-robot about +Z, i.e. the angle of the robot's
//     +X axis from east, counter-clockwise. With that, forward = (-sin theta, cos theta). It is the
//     angle the InertialUnit reports as yaw (ENU, north = +Y), and the one room_concept's RT edge
//     carries as theta_room_to_robot.
//
// ── MODEL ───────────────────────────────────────────────────────────────────────────────────────
//   predict:  wheel odometry gives the body velocity (adv, side); the GYRO gives the yaw rate. The
//             wheel yaw rate is NOT used: a skid-steer scrubs, and on the Husky the wheels over-report
//             rotation by 40-130 % depending on the manoeuvre (measured, plan.md F1). The gyro does not.
//   correct:  GPS antenna position (with the mast lever arm, so a moving robot's GPS also informs
//             heading), and the IMU's absolute yaw.
//   Process noise is a DENSITY that grows with the physical covariate (speed, yaw rate), never a
//   constant per step and never a gate (active_inference/CLAUDE.md: no thresholds).
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>

namespace rc::openfield
{

struct EkfParams
{
    // Wheel odometry, forward channel: sigma(v) = hypot(adv_sigma0, adv_slip * |v|), a density in
    // m/sqrt(s). The floor is the parked-encoder residual; the slip term is proportional to speed.
    double adv_sigma0 = 0.01;
    double adv_slip   = 0.05;
    // Lateral channel. A skid-steer has no lateral DOF, but it DOES slide sideways while turning:
    // sigma = hypot(side_sigma0, side_slip * |v * w|) (centripetal load drives the slide).
    double side_sigma0 = 0.005;
    double side_slip   = 0.10;
    // Gyro yaw-rate density (rad/sqrt(s)) plus a rate-proportional scale-error term.
    double gyro_sigma0 = 0.002;
    double gyro_scale  = 0.01;
    // Measurements (per-sample standard deviations).
    double gps_sigma_m   = 0.05;    // RTK-class fix; the correlated sim noise makes 0.02 over-confident
    double yaw_sigma_rad = 0.005;
    // GPS antenna in the body frame (x right, y forward), metres — husky.json body<-gps.
    Eigen::Vector2d gps_lever{0.0, -0.25};
};

class OpenFieldEkf
{
public:
    explicit OpenFieldEkf(EkfParams p = {}) : p_(std::move(p)) {}

    void set_params(const EkfParams &p) { p_ = p; }
    const EkfParams &params() const { return p_; }

    // Start from a GPS fix of the ANTENNA and an absolute yaw: the body origin is the antenna minus
    // the rotated lever arm. Covariance = the two measurement covariances.
    void initialize(const Eigen::Vector2d &gps_antenna_xy, double yaw);
    [[nodiscard]] bool initialized() const { return initialized_; }
    void reset() { initialized_ = false; }

    // dt in seconds (on the clock the RATES are measured in — the sim clock in simulation).
    void predict(double dt, double adv, double side, double yaw_rate);
    // Returns the squared Mahalanobis distance of the innovation (diagnostic only — never a gate).
    double correct_gps(const Eigen::Vector2d &gps_antenna_xy);
    double correct_gps(const Eigen::Vector2d &gps_antenna_xy, double sigma_m);
    double correct_yaw(double yaw);
    // Student-t GPS likelihood (nu degrees of freedom), as ONE iteratively-reweighted step of its Gaussian scale
    // mixture: the measurement precision is scaled by E[lambda | r] = (nu + 2) / (nu + d2), d2 the squared
    // Mahalanobis innovation under sigma. A consistent fix (d2 ~ 2) is fused almost as a Gaussian one; a
    // multipath jump (d2 >> nu) is down-weighted continuously, never gated. Returns d2; *weight gets the scale.
    double correct_gps_student(const Eigen::Vector2d &gps_antenna_xy, double sigma_m, double nu, double *weight = nullptr);

    [[nodiscard]] const Eigen::Vector3d &state() const { return x_; }
    [[nodiscard]] const Eigen::Matrix3d &covariance() const { return P_; }
    // Antenna position predicted by the current state (what the GPS should read).
    [[nodiscard]] Eigen::Vector2d predicted_antenna() const;

    // Synthetic drive with known truth; returns true on PASS and fills `report`.
    static bool self_test(std::string *report = nullptr);

private:
    EkfParams p_;
    bool initialized_ = false;
    Eigen::Vector3d x_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d P_ = Eigen::Matrix3d::Identity();
};

double wrap_pi(double a);

} // namespace rc::openfield
