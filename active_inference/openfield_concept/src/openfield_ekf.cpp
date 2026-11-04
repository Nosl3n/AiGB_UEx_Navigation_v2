#include "openfield_ekf.h"

#include <cmath>
#include <format>
#include <random>

namespace rc::openfield
{

double wrap_pi(double a)
{
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

namespace
{
// R(theta) * lever, and its derivative with respect to theta.
Eigen::Vector2d rotate(double th, const Eigen::Vector2d &v)
{
    const double c = std::cos(th), s = std::sin(th);
    return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
}
Eigen::Vector2d d_rotate(double th, const Eigen::Vector2d &v)
{
    const double c = std::cos(th), s = std::sin(th);
    return {-s * v.x() - c * v.y(), c * v.x() - s * v.y()};
}
} // namespace

void OpenFieldEkf::initialize(const Eigen::Vector2d &gps_antenna_xy, double yaw)
{
    const Eigen::Vector2d body = gps_antenna_xy - rotate(yaw, p_.gps_lever);
    x_ = {body.x(), body.y(), wrap_pi(yaw)};
    // The lever arm couples the yaw uncertainty into position; include it so the first published
    // covariance is honest rather than optimistic.
    const Eigen::Vector2d j = -d_rotate(yaw, p_.gps_lever);
    const double sg2 = p_.gps_sigma_m * p_.gps_sigma_m, sy2 = p_.yaw_sigma_rad * p_.yaw_sigma_rad;
    P_.setZero();
    P_.topLeftCorner<2, 2>() = sg2 * Eigen::Matrix2d::Identity() + sy2 * j * j.transpose();
    P_.block<2, 1>(0, 2) = sy2 * j;
    P_.block<1, 2>(2, 0) = sy2 * j.transpose();
    P_(2, 2) = sy2;
    initialized_ = true;
}

void OpenFieldEkf::predict(double dt, double adv, double side, double yaw_rate)
{
    if (not initialized_ or not (dt > 0.0) or not std::isfinite(dt)) return;

    // Midpoint heading: exact for a constant-rate arc to second order, and cheap.
    const double th = x_(2) + 0.5 * yaw_rate * dt;
    const double c = std::cos(th), s = std::sin(th);
    // Body (x right = side, y forward = adv) -> field.
    const double dx = (c * side - s * adv) * dt;
    const double dy = (s * side + c * adv) * dt;
    x_(0) += dx;
    x_(1) += dy;
    x_(2) = wrap_pi(x_(2) + yaw_rate * dt);

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -dy;          // d(dx)/d(theta)
    F(1, 2) =  dx;          // d(dy)/d(theta)

    // Noise enters in the body frame: [adv, side, yaw_rate], as densities integrated over dt.
    const double sv = std::hypot(p_.adv_sigma0, p_.adv_slip * std::abs(adv));
    const double ss = std::hypot(p_.side_sigma0, p_.side_slip * std::abs(adv * yaw_rate));
    const double sw = std::hypot(p_.gyro_sigma0, p_.gyro_scale * std::abs(yaw_rate));
    Eigen::Matrix<double, 3, 3> G;
    G << -s * dt, c * dt, 0.0,
          c * dt, s * dt, 0.0,
          0.0,    0.0,    dt;
    // Density sigma^2 over dt gives a rate variance sigma^2/dt; times dt^2 in G => sigma^2 * dt.
    const Eigen::Vector3d q{sv * sv / dt, ss * ss / dt, sw * sw / dt};
    P_ = F * P_ * F.transpose() + G * q.asDiagonal() * G.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

Eigen::Vector2d OpenFieldEkf::predicted_antenna() const
{
    return x_.head<2>() + rotate(x_(2), p_.gps_lever);
}

double OpenFieldEkf::correct_gps(const Eigen::Vector2d &z)
{
    return correct_gps(z, p_.gps_sigma_m);
}

double OpenFieldEkf::correct_gps(const Eigen::Vector2d &z, double sigma_m)
{
    if (not initialized_ or not z.allFinite()) return 0.0;
    Eigen::Matrix<double, 2, 3> H = Eigen::Matrix<double, 2, 3>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H.col(2) = d_rotate(x_(2), p_.gps_lever);
    const Eigen::Matrix2d R = sigma_m * sigma_m * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d r = z - predicted_antenna();
    const Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
    x_ += K * r;
    x_(2) = wrap_pi(x_(2));
    // Joseph form: stays symmetric positive-definite under round-off.
    const Eigen::Matrix3d IKH = Eigen::Matrix3d::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    return r.dot(S.ldlt().solve(r));
}

double OpenFieldEkf::correct_gps_student(const Eigen::Vector2d &z, double sigma_m, double nu, double *weight)
{
    if (not initialized_ or not z.allFinite() or not (sigma_m > 0.0)) return 0.0;
    Eigen::Matrix<double, 2, 3> H = Eigen::Matrix<double, 2, 3>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H.col(2) = d_rotate(x_(2), p_.gps_lever);
    const Eigen::Matrix2d S = H * P_ * H.transpose() + sigma_m * sigma_m * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d r = z - predicted_antenna();
    const double d2 = r.dot(S.ldlt().solve(r));
    const double w = (nu + 2.0) / (nu + d2);
    if (weight) *weight = w;
    correct_gps(z, sigma_m / std::sqrt(w));
    return d2;
}

double OpenFieldEkf::correct_yaw(double yaw)
{
    if (not initialized_ or not std::isfinite(yaw)) return 0.0;
    const double r = wrap_pi(yaw - x_(2));
    const double Rv = p_.yaw_sigma_rad * p_.yaw_sigma_rad;
    const double S = P_(2, 2) + Rv;
    const Eigen::Vector3d K = P_.col(2) / S;
    x_ += K * r;
    x_(2) = wrap_pi(x_(2));
    Eigen::RowVector3d H = Eigen::RowVector3d::Zero();
    H(2) = 1.0;
    const Eigen::Matrix3d IKH = Eigen::Matrix3d::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + Rv * K * K.transpose();
    return r * r / S;
}

// ── self_test ─────────────────────────────────────────────────────────────────────────────────────
// Drives a synthetic skid-steer along straight runs and pivots with noisy wheels (scrub included on
// the wheel yaw, which the filter must ignore), a noisy gyro, a 10 Hz GPS and a 100 Hz IMU yaw.
// PASS = final position error < 5 sigma_gps and heading error < 1 deg, and the covariance is sane.
bool OpenFieldEkf::self_test(std::string *report)
{
    EkfParams p;
    OpenFieldEkf ekf(p);
    std::mt19937 rng(7);
    std::normal_distribution<double> n01(0.0, 1.0);

    Eigen::Vector3d truth{2.0, -3.0, 0.4};
    const auto antenna = [&](const Eigen::Vector3d &t) { return Eigen::Vector2d(t.head<2>() + rotate(t(2), p.gps_lever)); };
    ekf.initialize(antenna(truth) + 0.02 * Eigen::Vector2d(n01(rng), n01(rng)), truth(2) + 0.002 * n01(rng));

    const double dt = 0.008;
    double max_pos_err = 0.0, max_yaw_err = 0.0;
    for (int k = 0; k < 20000; ++k)                         // 160 s
    {
        const double t = k * dt;
        const int phase = static_cast<int>(t / 10.0) % 4;   // straight, pivot, arc, stop
        const double v = phase == 0 ? 1.0 : phase == 2 ? 0.6 : 0.0;
        const double w = phase == 1 ? 0.5 : phase == 2 ? -0.3 : 0.0;
        // truth
        const double th = truth(2) + 0.5 * w * dt;
        truth(0) += -std::sin(th) * v * dt;
        truth(1) +=  std::cos(th) * v * dt;
        truth(2) = wrap_pi(truth(2) + w * dt);
        // sensors
        const double adv  = v * 1.01 + 0.01 * n01(rng) / std::sqrt(dt);
        const double gyro = w + 0.002 * n01(rng) / std::sqrt(dt);
        ekf.predict(dt, adv, 0.0, gyro);
        if (k % 12 == 0) ekf.correct_gps(antenna(truth) + 0.02 * Eigen::Vector2d(n01(rng), n01(rng)));
        if (k % 12 == 6) ekf.correct_yaw(truth(2) + 0.002 * n01(rng));
        if (t > 5.0)
        {
            max_pos_err = std::max(max_pos_err, (ekf.state().head<2>() - truth.head<2>()).norm());
            max_yaw_err = std::max(max_yaw_err, std::abs(wrap_pi(ekf.state()(2) - truth(2))));
        }
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(ekf.covariance());
    const bool pd = es.eigenvalues().minCoeff() > 0.0;
    const bool ok = pd and max_pos_err < 5.0 * p.gps_sigma_m and max_yaw_err < M_PI / 180.0;
    if (report)
        *report = std::format("OpenFieldEkf::self_test {}: max |pos err| {:.3f} m, max |yaw err| {:.3f} deg, "
                              "P positive-definite {}, final sigma xy {:.3f} m, sigma yaw {:.4f} rad",
                              ok ? "PASS" : "FAIL", max_pos_err, max_yaw_err * 180.0 / M_PI, pd,
                              std::sqrt(0.5 * (ekf.covariance()(0, 0) + ekf.covariance()(1, 1))),
                              std::sqrt(ekf.covariance()(2, 2)));
    return ok;
}

} // namespace rc::openfield
