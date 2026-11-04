#include "openfield_variants.h"

#include <algorithm>
#include <cmath>

namespace rc::openfield
{

void AntennaHistory::push(std::int64_t clock_ms, const Eigen::Vector2d &xy)
{
    if (not h_.empty() and clock_ms <= h_.back().first) return;
    h_.emplace_back(clock_ms, xy);
    while (h_.size() > 600) h_.pop_front();   // ~5 s at the IMU's 120 Hz: far longer than any receiver latency
}

std::optional<Eigen::Vector2d> AntennaHistory::at(std::int64_t t) const
{
    if (h_.size() < 2 or t < h_.front().first or t > h_.back().first) return std::nullopt;
    const auto hi = std::ranges::lower_bound(h_, t, {}, &std::pair<std::int64_t, Eigen::Vector2d>::first);
    if (hi == h_.begin()) return hi->second;
    const auto lo = std::prev(hi);
    const double a = hi->first == lo->first ? 0.0
                   : static_cast<double>(t - lo->first) / static_cast<double>(hi->first - lo->first);
    return Eigen::Vector2d(lo->second + a * (hi->second - lo->second));
}

GpsDiag fuse_gps(OpenFieldEkf &ekf, const GpsModel &m, const GpsFix &fix, double fixed_sigma,
                 const AntennaHistory &hist, std::int64_t now_clock_ms)
{
    GpsDiag d;
    if (not ekf.initialized() or not fix.valid) return d;
    d.sigma = m.receiver_sigma and fix.has_status and fix.sigma > 0.0 ? fix.sigma : fixed_sigma;
    Eigen::Vector2d z = fix.xy;
    if (m.lag_comp and fix.has_status and fix.meas_ms > 0 and now_clock_ms > fix.meas_ms)
        if (const auto then = hist.at(fix.meas_ms); then.has_value())
        {
            // Where the antenna moved since the fix was measured, by the filter's own prediction.
            z += ekf.predicted_antenna() - *then;
            d.lag_ms = static_cast<double>(now_clock_ms - fix.meas_ms);
        }
    if (m.student)
        d.d2 = ekf.correct_gps_student(z, d.sigma, m.nu, &d.w);
    else
        d.d2 = ekf.correct_gps(z, d.sigma);
    return d;
}

void Shadow::record(std::int64_t stamp_ms)
{
    if (not ekf.initialized()) return;
    if (not history.empty() and std::get<0>(history.back()) >= stamp_ms) return;
    history.emplace_back(stamp_ms, ekf.state(), ekf.covariance());
    while (history.size() > 600) history.pop_front();
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> Shadow::at(std::int64_t t) const
{
    if (history.size() < 2 or t < std::get<0>(history.front()) or t > std::get<0>(history.back()))
        return std::nullopt;
    auto hi = std::ranges::lower_bound(history, t, {}, [](const auto &e) { return std::get<0>(e); });
    if (hi == history.begin()) return std::pair{std::get<1>(*hi), std::get<2>(*hi)};
    const auto lo = std::prev(hi);
    const double a = std::get<0>(*hi) == std::get<0>(*lo) ? 0.0
                   : static_cast<double>(t - std::get<0>(*lo)) / static_cast<double>(std::get<0>(*hi) - std::get<0>(*lo));
    Eigen::Vector3d x = std::get<1>(*lo) + a * (std::get<1>(*hi) - std::get<1>(*lo));
    x(2) = wrap_pi(std::get<1>(*lo)(2) + a * wrap_pi(std::get<1>(*hi)(2) - std::get<1>(*lo)(2)));
    return std::pair{x, a < 0.5 ? std::get<2>(*lo) : std::get<2>(*hi)};
}

std::vector<Shadow> make_shadows(const EkfParams &p, double nu)
{
    std::vector<Shadow> s(4);
    s[0].name = "A_current";
    s[1].name = "B_receiver";
    s[1].gps = {.receiver_sigma = true, .student = true, .nu = nu, .lag_comp = true};
    s[2].name = "C_no_gnss";
    s[2].use_gps = false;
    s[3].name = "D_no_yaw";
    s[3].use_yaw = false;
    for (auto &f : s) f.ekf.set_params(p);
    return s;
}

} // namespace rc::openfield
