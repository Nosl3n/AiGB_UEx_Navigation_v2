#include "gnss_degradation.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace gnss
{

Degrader::Degrader(Params p) : p_(std::move(p)), rng_(p_.seed)
{
    build_constellation();
}

void Degrader::set_params(const Params& p)
{
    p_ = p;
    rng_.seed(p_.seed);
    build_constellation();
    state_ = 4;
    good_since_s_ = float_since_s_ = last_s_ = outage_until_s_ = -1;
    ex_ = ey_ = mpx_ = mpy_ = 0;
    queue_.clear();
}

void Degrader::build_constellation()
{
    // Uniform over the sky above the mask: sin(elevation) uniform gives equal solid angle per band.
    std::mt19937 g(p_.seed * 7919u + 13u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const double s0 = std::sin(p_.mask_elev_deg * M_PI / 180.0);
    sats_.clear();
    for (int i = 0; i < p_.n_sats; ++i)
        sats_.push_back({std::asin(s0 + (1.0 - s0) * u01(g)), 2.0 * M_PI * u01(g)});
}

// Length of the ray p + s d (s >= 0, d unit) inside a vertical cylinder of foliage.
double Degrader::chord(double px, double py, double pz, double dx, double dy, double dz, const Canopy& c) const
{
    const double ox = px - c.x, oy = py - c.y;
    const double a = dx * dx + dy * dy;
    if (a < 1e-12) return (ox * ox + oy * oy < c.r * c.r and pz < c.z_top) ? c.z_top - std::max(pz, c.z_bottom) : 0.0;
    const double b = 2.0 * (ox * dx + oy * dy), cc = ox * ox + oy * oy - c.r * c.r;
    const double disc = b * b - 4.0 * a * cc;
    if (disc <= 0.0) return 0.0;
    const double sq = std::sqrt(disc);
    double lo = (-b - sq) / (2.0 * a), hi = (-b + sq) / (2.0 * a);
    if (dz > 1e-9)
    {
        lo = std::max(lo, (c.z_bottom - pz) / dz);
        hi = std::min(hi, (c.z_top - pz) / dz);
    }
    lo = std::max(lo, 0.0);
    return std::max(0.0, hi - lo);
}

void Degrader::visibility(double x, double y, double z, int& n_vis, int& n_partial, int& n_above) const
{
    n_vis = n_partial = 0;
    n_above = static_cast<int>(sats_.size());
    for (const auto& s : sats_)
    {
        const double dx = std::cos(s.el) * std::cos(s.az), dy = std::cos(s.el) * std::sin(s.az), dz = std::sin(s.el);
        // Only canopies the ray can reach before it climbs above every tree top (8 m reach is plenty here).
        double L = 0.0;
        for (const auto& c : canopy_)
            if (std::abs(c.x - x) < 8.0 and std::abs(c.y - y) < 8.0 and c.z_top > z)
                L += chord(x, y, z, dx, dy, dz, c);
        if (L < p_.block_chord_m) ++n_vis;
        if (L >= p_.partial_chord_m and L < p_.block_chord_m) ++n_partial;
    }
    for (const auto& zn : p_.zones)
        if (x >= std::min(zn.x0, zn.x1) and x <= std::max(zn.x0, zn.x1) and y >= std::min(zn.y0, zn.y1)
            and y <= std::max(zn.y0, zn.y1))
            n_vis = zn.denied ? 0 : n_vis / 2;
}

Fix Degrader::epoch(double x, double y, double z, std::int64_t sim_ms)
{
    const double t = 1e-3 * static_cast<double>(sim_ms);
    const double dt = last_s_ < 0 ? 0.1 : std::max(1e-3, t - last_s_);
    last_s_ = t;
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    Fix f;
    f.meas_ms = sim_ms;
    visibility(x, y, z, f.n_vis, f.n_partial, f.n_above);

    // Random outage (Poisson onset, exponential length).
    if (t >= outage_until_s_ and p_.outage_rate_per_min > 0.0 and u01(rng_) < p_.outage_rate_per_min / 60.0 * dt)
        outage_until_s_ = t + std::exponential_distribution<double>(1.0 / std::max(0.1, p_.outage_mean_s))(rng_);
    const bool cut = t < outage_until_s_;

    // Fix state with hysteresis: down at once, up only after the geometry held long enough.
    const int n = cut ? 0 : f.n_vis;
    const int target = n >= p_.fix_min_sats ? 4 : n >= p_.float_min_sats ? 5 : n >= p_.single_min_sats ? 1 : 0;
    if (n >= p_.fix_min_sats) { if (good_since_s_ < 0) good_since_s_ = t; } else good_since_s_ = -1;
    if (n >= p_.float_min_sats) { if (float_since_s_ < 0) float_since_s_ = t; } else float_since_s_ = -1;
    const auto rank = [](int s) { return s == 4 ? 3 : s == 5 ? 2 : s == 1 ? 1 : 0; };
    if (rank(target) < rank(state_))
        state_ = target;
    else if (target == 4 and state_ != 4 and good_since_s_ >= 0 and t - good_since_s_ >= p_.reacq_fix_s)
        state_ = 4;
    else if (rank(state_) < 2 and rank(target) >= 2 and float_since_s_ >= 0 and t - float_since_s_ >= p_.reacq_float_s)
        state_ = 5;
    else if (state_ == 0 and target != 0)
        state_ = 1;

    // Multipath: onset rate ~ share of the sky that is blocked; a jump of random direction, decaying.
    const double blocked = f.n_above > 0 ? 1.0 - static_cast<double>(f.n_vis) / f.n_above : 0.0;
    const double decay = std::exp(-dt / std::max(0.1, p_.mp_tau_s));
    mpx_ *= decay;
    mpy_ *= decay;
    if (state_ != 0 and u01(rng_) < p_.mp_rate_hz * blocked * dt)
    {
        const double mag = p_.mp_sigma_m * std::sqrt(-2.0 * std::log(std::max(1e-12, u01(rng_))));
        const double ang = 2.0 * M_PI * u01(rng_);
        mpx_ += mag * std::cos(ang);
        mpy_ += mag * std::sin(ang);
        if (state_ == 4) state_ = 5;   // a multipath jump costs the fixed ambiguities
        good_since_s_ = t;
    }

    // Gauss-Markov receiver error of the current state.
    const double sig = state_ == 4 ? p_.sigma_fix_m : state_ == 5 ? p_.sigma_float_m : p_.sigma_single_m;
    const double tau = state_ == 4 ? p_.tau_fix_s : state_ == 5 ? p_.tau_float_s : p_.tau_single_s;
    const double a = std::exp(-dt / std::max(1e-3, tau));
    const double q = sig * std::sqrt(std::max(0.0, 1.0 - a * a));
    ex_ = a * ex_ + q * n01(rng_);
    ey_ = a * ey_ + q * n01(rng_);

    f.fix_type = state_;
    f.valid = state_ != 0;
    f.x = x + ex_ + mpx_;
    f.y = y + ey_ + mpy_;
    f.z = z;
    const int min_n = state_ == 4 ? p_.fix_min_sats : state_ == 5 ? p_.float_min_sats : p_.single_min_sats;
    f.sigma = sig * std::sqrt(static_cast<double>(min_n) / std::max(1, n));
    f.mp = std::hypot(mpx_, mpy_);
    f.err = std::hypot(f.x - x, f.y - y);

    // Latency: deliver what was measured latency_ms ago.
    queue_.push_back(f);
    Fix out;
    while (not queue_.empty() and static_cast<double>(sim_ms - queue_.front().meas_ms) >= p_.latency_ms)
    {
        out = queue_.front();
        queue_.pop_front();
    }
    return out;
}

bool Degrader::self_test(std::string* report)
{
    // A row of canopies along x = 0; a receiver 1.25 m beside it sees less sky than one in the open, loses the
    // RTK fix under it, gets it back only after reacq_fix_s in the open, and a denied zone gives no fix.
    Params p;
    p.enabled = true;
    p.seed = 3;
    p.zones = {{-1.0, 20.0, 1.0, 22.0, true}};
    Degrader d(p);
    std::vector<Canopy> c;
    for (int i = 0; i < 12; ++i) c.push_back({0.0, -7.0 + 1.25 * i, 0.7, 3.0, 0.55});
    for (int i = 0; i < 12; ++i) c.push_back({2.5, -7.0 + 1.25 * i, 0.7, 3.0, 0.55});
    d.set_canopy(c);
    int v_open, v_row, pp, na;
    d.visibility(1.25, -15.0, 1.1, v_open, pp, na);
    d.visibility(1.25, 0.0, 1.1, v_row, pp, na);
    // Drive: 10 s in the open, 20 s under the row, 20 s in the open, then into the denied zone.
    int type_row = -1, type_back = -1, type_denied = -1;
    std::int64_t t = 0;
    for (; t < 10000; t += 100) d.epoch(0.0, -15.0, 1.1, t);
    const int type_open = d.epoch(0.0, -15.0, 1.1, t).fix_type;
    for (; t < 30000; t += 100) type_row = d.epoch(1.25, 0.0, 1.1, t).fix_type;
    const int just_out = d.epoch(1.25, -15.0, 1.1, t += 100).fix_type;
    for (; t < 50000; t += 100) type_back = d.epoch(1.25, -15.0, 1.1, t).fix_type;
    for (int k = 0; k < 10; ++k, t += 100) type_denied = d.epoch(0.0, 21.0, 1.1, t).fix_type;
    const bool ok = v_open > v_row and type_open == 4 and type_row != 4 and just_out != 4 and type_back == 4
                    and type_denied == 0;
    if (report)
        *report = std::format("gnss::Degrader::self_test {}: sky {} sats open / {} beside a row, fix type open {} -> "
                              "under canopy {} -> just out {} -> after reacquisition {} -> denied zone {}",
                              ok ? "PASS" : "FAIL", v_open, v_row, type_open, type_row, just_out, type_back,
                              type_denied);
    return ok;
}

}  // namespace gnss
