// gnss_degradation.h — SIMULATION-ONLY degradation of the GPS fix: what a real RTK receiver does under an
// orchard canopy, which the Webots GPS device (Gaussian noise around the truth) does not.
//
// PURE C++ (no Webots, no Ice) so it can be self-tested. Switched by [GnssDegradation] Enabled in
// etc/config_husky; OFF by default, and with it OFF the bridge serves exactly what it served before.
//
// ── MODEL ────────────────────────────────────────────────────────────────────────────────────────────────────
//   * SKY: a fixed constellation of n_sats satellites (azimuth, elevation above the mask), drawn from the seed.
//     Each epoch casts one ray per satellite from the antenna through the canopies of the world (vertical
//     cylinders: trunk-top .. tree-top, canopy radius) and accumulates the foliage chord it crosses. A chord
//     over block_chord_m blocks the satellite; a shorter one attenuates it (counted as "partial").
//   * FIX STATE from the visible count, with the receiver's hysteresis: losing satellites downgrades at once,
//     regaining them upgrades only after the geometry stayed good for reacq_*_s (an RTK float has to re-solve
//     its ambiguities before it is FIXED again).  4 = RTK fixed, 5 = RTK float, 1 = single, 0 = no fix
//     (NMEA GGA quality codes).
//   * NOISE per state: a Gauss-Markov error (sigma_*, tau_*) — receiver errors are time-correlated, never white.
//     The REPORTED sigma is that same sigma scaled by a DOP-like sqrt(fix_min_sats / visible).
//   * MULTIPATH: a jump of random direction and Rayleigh(mp_sigma_m) size starts at a rate proportional to the
//     share of blocked satellites (reflections off foliage), then decays with mp_tau_s. It is NOT in the
//     reported sigma: receivers do not see it — which is what a heavy-tailed likelihood is for.
//   * ZONES: declared rectangles where there is no fix at all (denied) or where half the sky is lost.
//   * OUTAGES: random cuts (Poisson rate per minute, exponential duration).
//   * LATENCY: a fix is delivered latency_ms after it was measured, carrying its measurement timestamp.
#pragma once

#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

namespace gnss
{

struct Canopy { double x, y, z_bottom, z_top, r; };
struct Zone { double x0, y0, x1, y1; bool denied; };   // denied = no fix; else the visible count is halved

struct Params
{
    bool enabled = false;
    std::uint32_t seed = 1;
    int n_sats = 16;
    double mask_elev_deg = 10.0;
    double block_chord_m = 0.40;          // foliage crossed by a ray that blocks it
    double partial_chord_m = 0.05;        // ... below this it is a clear line of sight
    int fix_min_sats = 12, float_min_sats = 8, single_min_sats = 5;
    double reacq_fix_s = 8.0, reacq_float_s = 2.0;
    double sigma_fix_m = 0.015, sigma_float_m = 0.25, sigma_single_m = 1.2;   // per-axis 1 sigma
    double tau_fix_s = 1.0, tau_float_s = 5.0, tau_single_s = 15.0;
    double mp_rate_hz = 0.15;             // multipath onset rate at a fully blocked sky
    double mp_sigma_m = 0.8, mp_tau_s = 4.0;
    double latency_ms = 0.0;
    double outage_rate_per_min = 0.0, outage_mean_s = 5.0;
    std::vector<Zone> zones;
};

struct Fix
{
    bool valid = false;
    double x = 0, y = 0, z = 0;           // delivered antenna position (local metres)
    double sigma = 0;                     // reported per-axis 1 sigma (m)
    int fix_type = 0;                     // 4 fixed, 5 float, 1 single, 0 none
    int n_vis = 0, n_partial = 0, n_above = 0;
    double mp = 0;                        // multipath offset magnitude in this fix (m) — truth, for the logs
    double err = 0;                       // |delivered - true| (m) — truth, for the logs
    std::int64_t meas_ms = 0;             // when it was measured (simulation clock)
};

class Degrader
{
public:
    explicit Degrader(Params p = {});
    void set_params(const Params& p);
    void set_canopy(std::vector<Canopy> c) { canopy_ = std::move(c); }
    [[nodiscard]] const Params& params() const { return p_; }
    [[nodiscard]] std::size_t canopy_count() const { return canopy_.size(); }

    // One receiver epoch: the TRUE antenna position and the simulation time. Returns the fix that is DELIVERED
    // now (the one measured latency_ms ago), or an invalid fix while nothing is due.
    Fix epoch(double x, double y, double z, std::int64_t sim_ms);

    // Satellites visible from a point (the sky model alone), for tests and logs.
    void visibility(double x, double y, double z, int& n_vis, int& n_partial, int& n_above) const;

    static bool self_test(std::string* report = nullptr);

private:
    Params p_;
    std::vector<Canopy> canopy_;
    struct Sat { double el, az; };
    std::vector<Sat> sats_;
    std::mt19937 rng_;
    int state_ = 4;
    double good_since_s_ = -1, float_since_s_ = -1;
    double ex_ = 0, ey_ = 0;                       // Gauss-Markov error
    double mpx_ = 0, mpy_ = 0;                     // multipath offset
    double last_s_ = -1;
    double outage_until_s_ = -1;
    std::deque<Fix> queue_;
    void build_constellation();
    [[nodiscard]] double chord(double px, double py, double pz, double dx, double dy, double dz, const Canopy& c) const;
};

}  // namespace gnss
