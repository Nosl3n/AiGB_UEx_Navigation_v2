#include "mixed_pixel_filter.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <vector>

namespace
{
constexpr float kRingGapRad = 0.5f * std::numbers::pi_v<float> / 180.f;   // as specificworker.cpp's ring table

struct Ring
{
    std::vector<std::size_t> idx;   // point indices, sorted by phi
    float step = 0.f;               // median azimuth spacing (rad)
};
}  // namespace

std::size_t mixed_pixel_filter(RoboCompLidar3D::TData& data, const MixedPixelFilterConfig& cfg)
{
    auto& pts = data.points;
    const std::size_t n = pts.size();
    if (n < 3)
        return 0;

    // 1. Rings: cluster the beam elevations (theta = acos(z/r), 0 up .. pi down; see processLidarData).
    std::vector<std::size_t> by_theta(n);
    for (std::size_t i = 0; i < n; ++i)
        by_theta[i] = i;
    std::ranges::sort(by_theta, {}, [&](std::size_t i) { return pts[i].theta; });
    std::vector<Ring> rings;
    std::vector<int> ring_of(n, -1);
    for (std::size_t k = 0; k < n; ++k)
    {
        const std::size_t i = by_theta[k];
        if (k == 0 or pts[i].theta - pts[by_theta[k - 1]].theta > kRingGapRad)
            rings.emplace_back();
        rings.back().idx.push_back(i);
        ring_of[i] = static_cast<int>(rings.size()) - 1;
    }
    std::vector<std::size_t> pos(n);   // position of each point inside its ring's phi order
    for (auto& rg : rings)
    {
        std::ranges::sort(rg.idx, {}, [&](std::size_t i) { return pts[i].phi; });
        std::vector<float> d;
        for (std::size_t k = 0; k + 1 < rg.idx.size(); ++k)
            d.push_back(pts[rg.idx[k + 1]].phi - pts[rg.idx[k]].phi);
        if (not d.empty())
        {
            std::ranges::nth_element(d, d.begin() + static_cast<std::ptrdiff_t>(d.size() / 2));
            rg.step = d[d.size() / 2];
        }
        for (std::size_t k = 0; k < rg.idx.size(); ++k)
            pos[rg.idx[k]] = k;
    }
    // Typical ring spacing, so a ring missing from this sweep does not make two far rings "adjacent".
    std::vector<float> gaps;
    for (std::size_t r = 0; r + 1 < rings.size(); ++r)
        gaps.push_back(pts[rings[r + 1].idx.front()].theta - pts[rings[r].idx.front()].theta);
    float ring_gap = 0.f;
    if (not gaps.empty())
    {
        std::ranges::nth_element(gaps, gaps.begin() + static_cast<std::ptrdiff_t>(gaps.size() / 2));
        ring_gap = gaps[gaps.size() / 2];
    }

    const auto wrap = [](float a)
    { return std::remainder(a, 2.f * std::numbers::pi_v<float>); };

    // 2. Keep a point if any of its 8 neighbours agrees on range.
    std::vector<char> keep(n, 0);
    #pragma omp parallel for schedule(static)
    for (std::size_t i = 0; i < n; ++i)
    {
        const auto& p = pts[i];
        const float tol = std::max(cfg.abs_tol_mm, cfg.rel_tol * p.r);
        const auto agrees = [&](std::size_t j) { return std::abs(pts[j].r - p.r) < tol; };
        const int r = ring_of[i];
        const Ring& rg = rings[r];
        const float max_dphi = 1.5f * std::max(rg.step, 1e-4f);
        bool ok = false;
        // same ring, previous / next in azimuth (wrapping round the full turn)
        const std::size_t m = rg.idx.size();
        if (m > 1)
            for (const std::size_t k : {(pos[i] + m - 1) % m, (pos[i] + 1) % m})
                if (k != pos[i] and std::abs(wrap(pts[rg.idx[k]].phi - p.phi)) < max_dphi and agrees(rg.idx[k]))
                    ok = true;
        // adjacent rings: the nearest azimuth and its two neighbours
        for (const int dr : {-1, 1})
        {
            if (ok)
                break;
            const int q = r + dr;
            if (q < 0 or q >= static_cast<int>(rings.size()))
                continue;
            const Ring& rq = rings[q];
            if (std::abs(pts[rq.idx.front()].theta - p.theta) > 1.6f * ring_gap + kRingGapRad)
                continue;   // a ring is missing in between: not neighbours
            const auto it = std::ranges::lower_bound(rq.idx, p.phi, {}, [&](std::size_t j) { return pts[j].phi; });
            const std::size_t mq = rq.idx.size();
            const std::size_t c = static_cast<std::size_t>(it - rq.idx.begin()) % mq;
            const float max_dq = 1.5f * std::max(std::max(rq.step, rg.step), 1e-4f);
            for (const std::size_t k : {(c + mq - 2) % mq, (c + mq - 1) % mq, c, (c + 1) % mq})
                if (std::abs(wrap(pts[rq.idx[k]].phi - p.phi)) < max_dq and agrees(rq.idx[k]))
                {
                    ok = true;
                    break;
                }
        }
        keep[i] = ok ? 1 : 0;
    }

    std::size_t w = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (keep[i])
            pts[w++] = pts[i];
    const std::size_t removed = n - w;
    pts.resize(w);
    return removed;
}

bool mixed_pixel_filter_self_test(std::string* report)
{
    // 16 rings x 360 columns (1 deg). A wall at 8 m all round; a pole 3 columns wide at 4 m in front of it
    // (columns 100-102, all rings); one mixed pixel at 6 m on the pole's edge (ring 8, column 103); one lone
    // spike at 2 m in open wall (ring 4, column 200). Expect: both removed, nothing else.
    RoboCompLidar3D::TData d;
    const float deg = std::numbers::pi_v<float> / 180.f;
    for (int c = 0; c < 360; ++c)
        for (int r = 0; r < 16; ++r)
        {
            float range = 8000.f;
            if (c >= 100 and c <= 102) range = 4000.f;
            if (r == 8 and c == 103) range = 6000.f;
            if (r == 4 and c == 200) range = 2000.f;
            const float el = (-15.f + 2.f * static_cast<float>(r)) * deg;
            const float phi = std::remainder(static_cast<float>(c) * deg, 2.f * std::numbers::pi_v<float>);
            RoboCompLidar3D::TPoint p{};
            p.x = range * std::cos(el) * std::sin(phi);
            p.y = range * std::cos(el) * std::cos(phi);
            p.z = range * std::sin(el);
            p.r = range;
            p.phi = phi;
            p.theta = std::acos(std::clamp(p.z / p.r, -1.f, 1.f));
            d.points.push_back(p);
        }
    const std::size_t before = d.points.size();
    const std::size_t removed = mixed_pixel_filter(d, {});
    const auto present = [&](float range)
    { return std::ranges::any_of(d.points, [&](const auto& p) { return std::abs(p.r - range) < 1.f; }); };
    const bool ok = removed == 2 and not present(6000.f) and not present(2000.f) and present(4000.f)
                    and d.points.size() == before - 2;
    if (report)
        *report = std::format("MixedPixelFilter self_test {}: removed {} of {} (expected the 2 planted mixed pixels)",
                              ok ? "PASS" : "FAIL", removed, before);
    return ok;
}
