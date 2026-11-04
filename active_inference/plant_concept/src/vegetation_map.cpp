#include "vegetation_map.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <random>
#include <unordered_set>

namespace rc::veg
{

int VegetationMap::add_frame(const std::vector<Eigen::Vector3f>& pts, std::int64_t now_ms)
{
    // This frame's weight: the time it stands for, in reference looks (VegetationParams::look_period_ms).
    float w = 1.0f;
    if (now_ms >= 0)
    {
        if (last_frame_ms_ >= 0 and p_.look_period_ms > 0.0f)
            w = std::clamp(static_cast<float>(now_ms - last_frame_ms_) / p_.look_period_ms, 0.0f, 1.0f);
        last_frame_ms_ = now_ms;
    }
    // One unit per cell per frame (see the header): collect this frame's cells first.
    std::unordered_map<std::int64_t, float> hit;   // cell -> highest point this frame
    for (const auto& q : pts)
    {
        if (not q.allFinite() or q.z() < p_.z_min or q.z() > p_.z_max)
            continue;
        const auto k = key(index(q.x()), index(q.y()));
        auto [it, inserted] = hit.try_emplace(k, q.z());
        if (not inserted)
            it->second = std::max(it->second, q.z());
    }
    for (const auto& [k, z] : hit)
    {
        auto& c = evidence_[k];
        c.evidence += w;
        c.z_top = std::max(c.z_top, z);
    }
    return static_cast<int>(hit.size());
}

VegetationMap::Shift VegetationMap::disc_stats(const Eigen::Vector2f& c) const
{
    Shift s{c};
    const float R = p_.plant_radius_m, R2 = R * R;
    Eigen::Vector2f sum = Eigen::Vector2f::Zero();
    Eigen::Vector2f sum2 = Eigen::Vector2f::Zero();
    int n = 0;
    for (int ix = index(c.x() - R); ix <= index(c.x() + R); ++ix)
        for (int iy = index(c.y() - R); iy <= index(c.y() + R); ++iy)
        {
            const auto it = evidence_.find(key(ix, iy));
            if (it == evidence_.end())
                continue;
            const Eigen::Vector2f q(centre_of(ix), centre_of(iy));
            if ((q - c).squaredNorm() > R2)
                continue;
            const float w = it->second.evidence;
            s.mass += w;
            sum += w * q;
            sum2 += w * q.cwiseProduct(q);
            s.top = std::max(s.top, it->second.z_top);
            ++n;
        }
    if (s.mass > 0.0f)
    {
        s.c = sum / s.mass;
        // Spread of the evidence in the disc, shrunk by the number of cells that carry it (each cell is one
        // roughly independent look at where the canopy is), plus the grid quantisation.
        const Eigen::Vector2f var = (sum2 / s.mass - s.c.cwiseProduct(s.c)).cwiseMax(0.0f);
        s.sigma = std::sqrt(0.5f * (var.x() + var.y()) / static_cast<float>(std::max(1, n))
                            + p_.cell_m * p_.cell_m / 12.0f);
    }
    return s;
}

VegetationMap::Shift VegetationMap::mean_shift(Eigen::Vector2f start, int iterations) const
{
    Shift s = disc_stats(start);
    for (int i = 0; i < iterations and s.mass > 0.0f; ++i)
    {
        const Shift next = disc_stats(s.c);
        const bool settled = (next.c - s.c).norm() < 0.25f * p_.cell_m;
        s = next;
        if (settled)
            break;
    }
    return s;
}

VegetationMap::Changes VegetationMap::update(int max_births)
{
    Changes ch;
    const float R = p_.plant_radius_m;

    // 1. Re-centre every plant on what has been seen of it so far — but never INTO another plant's space.
    // ★Without that bound, along a continuous hedge (a row seen from outside reads as one band of foliage) two
    // neighbours' mean shifts slid towards the same density peak, met inside 2R, the younger was merged away,
    // the evidence it left seeded a new birth in the gap, and that one slid in again: measured 598 plant nodes
    // created and 556 deleted in one 10-minute run. A plant keeps its place rather than overlap another (older OR
    // younger: an old plant sliding onto a young one ends the same way), so after birth nothing needs merging.
    std::sort(plants_.begin(), plants_.end(), [](const Plant& a, const Plant& b) { return a.id < b.id; });
    for (std::size_t i = 0; i < plants_.size(); ++i)
    {
        auto& pl = plants_[i];
        const Shift s = mean_shift(pl.c, 3);
        if (s.mass <= 0.0f)
            continue;
        bool crowds = false;
        for (std::size_t j = 0; j < plants_.size(); ++j)
            crowds = crowds or (j != i and (plants_[j].c - s.c).norm() < 2.0f * R);
        if (not crowds)
            pl.c = s.c;
        const Shift here = crowds ? disc_stats(pl.c) : s;
        pl.mass = here.mass;
        pl.height = here.top;
        pl.sigma_xy = here.sigma;
    }

    // 2. No overlap: two discs closer than 2R are one plant seen twice. The older (lower id) keeps it.
    std::sort(plants_.begin(), plants_.end(), [](const Plant& a, const Plant& b) { return a.id < b.id; });
    std::vector<bool> gone(plants_.size(), false);
    for (std::size_t i = 0; i < plants_.size(); ++i)
        for (std::size_t j = i + 1; j < plants_.size(); ++j)
            if (not gone[i] and not gone[j] and (plants_[i].c - plants_[j].c).norm() < 2.0f * R)
            {
                gone[j] = true;
                ch.removed.push_back(plants_[j].id);
            }
    std::vector<Plant> kept;
    for (std::size_t i = 0; i < plants_.size(); ++i)
        if (not gone[i])
            kept.push_back(plants_[i]);
    plants_ = std::move(kept);

    // 3. Births, strongest unexplained evidence first.
    const auto free_of_plants = [&](const Eigen::Vector2f& c, float clearance)
    {
        return std::ranges::none_of(plants_, [&](const Plant& pl) { return (pl.c - c).norm() < clearance; });
    };
    std::vector<std::pair<float, Eigen::Vector2f>> seeds;
    seeds.reserve(evidence_.size());
    for (const auto& [k, cell] : evidence_)
    {
        const int ix = static_cast<int>(k >> 32);
        const int iy = static_cast<int>(static_cast<std::int32_t>(static_cast<std::uint32_t>(k & 0xffffffff)));
        const Eigen::Vector2f q(centre_of(ix), centre_of(iy));
        if (free_of_plants(q, R))
            seeds.emplace_back(cell.evidence, q);
    }
    std::ranges::sort(seeds, [](const auto& a, const auto& b) { return a.first > b.first; });
    int births = 0;
    for (const auto& [w, q] : seeds)
    {
        if (births >= max_births)
            break;
        if (not free_of_plants(q, R))          // swallowed by a plant born earlier in this pass
            continue;
        const Shift s = mean_shift(q, 6);
        if (s.mass < p_.birth_mass or not free_of_plants(s.c, 2.0f * R))
            continue;
        plants_.push_back({next_id_++, s.c, s.top, s.mass, s.sigma});
        ch.born.push_back(plants_.back().id);
        ++births;
    }
    return ch;
}

bool VegetationMap::self_test(std::string* report)
{
    // A row of 6 trees 1.25 m apart (x = 0, y = -3.125 .. 3.125), canopy radius 0.45 m, seen for 12 frames,
    // each frame sampling only part of every canopy. Expect one plant per tree, each within 0.15 m of the
    // truth, none overlapping, and nothing from the soil points.
    VegetationParams p;
    VegetationMap m(p);
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> u01(0.0f, 1.0f);
    std::vector<Eigen::Vector2f> truth;
    for (int i = 0; i < 6; ++i)
        truth.emplace_back(0.0f, -3.125f + 1.25f * static_cast<float>(i));
    for (int f = 0; f < 12; ++f)
    {
        std::vector<Eigen::Vector3f> pts;
        for (const auto& t : truth)
            for (int k = 0; k < 150; ++k)
            {
                const float a = 6.2832f * u01(rng), r = 0.45f * std::sqrt(u01(rng));
                pts.emplace_back(t.x() + r * std::cos(a), t.y() + r * std::sin(a), 0.6f + 2.2f * u01(rng));
            }
        for (int k = 0; k < 200; ++k)   // soil the mask bled onto: must not make plants
            pts.emplace_back(-2.0f + 4.0f * u01(rng), -4.0f + 8.0f * u01(rng), 0.1f * u01(rng));
        m.add_frame(pts);
        m.update();
    }
    float worst = 0.0f;
    int matched = 0;
    for (const auto& t : truth)
    {
        float best = 1e9f;
        for (const auto& pl : m.plants())
            best = std::min(best, (pl.c - t).norm());
        worst = std::max(worst, best);
        matched += best < 0.15f ? 1 : 0;
    }
    float min_sep = 1e9f;
    for (std::size_t i = 0; i < m.plants().size(); ++i)
        for (std::size_t j = i + 1; j < m.plants().size(); ++j)
            min_sep = std::min(min_sep, (m.plants()[i].c - m.plants()[j].c).norm());
    // A continuous HEDGE (a row of foliage with no gaps, as seen from outside the row): plants tile it, and over
    // many updates no plant may be merged away and re-born (graph churn).
    VegetationMap h(p);
    int removed = 0, born = 0;
    for (int f = 0; f < 60; ++f)
    {
        std::vector<Eigen::Vector3f> pts;
        for (int k = 0; k < 900; ++k)
            pts.emplace_back(10.0f + 0.4f * (u01(rng) - 0.5f), -4.0f + 8.0f * u01(rng), 0.6f + 2.0f * u01(rng));
        h.add_frame(pts);
        const auto ch = h.update();
        removed += static_cast<int>(ch.removed.size());
        born += static_cast<int>(ch.born.size());
    }
    const bool hedge_ok = removed == 0 and born == static_cast<int>(h.plants().size());

    const bool ok = m.plants().size() == truth.size() and matched == static_cast<int>(truth.size())
                    and min_sep >= 2.0f * p.plant_radius_m and hedge_ok;
    if (report)
        *report = std::format("VegetationMap::self_test {}: {} plants for {} trees, worst centre error {:.3f} m, "
                              "min separation {:.2f} m | hedge: {} plants, {} merged away over 60 updates",
                              ok ? "PASS" : "FAIL", m.plants().size(), truth.size(), worst,
                              m.plants().size() > 1 ? min_sep : 0.0f, h.plants().size(), removed);
    return ok;
}

}  // namespace rc::veg
