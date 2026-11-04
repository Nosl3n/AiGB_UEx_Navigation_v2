#include "trunk_map.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <random>
#include <unordered_set>

namespace rc::trunk
{

Eigen::Vector2f TrunkMap::cell_xy(std::int64_t k) const
{
    const int ix = static_cast<int>(k >> 32);
    const int iy = static_cast<int>(static_cast<std::int32_t>(static_cast<std::uint32_t>(k & 0xffffffff)));
    return {centre_of(ix), centre_of(iy)};
}

Trunk* TrunkMap::nearest_trunk(const Eigen::Vector2f& q, float gate)
{
    Trunk* best = nullptr;
    float best_d = gate;
    for (auto& t : trunks_)
        if (const float d = (t.c - q).norm(); d < best_d)
        {
            best = &t;
            best_d = d;
        }
    return best;
}

Eigen::Vector2f TrunkMap::median_xy(std::vector<Eigen::Vector2f>& pts)
{
    // Coordinate-wise median: the edge pixels of a thin trunk take their depth from what is behind or in front
    // of it and streak along the ray; a mean follows the streak, a median stays on the trunk.
    const auto mid = static_cast<std::ptrdiff_t>(pts.size() / 2);
    std::nth_element(pts.begin(), pts.begin() + mid, pts.end(), [](const auto& a, const auto& b) { return a.x() < b.x(); });
    const float x = pts[static_cast<std::size_t>(mid)].x();
    std::nth_element(pts.begin(), pts.begin() + mid, pts.end(), [](const auto& a, const auto& b) { return a.y() < b.y(); });
    return {x, pts[static_cast<std::size_t>(mid)].y()};
}

bool TrunkMap::inside(const Eigen::Vector2f& q, const std::vector<Occluder>& discs)
{
    return std::ranges::any_of(discs, [&](const Occluder& d) { return (q - d.c).norm() < d.r; });
}

void TrunkMap::observe(Trunk& t, const Eigen::Vector2f& m, float sigma, float w)
{
    // Scalar Kalman update of a static point, with a little process noise so a localisation correction
    // (the whole map shifting under the robot) can still carry the trunk to where it is now seen.
    const float r2 = sigma * sigma;
    const float k = t.var / (t.var + r2);
    const Eigen::Vector2f next = t.c + k * (m - t.c);
    // Never INTO another trunk's space: two trunks meeting inside min_separation would be merged and one lost
    // (the same churn vegetation_map.cpp guards against). The observation still tightens the variance.
    const bool crowds = std::ranges::any_of(trunks_, [&](const Trunk& o)
                                            { return o.id != t.id and (o.c - next).norm() < p_.min_separation_m; });
    if (not crowds)
        t.c = next;
    t.var = (1.0f - k) * t.var + w * p_.drift_q;
}

bool TrunkMap::occluded(const Eigen::Vector2f& cam, const Trunk& t, const std::vector<Occluder>& occ) const
{
    const Eigen::Vector2f seg = t.c - cam;
    const float len2 = seg.squaredNorm();
    if (len2 < 1e-6f)
        return false;
    for (const auto& o : occ)
    {
        // The trunk's OWN canopy is centred on it and does not hide it from the side it is seen from.
        if ((o.c - t.c).norm() < o.r)
            continue;
        const float s = std::clamp((o.c - cam).dot(seg) / len2, 0.0f, 1.0f);
        if (s > 0.0f and s < 1.0f and (cam + s * seg - o.c).norm() < o.r)
            return true;
    }
    return false;
}

void TrunkMap::add_zed_frame(const std::vector<Eigen::Vector3f>& band_pts, const Eigen::Vector2f& cam,
                             float cam_heading, const std::vector<Occluder>& occluders,
                             const std::vector<Occluder>& people, bool looked, std::int64_t now_ms)
{
    // This look's weight: the time it stands for, in reference looks (TrunkParams::look_period_ms).
    const float w = last_look_ms_ < 0 or p_.look_period_ms <= 0.0f
                        ? 1.0f
                        : std::clamp(static_cast<float>(now_ms - last_look_ms_) / p_.look_period_ms, 0.0f, 1.0f);
    last_look_ms_ = now_ms;
    std::unordered_map<int, std::vector<Eigen::Vector2f>> acc;   // trunk id -> its band points this frame
    std::unordered_set<std::int64_t> cells;          // candidate cells lit this frame (one unit each)
    for (const auto& q : band_pts)
    {
        if (not q.allFinite() or q.z() < p_.band_z_min or q.z() > p_.band_z_max)
            continue;
        const Eigen::Vector2f xy(q.x(), q.y());
        if (inside(xy, people))
            continue;
        if (Trunk* t = nearest_trunk(xy, p_.assoc_gate_m); t != nullptr)
            acc[t->id].push_back(xy);
        else
            cells.insert(key(index(xy.x()), index(xy.y())));
    }
    for (const auto k : cells)
    {
        auto& c = grid_[k];
        c.evidence += w;
        c.last_ms = now_ms;
    }

    for (auto& t : trunks_)
    {
        const Eigen::Vector2f d = t.c - cam;
        const float range = d.norm();
        if (auto it = acc.find(t.id); it != acc.end())
        {
            // Seen: the points lie on the near face, so the axis is a trunk radius further along the ray.
            const Eigen::Vector2f m = median_xy(it->second);
            const Eigen::Vector2f ray = (m - cam).norm() > 1e-3f ? Eigen::Vector2f((m - cam).normalized()) : Eigen::Vector2f::Zero();
            observe(t, m + p_.trunk_radius_m * ray, p_.zed_sigma0 + p_.zed_sigma_k * range * range, w);
            t.existence.note_viewpoint(cam.x(), cam.y(), t.c.x(), t.c.y(), p_.cluster_r);
            t.existence.integrate(1.0f, w * p_.llr_hit);
            ++t.zed_hits;
            continue;
        }
        // Not seen. It is only a MISS if it should have been: a look that really happened, inside the field of
        // view and range, with nothing in between. Anything else is no evidence at all (HOLD).
        if (not looked or range > p_.zed_max_range or range < p_.zed_min_range)
            continue;
        const float off_axis = std::remainder(std::atan2(d.y(), d.x()) - cam_heading, 2.0f * static_cast<float>(M_PI));
        if (std::abs(off_axis) > p_.zed_half_fov or (p_.visibility_model and occluded(cam, t, occluders)))
            continue;
        // How resolvable a trunk is falls with range (fewer pixels on a 7 cm stem).
        const float p_vis = p_.visibility_model ? 1.0f - range / p_.zed_max_range : 1.0f;
        t.existence.note_viewpoint(cam.x(), cam.y(), t.c.x(), t.c.y(), p_.cluster_r);
        t.existence.integrate(p_vis, w * p_.llr_miss);
        ++t.zed_misses;
    }
}

void TrunkMap::add_lidar_sweep(const std::vector<Eigen::Vector3f>& sweep, const Eigen::Vector3f& origin,
                               const std::vector<Occluder>& people)
{
    const Eigen::Vector2f o(origin.x(), origin.y());
    struct Acc
    {
        std::vector<Eigen::Vector2f> xy;
        float z_min = 1e9f, z_max = -1e9f;
        void add(const Eigen::Vector3f& q) { xy.emplace_back(q.x(), q.y()); z_min = std::min(z_min, q.z()); z_max = std::max(z_max, q.z()); }
    };
    std::unordered_map<int, Acc> acc;          // trunk id -> its band returns this sweep
    std::vector<Acc> cand_acc(candidates_.size());
    for (const auto& q : sweep)
    {
        if (q.z() < p_.band_z_min or q.z() > p_.band_z_max)
            continue;
        const Eigen::Vector2f xy(q.x(), q.y());
        if (inside(xy, people))
            continue;
        if (Trunk* t = nearest_trunk(xy, p_.assoc_gate_m); t != nullptr)
        {
            acc[t->id].add(q);
            continue;
        }
        for (std::size_t i = 0; i < candidates_.size(); ++i)
            if ((xy - candidates_[i].c).norm() < p_.cluster_r + 0.1f)
            {
                cand_acc[i].add(q);
                break;
            }
    }
    // Returns lie on the near face: the axis is a trunk radius further along the ray.
    const auto axis = [&](std::vector<Eigen::Vector2f>& pts)
    {
        const Eigen::Vector2f m = median_xy(pts);
        const Eigen::Vector2f ray = (m - o).norm() > 1e-3f ? Eigen::Vector2f((m - o).normalized()) : Eigen::Vector2f::Zero();
        return Eigen::Vector2f(m + p_.trunk_radius_m * ray);
    };
    const auto vertical = [&](const Acc& a)
    { return static_cast<int>(a.xy.size()) >= p_.lidar_confirm_pts and a.z_max - a.z_min >= p_.lidar_confirm_dz; };

    for (auto& t : trunks_)
        if (auto it = acc.find(t.id); it != acc.end() and vertical(it->second))
        {
            observe(t, axis(it->second.xy), p_.lidar_sigma);
            ++t.lidar_hits;
        }
    // A candidate is confirmed by ONE sweep that sees a vertical object on it — never by evidence summed over
    // sweeps, which ground noise grazing the bottom of the band eventually supplies anywhere.
    for (std::size_t i = 0; i < candidates_.size(); ++i)
        if (not candidates_[i].confirmed and vertical(cand_acc[i]))
        {
            candidates_[i].confirmed = true;
            candidates_[i].lidar_pos = axis(cand_acc[i].xy);
        }
}

void TrunkMap::refresh_candidates(std::int64_t now_ms)
{
    std::erase_if(grid_, [&](const auto& kv) { return now_ms - kv.second.last_ms > static_cast<std::int64_t>(p_.candidate_ttl_ms); });

    // Disc-kernel mean shift over the unexplained band evidence.
    const auto shift = [&](Eigen::Vector2f c, float& mass, std::int64_t& last)
    {
        for (int it = 0; it < 4; ++it)
        {
            Eigen::Vector2f sum = Eigen::Vector2f::Zero();
            mass = 0.0f;
            last = 0;
            for (int ix = index(c.x() - p_.cluster_r); ix <= index(c.x() + p_.cluster_r); ++ix)
                for (int iy = index(c.y() - p_.cluster_r); iy <= index(c.y() + p_.cluster_r); ++iy)
                {
                    const auto g = grid_.find(key(ix, iy));
                    if (g == grid_.end())
                        continue;
                    const Eigen::Vector2f q(centre_of(ix), centre_of(iy));
                    if ((q - c).norm() > p_.cluster_r)
                        continue;
                    sum += g->second.evidence * q;
                    mass += g->second.evidence;
                    last = std::max(last, g->second.last_ms);
                }
            if (mass <= 0.0f)
                return c;
            c = sum / mass;
        }
        return c;
    };
    const auto clear_of = [&](const Eigen::Vector2f& c, const std::vector<Candidate>& others)
    {
        return std::ranges::none_of(trunks_, [&](const Trunk& t) { return (t.c - c).norm() < p_.min_separation_m; })
               and std::ranges::none_of(others, [&](const Candidate& o) { return (o.c - c).norm() < p_.min_separation_m; });
    };

    std::vector<std::pair<float, std::int64_t>> seeds;
    for (const auto& [k, cell] : grid_)
        seeds.emplace_back(cell.evidence, k);
    std::ranges::sort(seeds, [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<Candidate> fresh;
    for (const auto& [w, k] : seeds)
    {
        const Eigen::Vector2f q = cell_xy(k);
        if (not clear_of(q, fresh))
            continue;
        float mass = 0.0f;
        std::int64_t last = 0;
        const Eigen::Vector2f c = shift(q, mass, last);
        if (mass < p_.birth_mass or not clear_of(c, fresh))
            continue;
        Candidate cand{c, mass};
        cand.last_ms = last;
        // Keep the LiDAR confirmation a candidate has already earned: it is the same place.
        for (const auto& old : candidates_)
            if ((old.c - c).norm() < p_.cluster_r)
            {
                cand.confirmed = old.confirmed;
                cand.lidar_pos = old.lidar_pos;
                break;
            }
        fresh.push_back(cand);
    }
    candidates_ = std::move(fresh);
}

TrunkMap::Changes TrunkMap::update(std::int64_t now_ms)
{
    Changes ch;
    // Verified gone (existence belief): removed.
    for (const auto& t : trunks_)
        if (t.existence.should_remove(p_.removal_prob))
            ch.removed.push_back(t.id);
    std::erase_if(trunks_, [&](const Trunk& t) { return t.existence.should_remove(p_.removal_prob); });

    // No overlap: two trunks that have converged closer than min_separation are one trunk seen twice. The
    // OLDER one (lower id) keeps the space.
    std::ranges::sort(trunks_, [](const Trunk& a, const Trunk& b) { return a.id < b.id; });
    for (std::size_t i = 0; i < trunks_.size(); ++i)
        for (std::size_t j = i + 1; j < trunks_.size(); ++j)
            if ((trunks_[i].c - trunks_[j].c).norm() < p_.min_separation_m
                and std::ranges::find(ch.removed, trunks_[j].id) == ch.removed.end())
                ch.removed.push_back(trunks_[j].id);
    std::erase_if(trunks_, [&](const Trunk& t) { return std::ranges::find(ch.removed, t.id) != ch.removed.end(); });

    refresh_candidates(now_ms);

    // Multimodal birth: a ZED candidate becomes a trunk once the LiDAR has returned band points on it. The
    // LiDAR places it (its centroid on the near face, plus a trunk radius along the ray).
    std::vector<Candidate> waiting;
    for (const auto& c : candidates_)
    {
        if (not c.confirmed and p_.lidar_confirm)
        {
            waiting.push_back(c);
            continue;
        }
        // Ablation (lidar_confirm = false): the ZED cluster centre places it.
        const Eigen::Vector2f pos = c.confirmed ? c.lidar_pos : c.c;
        if (std::ranges::any_of(trunks_, [&](const Trunk& t) { return (t.c - pos).norm() < p_.min_separation_m; }))
            continue;
        Trunk t;
        t.id = next_id_++;
        t.c = pos;
        t.var = p_.lidar_sigma * p_.lidar_sigma;
        trunks_.push_back(t);
        ch.born.push_back(t.id);
        // Its evidence is now explained by the trunk: clear it so it cannot seed a second candidate.
        std::erase_if(grid_, [&](const auto& kv) { return (cell_xy(kv.first) - pos).norm() < p_.min_separation_m; });
    }
    candidates_ = std::move(waiting);
    return ch;
}

bool TrunkMap::self_test(std::string* report)
{
    TrunkParams p;
    p.look_period_ms = 250.0f;   // the test's frames are 250 ms apart: one reference look each
    TrunkMap m(p);
    std::mt19937 rng(5);
    std::normal_distribution<float> n01(0.0f, 1.0f);
    std::uniform_real_distribution<float> u01(0.0f, 1.0f);

    const std::vector<Eigen::Vector2f> truth = {{0.0f, 0.0f}, {0.0f, 1.25f}, {0.0f, 2.5f}, {0.0f, 3.75f}};
    const Eigen::Vector2f zed_only(2.5f, 1.0f);   // a semantic false positive the LiDAR never confirms
    const float heading = 0.5f * static_cast<float>(M_PI);   // camera looking north
    std::int64_t t_ms = 0;

    const auto face_points = [&](const Eigen::Vector2f& c, const Eigen::Vector2f& from, int n, float noise)
    {
        std::vector<Eigen::Vector3f> pts;
        const Eigen::Vector2f ray = (c - from).normalized();
        for (int k = 0; k < n; ++k)
        {
            const Eigen::Vector2f s = c - 0.04f * ray + noise * Eigen::Vector2f(n01(rng), n01(rng));
            pts.emplace_back(s.x(), s.y(), p.band_z_min + (p.band_z_max - p.band_z_min) * u01(rng));
        }
        return pts;
    };

    // Phase 1 — the robot drives up the corridor seeing all four trunks (+ one ZED-only phantom).
    for (int f = 0; f < 25; ++f, t_ms += 250)
    {
        const Eigen::Vector2f cam(1.25f, -3.0f + 0.04f * static_cast<float>(f));
        std::vector<Eigen::Vector3f> zed, lidar;
        for (const auto& c : truth)
        {
            auto z = face_points(c, cam, 15, 0.03f);
            zed.insert(zed.end(), z.begin(), z.end());
            auto l = face_points(c, cam, 6, 0.01f);
            lidar.insert(lidar.end(), l.begin(), l.end());
        }
        auto ph = face_points(zed_only, cam, 15, 0.03f);
        zed.insert(zed.end(), ph.begin(), ph.end());
        m.add_zed_frame(zed, cam, heading, {}, {}, true, t_ms);
        m.add_lidar_sweep(lidar, Eigen::Vector3f(cam.x(), cam.y(), 0.95f));
        m.update(t_ms);
    }
    float worst = 0.0f;
    int matched = 0;
    for (const auto& c : truth)
    {
        float best = 1e9f;
        for (const auto& t : m.trunks())
            best = std::min(best, (t.c - c).norm());
        worst = std::max(worst, best);
        matched += best < 0.05f ? 1 : 0;
    }
    const bool phantom_born = std::ranges::any_of(m.trunks(), [&](const Trunk& t) { return (t.c - zed_only).norm() < 0.3f; });
    const bool phase1 = m.trunks().size() == truth.size() and matched == 4 and not phantom_born;

    // Phase 2 — the robot drives on. Trunk 2 is gone; trunk 3 stops being seen because ANOTHER plant's foliage
    // stands between it and the camera. Trunk 2 must be removed (looked at, unoccluded, absent); trunk 3 must be
    // HELD (occluded: no evidence either way).
    const Eigen::Vector2f hide_dir = Eigen::Vector2f(std::cos(-1.285f), std::sin(-1.285f));   // trunk 3 -> cameras
    const std::vector<Occluder> foliage = {{truth[3] + 0.5f * hide_dir, 0.42f}};
    for (int f = 0; f < 120; ++f, t_ms += 250)
    {
        const Eigen::Vector2f cam(1.25f, -2.0f + 0.02f * static_cast<float>(f));
        std::vector<Eigen::Vector3f> zed;
        for (const auto i : {0, 1})
        {
            auto z = face_points(truth[static_cast<std::size_t>(i)], cam, 15, 0.03f);
            zed.insert(zed.end(), z.begin(), z.end());
        }
        m.add_zed_frame(zed, cam, heading, foliage, {}, true, t_ms);
        m.update(t_ms);
    }
    const auto alive = [&](const Eigen::Vector2f& c)
    { return std::ranges::any_of(m.trunks(), [&](const Trunk& t) { return (t.c - c).norm() < 0.2f; }); };
    Trunk probe;
    probe.c = truth[3];
    const bool occluded_ok = m.occluded(Eigen::Vector2f(1.25f, -1.0f), probe, foliage);
    const bool phase2 = alive(truth[0]) and alive(truth[1]) and not alive(truth[2]) and alive(truth[3]);

    const bool ok = phase1 and phase2 and occluded_ok;
    if (report)
        *report = std::format("TrunkMap::self_test {}: birth {} trunks (worst error {:.3f} m, ZED-only phantom born: {}), "
                              "after verification: gone trunk {}, occluded trunk {}", ok ? "PASS" : "FAIL",
                              phase1 ? 4 : static_cast<int>(m.trunks().size()), worst, phantom_born ? "yes" : "no",
                              alive(truth[2]) ? "KEPT (wrong)" : "removed", alive(truth[3]) ? "held" : "REMOVED (wrong)");
    return ok;
}

}  // namespace rc::trunk
