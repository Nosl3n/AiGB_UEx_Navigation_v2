/*
 * common/vegetation/rows.h — ROWS from plants: the metaconcept a set of aligned plants forms. SHARED, header-only.
 * PURE Eigen: no DSR, no Qt (RowTracker::self_test() runs anywhere).
 *
 * A row is not seen; it is INFERRED from its members (trunks, or foliage discs): plants that stand close to each
 * other on a line. Each row is a RECTANGLE on the ground — centre, heading, length along the line, width across
 * it — that grows in length as plants near its line join it. A field holds several rows.
 *
 * ── HOW ─────────────────────────────────────────────────────────────────────────────────────────────────
 *   1. The crop's heading: every plant's two nearest neighbours lie along its own row (planting distance within a
 *      row is shorter than between rows), so the doubled-angle mean of those neighbour directions is the row
 *      heading — read from the data, not assumed.
 *   2. Rows: project every plant on the PERPENDICULAR to that heading; a gap wider than row_gap_m separates
 *      two rows (plants of one row scatter a few cm about its line).
 *   3. Segments: along each row, a gap longer than along_gap_m (several missing plants) splits it in two.
 *      A segment needs min_members plants to be a row.
 *   4. The rectangle: each segment's own principal axis (its line can differ slightly from the global heading),
 *      length = extent of its plants + an end margin, width = the plants' disc + their scatter about the line.
 *   5. IDENTITY comes from the MEMBERS (plant ids are stable): a new segment inherits the id of the previous row
 *      it shares most members with — the OLDER one first — so a row keeps its name while it grows; two rows that
 *      merge keep the older name; a split keeps it on the larger part.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace rc::rows
{

struct RowParams
{
    float neighbour_max_m = 3.0f;   // a plant's neighbours for the heading: within this distance
    float row_gap_m = 1.0f;         // across-row gap that separates two rows
    float along_gap_m = 4.0f;       // along-row gap that splits a row (~3 missing plants at 1.25 m)
    int   min_members = 3;          // plants needed to call it a row
    float half_width_m = 0.6f;      // half the rectangle's width beyond the line (= the member disc radius)
    float end_margin_m = 0.6f;      // rectangle beyond the first and last member (= the member disc radius)
};

struct Member { int id; Eigen::Vector2f p; };

struct Row
{
    int id = 0;
    Eigen::Vector2f c = Eigen::Vector2f::Zero();   // rectangle centre
    float yaw = 0.0f;                              // heading of the row line (rad, from +x, in [0, pi))
    float length = 0.0f;                           // along the line (m)
    float width = 0.0f;                            // across it (m)
    std::vector<int> members;                      // member ids, sorted
};

class RowTracker
{
public:
    explicit RowTracker(RowParams p = {}) : p_(p) {}
    void set_params(const RowParams& p) { p_ = p; }
    [[nodiscard]] const RowParams& params() const { return p_; }
    [[nodiscard]] const std::vector<Row>& rows() const { return rows_; }

    struct Changes { std::vector<int> born, removed; };

    // Re-infer the rows from the current members. Cheap for a field's worth of plants (O(n²) neighbours).
    Changes update(const std::vector<Member>& members)
    {
        std::vector<Row> fresh = infer(members);

        // Identity by shared members, older rows first.
        std::vector<int> old_ids;
        for (const auto& r : rows_) old_ids.push_back(r.id);
        std::ranges::sort(old_ids);
        std::vector<bool> taken(fresh.size(), false);
        std::set<int> kept;
        for (const int oid : old_ids)
        {
            const auto& old = *std::ranges::find(rows_, oid, &Row::id);
            int best = -1;
            std::size_t best_shared = 0;
            for (std::size_t i = 0; i < fresh.size(); ++i)
            {
                if (taken[i]) continue;
                std::vector<int> shared;
                std::ranges::set_intersection(old.members, fresh[i].members, std::back_inserter(shared));
                if (shared.size() > best_shared) { best_shared = shared.size(); best = static_cast<int>(i); }
            }
            if (best >= 0)
            {
                fresh[static_cast<std::size_t>(best)].id = oid;
                taken[static_cast<std::size_t>(best)] = true;
                kept.insert(oid);
            }
        }
        Changes ch;
        for (std::size_t i = 0; i < fresh.size(); ++i)
            if (not taken[i])
            {
                fresh[i].id = next_id_++;
                ch.born.push_back(fresh[i].id);
            }
        for (const int oid : old_ids)
            if (not kept.contains(oid)) ch.removed.push_back(oid);
        rows_ = std::move(fresh);
        return ch;
    }

    // Three rows (one with a missing plant), a far isolated plant, a rotated field; then one row grows and keeps
    // its id. Returns true on PASS and fills `report`.
    static bool self_test(std::string* report = nullptr)
    {
        std::mt19937 rng(9);
        std::normal_distribution<float> n01(0.0f, 1.0f);
        const float th = 1.75f;                              // ~100 deg: rows run roughly north
        const Eigen::Vector2f d(std::cos(th), std::sin(th)), n(-std::sin(th), std::cos(th));
        std::vector<Member> ms;
        int id = 0;
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 8; ++k)
            {
                if (r == 1 and k == 4) continue;                // a missing plant: the row must not split
                ms.push_back({id++, (2.5f * static_cast<float>(r)) * n + (1.25f * static_cast<float>(k)) * d
                                     + 0.03f * Eigen::Vector2f(n01(rng), n01(rng))});
            }
        ms.push_back({id++, Eigen::Vector2f(30.0f, 30.0f)});   // isolated: no row
        RowTracker t;
        t.update(ms);
        const auto rows0 = t.rows();
        bool ok = rows0.size() == 3;
        float worst_yaw = 0.0f;
        for (const auto& r : rows0)
        {
            const float dy = std::abs(std::remainder(r.yaw - th, static_cast<float>(M_PI)));
            worst_yaw = std::max(worst_yaw, dy);
            ok = ok and (r.members.size() == 8 or r.members.size() == 7) and r.length > 7.0f * 1.25f;
        }
        ok = ok and worst_yaw < 0.035f;   // 2 degrees
        // Grow row of member 0 by one plant at its end: same id, longer rectangle.
        const auto& r0 = *std::ranges::find_if(rows0, [](const Row& r) { return std::ranges::find(r.members, 0) != r.members.end(); });
        ms.push_back({id++, 8.0f * 1.25f * d});
        const auto ch = t.update(ms);
        const auto& r1 = *std::ranges::find(t.rows(), r0.id, &Row::id);
        const bool grew = ch.born.empty() and ch.removed.empty() and r1.length > r0.length + 1.0f;
        ok = ok and grew;
        if (report)
            *report = std::format("RowTracker::self_test {}: {} rows from 3 (worst heading error {:.2f} deg), row {} "
                                  "grew {:.2f} -> {:.2f} m keeping its id", ok ? "PASS" : "FAIL", rows0.size(),
                                  worst_yaw * 57.2958f, r0.id, r0.length, r1.length);
        return ok;
    }

private:
    RowParams p_;
    std::vector<Row> rows_;
    int next_id_ = 0;

    [[nodiscard]] std::vector<Row> infer(const std::vector<Member>& ms) const
    {
        std::vector<Row> out;
        if (static_cast<int>(ms.size()) < p_.min_members)
            return out;

        // 1. Heading from each plant's two nearest neighbours (doubled angles: a line has no direction).
        Eigen::Vector2f acc = Eigen::Vector2f::Zero();
        for (std::size_t i = 0; i < ms.size(); ++i)
        {
            std::vector<std::pair<float, std::size_t>> nb;
            for (std::size_t j = 0; j < ms.size(); ++j)
                if (j != i)
                    if (const float dd = (ms[j].p - ms[i].p).norm(); dd > 1e-3f and dd < p_.neighbour_max_m)
                        nb.emplace_back(dd, j);
            std::ranges::sort(nb);
            for (std::size_t k = 0; k < std::min<std::size_t>(2, nb.size()); ++k)
            {
                const Eigen::Vector2f v = ms[nb[k].second].p - ms[i].p;
                const float a = 2.0f * std::atan2(v.y(), v.x());
                acc += Eigen::Vector2f(std::cos(a), std::sin(a));
            }
        }
        if (acc.norm() < 1e-6f)
            return out;
        const float heading = 0.5f * std::atan2(acc.y(), acc.x());
        const Eigen::Vector2f d(std::cos(heading), std::sin(heading)), n(-std::sin(heading), std::cos(heading));

        // 2. Rows: clusters of the across-row coordinate.
        std::vector<std::size_t> idx(ms.size());
        for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
        std::ranges::sort(idx, [&](std::size_t a, std::size_t b) { return ms[a].p.dot(n) < ms[b].p.dot(n); });
        std::vector<std::vector<std::size_t>> lines(1);
        for (std::size_t k = 0; k < idx.size(); ++k)
        {
            if (k > 0 and ms[idx[k]].p.dot(n) - ms[idx[k - 1]].p.dot(n) > p_.row_gap_m)
                lines.emplace_back();
            lines.back().push_back(idx[k]);
        }

        // 3. Segments along each row, 4. their rectangles.
        for (auto& line : lines)
        {
            std::ranges::sort(line, [&](std::size_t a, std::size_t b) { return ms[a].p.dot(d) < ms[b].p.dot(d); });
            std::vector<std::vector<std::size_t>> segs(1);
            for (std::size_t k = 0; k < line.size(); ++k)
            {
                if (k > 0 and ms[line[k]].p.dot(d) - ms[line[k - 1]].p.dot(d) > p_.along_gap_m)
                    segs.emplace_back();
                segs.back().push_back(line[k]);
            }
            for (const auto& seg : segs)
                if (static_cast<int>(seg.size()) >= p_.min_members)
                    out.push_back(rectangle(ms, seg, d));
        }
        return out;
    }

    [[nodiscard]] Row rectangle(const std::vector<Member>& ms, const std::vector<std::size_t>& seg,
                                const Eigen::Vector2f& d_global) const
    {
        Eigen::Vector2f mean = Eigen::Vector2f::Zero();
        for (const auto i : seg) mean += ms[i].p;
        mean /= static_cast<float>(seg.size());
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto i : seg) { const Eigen::Vector2f q = ms[i].p - mean; cov += q * q.transpose(); }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
        Eigen::Vector2f d = es.eigenvectors().col(1);           // largest eigenvalue: the row's own line
        if (d.dot(d_global) < 0.0f) d = -d;
        const Eigen::Vector2f n(-d.y(), d.x());
        float tmin = 1e9f, tmax = -1e9f, smax = 0.0f;
        Row r;
        for (const auto i : seg)
        {
            const Eigen::Vector2f q = ms[i].p - mean;
            tmin = std::min(tmin, q.dot(d));
            tmax = std::max(tmax, q.dot(d));
            smax = std::max(smax, std::abs(q.dot(n)));
            r.members.push_back(ms[i].id);
        }
        std::ranges::sort(r.members);
        r.c = mean + 0.5f * (tmin + tmax) * d;
        float yaw = std::atan2(d.y(), d.x());
        if (yaw < 0.0f) yaw += static_cast<float>(M_PI);
        if (yaw >= static_cast<float>(M_PI)) yaw -= static_cast<float>(M_PI);
        r.yaw = yaw;
        r.length = (tmax - tmin) + 2.0f * p_.end_margin_m;
        r.width = 2.0f * (p_.half_width_m + smax);
        return r;
    }
};

}  // namespace rc::rows
