#pragma once
// common/vegetation/plots.h — PLOTS (parcelas) from ROWS, and the FIELD from plots. Pure, header-only, no DSR.
//
// Metaconcepts all the way up (active_inference/conceptos.md): field -> plot -> row -> plant.
//   * A PLOT is a set of rows planted together: PARALLEL, SIDE BY SIDE (their extents overlap along the row
//     direction) and at the PLANTING SPACING from each other. The spacing is not configured: it is read from the
//     rows themselves, as the median distance from each row to its nearest parallel neighbour. Two neighbouring
//     rows further apart than that spacing allows (a headland, a track, the gap between two plantations) are in
//     different plots.
//   * The FIELD is the set of all plots.
// Both are oriented rectangles (centre, heading = the rows' direction, length along it, width across it), so a
// consumer draws and reasons about them exactly like a row.
//
// Identity: a plot keeps its id while it keeps (most of) its rows — it grows as rows join, two plots that merge
// keep the older id, and a plot is removed only when none of its rows is left in any plot.

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <format>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <vector>

namespace rc::plots
{

struct RowGeom
{
    int id = -1;
    Eigen::Vector2f c = Eigen::Vector2f::Zero();   // rectangle centre, world frame (m)
    float yaw = 0.0f;                              // row direction (rad)
    float length = 0.0f;                           // along the row (m)
    float width = 0.0f;                            // across the row (m)
};

struct Rect
{
    Eigen::Vector2f c = Eigen::Vector2f::Zero();
    float yaw = 0.0f, length = 0.0f, width = 0.0f;
};

struct Plot
{
    int id = -1;
    std::vector<int> rows;   // member row ids, sorted
    Rect rect;
};

struct PlotParams
{
    // Rows of one plot share a direction up to planting error; this is how far that error may go (rad, ~10°).
    float parallel_tol_rad = 0.17f;
    // A neighbour at up to this many times the planting spacing is the next row of the same plot. 1.5 accepts a
    // missing tree line's worth of irregularity and rejects a headland or a second plantation (the field here:
    // rows 2.5 m apart, plantations 5 m apart = 2 spacings).
    float spacing_factor = 1.5f;
};

// The rectangle that holds a set of row rectangles, oriented along their (circular) mean direction.
inline Rect enclosing(const std::vector<const RowGeom*>& rows)
{
    Rect r;
    if (rows.empty())
        return r;
    // Mean of a direction defined modulo pi: average the doubled angle.
    float s = 0.0f, c = 0.0f;
    for (const auto* g : rows) { s += std::sin(2.0f * g->yaw); c += std::cos(2.0f * g->yaw); }
    r.yaw = 0.5f * std::atan2(s, c);
    const Eigen::Vector2f a(std::cos(r.yaw), std::sin(r.yaw)), x(-a.y(), a.x());
    float a0 = 1e9f, a1 = -1e9f, x0 = 1e9f, x1 = -1e9f;
    for (const auto* g : rows)
    {
        const float ga = a.dot(g->c), gx = x.dot(g->c);
        a0 = std::min(a0, ga - 0.5f * g->length); a1 = std::max(a1, ga + 0.5f * g->length);
        x0 = std::min(x0, gx - 0.5f * g->width);  x1 = std::max(x1, gx + 0.5f * g->width);
    }
    r.c = 0.5f * (a0 + a1) * a + 0.5f * (x0 + x1) * x;
    r.length = a1 - a0;
    r.width = x1 - x0;
    return r;
}

class PlotTracker
{
public:
    struct Changes { std::vector<int> created, removed; };

    void set_params(const PlotParams& p) { p_ = p; }

    Changes update(const std::vector<RowGeom>& rows)
    {
        Changes ch;
        const auto n = rows.size();
        const auto angle_between = [](float a, float b)
        {
            float d = std::fmod(std::abs(a - b), static_cast<float>(M_PI));
            return std::min(d, static_cast<float>(M_PI) - d);
        };
        // Across distance and along overlap between two parallel rows (in the frame of the first).
        const auto relation = [&](const RowGeom& i, const RowGeom& j, float& across, float& overlap)
        {
            const Eigen::Vector2f a(std::cos(i.yaw), std::sin(i.yaw)), x(-a.y(), a.x());
            const Eigen::Vector2f d = j.c - i.c;
            across = std::abs(x.dot(d));
            const float along = a.dot(d);
            overlap = std::min(0.5f * i.length, along + 0.5f * j.length) - std::max(-0.5f * i.length, along - 0.5f * j.length);
        };
        // The planting spacing: median distance from each row to its nearest parallel, side-by-side neighbour.
        std::vector<int> nearest(n, -1);
        std::vector<float> nearest_d(n, 1e9f);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
            {
                if (i == j or angle_between(rows[i].yaw, rows[j].yaw) > p_.parallel_tol_rad)
                    continue;
                float across, overlap;
                relation(rows[i], rows[j], across, overlap);
                if (overlap > 0.0f and across < nearest_d[i])
                {
                    nearest_d[i] = across;
                    nearest[i] = static_cast<int>(j);
                }
            }
        std::vector<float> ds;
        for (std::size_t i = 0; i < n; ++i)
            if (nearest[i] >= 0) ds.push_back(nearest_d[i]);
        spacing_ = 0.0f;
        if (not ds.empty())
        {
            std::ranges::nth_element(ds, ds.begin() + static_cast<std::ptrdiff_t>(ds.size() / 2));
            spacing_ = ds[ds.size() / 2];
        }

        // Union-find over "next row of the same plot": parallel, side by side, within the spacing allowance.
        std::vector<std::size_t> parent(n);
        std::iota(parent.begin(), parent.end(), 0);
        const auto find = [&](std::size_t i) { while (parent[i] != i) i = parent[i] = parent[parent[i]]; return i; };
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j)
            {
                if (angle_between(rows[i].yaw, rows[j].yaw) > p_.parallel_tol_rad)
                    continue;
                float across, overlap;
                relation(rows[i], rows[j], across, overlap);
                if (overlap > 0.0f and spacing_ > 0.0f and across <= p_.spacing_factor * spacing_)
                    parent[find(i)] = find(j);
            }
        std::map<std::size_t, std::vector<const RowGeom*>> groups;
        for (std::size_t i = 0; i < n; ++i)
            groups[find(i)].push_back(&rows[i]);

        // Identity: each new group takes the id of the existing plot it shares most rows with (oldest on a tie);
        // a group sharing none is a new plot; a plot that no group took is gone.
        std::map<int, Plot> next;
        std::set<int> taken;
        std::vector<std::vector<const RowGeom*>> unassigned;
        for (auto& [root, members] : groups)
        {
            std::ranges::sort(members, {}, [](const RowGeom* g) { return g->id; });
            int best = -1, best_shared = 0;
            for (const auto& [pid, pl] : plots_)
            {
                if (taken.contains(pid)) continue;
                int shared = 0;
                for (const auto* g : members) shared += std::ranges::binary_search(pl.rows, g->id) ? 1 : 0;
                if (shared > best_shared) { best_shared = shared; best = pid; }
            }
            if (best < 0) { unassigned.push_back(members); continue; }
            taken.insert(best);
            next[best] = make_plot(best, members);
        }
        for (const auto& members : unassigned)
        {
            const int id = next_id_++;
            next[id] = make_plot(id, members);
            ch.created.push_back(id);
        }
        for (const auto& [pid, pl] : plots_)
            if (not next.contains(pid)) ch.removed.push_back(pid);
        plots_ = std::move(next);
        return ch;
    }

    [[nodiscard]] const std::map<int, Plot>& plots() const { return plots_; }
    [[nodiscard]] float spacing() const { return spacing_; }

    // The field: the rectangle holding every plot.
    [[nodiscard]] Rect field() const
    {
        std::vector<RowGeom> as_rows;
        for (const auto& [id, pl] : plots_)
            as_rows.push_back({id, pl.rect.c, pl.rect.yaw, pl.rect.length, pl.rect.width});
        std::vector<const RowGeom*> ptrs;
        for (const auto& g : as_rows) ptrs.push_back(&g);
        return enclosing(ptrs);
    }

    // Two plantations of 4 rows 2.5 m apart with a 5 m gap -> 2 plots; a row joining one keeps its id.
    static bool self_test(std::string* report)
    {
        bool ok = true;
        std::string log;
        const auto fail = [&](const std::string& m) { ok = false; log += "  FAIL " + m + "\n"; };
        const float yaw = static_cast<float>(M_PI) / 2.0f;
        std::vector<RowGeom> rows;
        for (int i = 0; i < 4; ++i) rows.push_back({i, {-3.75f + 2.5f * i, 0.0f}, yaw, 13.5f, 1.1f});
        for (int i = 0; i < 3; ++i) rows.push_back({4 + i, {8.75f + 2.5f * i, 0.0f}, yaw, 13.5f, 1.1f});
        PlotTracker t;
        t.update(rows);
        if (t.plots().size() != 2) fail(std::format("expected 2 plots, got {}", t.plots().size()));
        if (std::abs(t.spacing() - 2.5f) > 0.01f) fail(std::format("spacing {:.2f} != 2.5", t.spacing()));
        const int east = std::ranges::find_if(t.plots(), [](auto& kv) { return kv.second.rows.size() == 3; })->first;
        rows.push_back({7, {16.25f, 0.0f}, yaw, 13.5f, 1.1f});   // the eastern plantation's 4th row is found
        const auto ch = t.update(rows);
        if (t.plots().size() != 2 or not ch.created.empty()) fail("a joining row must not create a plot");
        if (not t.plots().contains(east) or t.plots().at(east).rows.size() != 4) fail("the eastern plot keeps its id and grows");
        if (const auto f = t.field(); std::abs(f.width - (16.25f + 3.75f + 1.1f)) > 0.05f)
            fail(std::format("field width {:.2f}", t.field().width));
        if (report) *report = std::format("PlotTracker::self_test {}\n", ok ? "PASS" : "FAIL") + log;
        return ok;
    }

private:
    static Plot make_plot(int id, const std::vector<const RowGeom*>& members)
    {
        Plot pl;
        pl.id = id;
        for (const auto* g : members) pl.rows.push_back(g->id);
        pl.rect = enclosing(members);
        return pl;
    }

    PlotParams p_;
    std::map<int, Plot> plots_;
    int next_id_ = 0;
    float spacing_ = 0.0f;
};

}  // namespace rc::plots
