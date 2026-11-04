// field_explorer.cpp — see field_explorer.h for the model.
#include "field_explorer.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

namespace rc::openfield
{

bool point_in_polygon(const Eigen::Vector2f &p, const std::vector<Eigen::Vector2f> &poly)
{
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
    {
        const auto &a = poly[i], &b = poly[j];
        if (((a.y() > p.y()) != (b.y() > p.y()))
            and (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x()))
            in = not in;
    }
    return in;
}

float distance_to_polygon_edge(const Eigen::Vector2f &p, const std::vector<Eigen::Vector2f> &poly)
{
    float best = std::numeric_limits<float>::max();
    for (std::size_t i = 0, n = poly.size(); i < n; ++i)
    {
        const Eigen::Vector2f a = poly[i], b = poly[(i + 1) % n], ab = b - a;
        const float t = std::clamp((p - a).dot(ab) / std::max(ab.squaredNorm(), 1e-9f), 0.f, 1.f);
        best = std::min(best, (a + t * ab - p).norm());
    }
    return best;
}

namespace
{
float binary_entropy(float p)
{
    p = std::clamp(p, 1e-6f, 1.f - 1e-6f);
    return -(p * std::log(p) + (1.f - p) * std::log(1.f - p));
}

// Point in the row's frame (x along the row, y across it).
Eigen::Vector2f to_rect(const RowRect &r, const Eigen::Vector2f &p)
{
    const float c = std::cos(r.yaw), s = std::sin(r.yaw);
    const Eigen::Vector2f d = p - r.c;
    return {c * d.x() + s * d.y(), -s * d.x() + c * d.y()};
}

bool inside_rect(const RowRect &r, const Eigen::Vector2f &p, float grow)
{
    const auto q = to_rect(r, p);
    return std::abs(q.x()) <= 0.5f * r.length + grow and std::abs(q.y()) <= 0.5f * r.width + grow;
}

// Does the segment a→b cross the rectangle? (Liang–Barsky in the rectangle's frame.)
bool segment_crosses(const RowRect &r, const Eigen::Vector2f &a, const Eigen::Vector2f &b)
{
    const auto p = to_rect(r, a), q = to_rect(r, b);
    const Eigen::Vector2f d = q - p;
    float t0 = 0.f, t1 = 1.f;
    const float hx = 0.5f * r.length, hy = 0.5f * r.width;
    const auto clip = [&](float den, float num)
    {
        if (std::abs(den) < 1e-9f) return num >= 0.f;
        const float t = num / den;
        if (den < 0.f) { if (t > t1) return false; t0 = std::max(t0, t); }
        else           { if (t < t0) return false; t1 = std::min(t1, t); }
        return true;
    };
    return clip(-d.x(), p.x() + hx) and clip(d.x(), hx - p.x()) and clip(-d.y(), p.y() + hy)
           and clip(d.y(), hy - p.y()) and t0 <= t1;
}
}  // namespace

void FieldExplorer::set_polygon(const std::vector<Eigen::Vector2f> &polygon)
{
    if (polygon.size() < 3 or polygon == polygon_)
        return;
    polygon_ = polygon;
    Eigen::Vector2f lo = polygon.front(), hi = polygon.front();
    for (const auto &v : polygon) { lo = lo.cwiseMin(v); hi = hi.cwiseMax(v); }
    nx_ = std::max(1, static_cast<int>(std::ceil((hi.x() - lo.x()) / p_.cell_m)));
    ny_ = std::max(1, static_cast<int>(std::ceil((hi.y() - lo.y()) / p_.cell_m)));
    origin_ = lo + Eigen::Vector2f::Constant(0.5f * p_.cell_m);
    u_.assign(static_cast<std::size_t>(nx_ * ny_), -1.f);
    h0_ = binary_entropy(p_.prior_plant_p);
    h_init_ = 0.f;
    for (int iy = 0; iy < ny_; ++iy)
        for (int ix = 0; ix < nx_; ++ix)
            if (point_in_polygon(centre(ix, iy), polygon_))
            {
                u_[static_cast<std::size_t>(iy * nx_ + ix)] = 1.f;
                h_init_ += h0_;
            }
}

void FieldExplorer::set_open_plants(const std::vector<OpenPlant> &open)
{
    std::vector<PlantQuestion> next;
    for (const auto &o : open)
    {
        PlantQuestion q{o, p_.part_prior, 0.f};
        for (const auto &old : questions_)
            if (old.plant.name == o.name and old.plant.missing_trunk == o.missing_trunk)
            {
                q.b = old.b;
                q.looked_s = old.looked_s;
            }
        next.push_back(q);
    }
    questions_ = std::move(next);
}

float FieldExplorer::concept_entropy() const
{
    float h = 0.f;
    for (const auto &q : questions_) h += binary_entropy(q.b);
    return h;
}

void FieldExplorer::set_obstacles(std::vector<RowRect> rows, std::vector<Disc> plants)
{
    rows_ = std::move(rows);
    plants_ = std::move(plants);
}

Eigen::Vector2f FieldExplorer::centre(int ix, int iy) const
{
    return origin_ + Eigen::Vector2f(static_cast<float>(ix), static_cast<float>(iy)) * p_.cell_m;
}

template <typename F> void FieldExplorer::for_cells_near(const Eigen::Vector2f &v, F &&f) const
{
    const float reach = 2.5f * p_.sensor_range_m;   // p_det(reach) = exp(-6.25) ≈ 0.002
    const int r = static_cast<int>(std::ceil(reach / p_.cell_m));
    const Eigen::Vector2f g = (v - origin_) / p_.cell_m;
    const int cx = static_cast<int>(std::lround(g.x())), cy = static_cast<int>(std::lround(g.y()));
    for (int iy = std::max(0, cy - r); iy <= std::min(ny_ - 1, cy + r); ++iy)
        for (int ix = std::max(0, cx - r); ix <= std::min(nx_ - 1, cx + r); ++ix)
        {
            const auto k = static_cast<std::size_t>(iy * nx_ + ix);
            if (u_[k] < 0.f) continue;
            f(k, centre(ix, iy));
        }
}

float FieldExplorer::p_det(const Eigen::Vector2f &from, const Eigen::Vector2f &to) const
{
    const float d = (to - from).norm() / p_.sensor_range_m;
    float p = std::exp(-d * d);
    for (const auto &r : rows_)
        if (not inside_rect(r, to, 0.f) and not inside_rect(r, from, 0.f) and segment_crosses(r, from, to))
            p *= p_.canopy_transmittance;
    return p;
}

void FieldExplorer::observe(const Eigen::Vector2f &pos, float dt)
{
    if (not ready() or dt <= 0.f) return;
    for_cells_near(pos, [&](std::size_t k, const Eigen::Vector2f &c)
                   { u_[k] *= std::exp(-p_.detect_rate * p_det(pos, c) * dt); });
    // Negative evidence about each missing part: a look that would have found it with probability 1 - e^{-q} did
    // not (the plant is still listed as incomplete), so b ← b·e^{-q} / (b·e^{-q} + 1 − b).
    for (auto &q : questions_)
    {
        const float rate = p_.detect_rate * part_det(pos, q.plant.c) * dt;
        const float miss = std::exp(-rate);
        q.b = q.b * miss / (q.b * miss + 1.f - q.b);
        q.looked_s += rate;
    }
}

float FieldExplorer::part_det(const Eigen::Vector2f &from, const Eigen::Vector2f &to) const
{
    const float d = (to - from).norm() / p_.part_range_m;
    float p = std::exp(-d * d);
    for (const auto &r : rows_)
        if (not inside_rect(r, to, 0.f) and not inside_rect(r, from, 0.f) and segment_crosses(r, from, to))
            p *= p_.canopy_transmittance;
    return p;
}

float FieldExplorer::gain_at(const Eigen::Vector2f &v) const
{
    float g = 0.f;
    for_cells_near(v, [&](std::size_t k, const Eigen::Vector2f &c)
                   { g += u_[k] * h0_ * (1.f - std::exp(-p_.detect_rate * p_det(v, c) * p_.dwell_s)); });
    // What is INSIDE: the expected entropy removed from each open plant question by a dwell here. Exact for the
    // binary belief: if the look finds the part (prob b·(1−m)) the question closes (H = 0); otherwise b goes to b'.
    for (const auto &q : questions_)
    {
        const float m = std::exp(-p_.detect_rate * part_det(v, q.plant.c) * p_.dwell_s);
        const float p_miss = q.b * m + 1.f - q.b;
        const float b_after = q.b * m / p_miss;
        const float h_after = p_miss * binary_entropy(b_after);
        g += p_.concept_weight * std::max(0.f, binary_entropy(q.b) - h_after);
    }
    return g;
}

float FieldExplorer::facing(const Eigen::Vector2f &v) const
{
    Eigen::Vector2f acc = Eigen::Vector2f::Zero();
    for_cells_near(v, [&](std::size_t k, const Eigen::Vector2f &c)
                   {
                       const Eigen::Vector2f d = c - v;
                       const float n = d.norm();
                       if (n > 1e-3f) acc += u_[k] * p_det(v, c) * d / n;
                   });
    for (const auto &q : questions_)
    {
        const Eigen::Vector2f d = q.plant.c - v;
        const float n = d.norm();
        if (n > 1e-3f) acc += p_.concept_weight * binary_entropy(q.b) * part_det(v, q.plant.c) * d / n;
    }
    return acc.squaredNorm() > 1e-12f ? std::atan2(acc.y(), acc.x()) : 0.f;
}

bool FieldExplorer::standpoint_ok(const Eigen::Vector2f &v) const
{
    if (not point_in_polygon(v, polygon_) or distance_to_polygon_edge(v, polygon_) < p_.boundary_clearance)
        return false;
    for (const auto &r : rows_)
        if (inside_rect(r, v, p_.row_clearance)) return false;
    for (const auto &d : plants_)
        if ((v - d.c).norm() < d.r + p_.plant_clearance) return false;
    for (const auto &q : questions_)
        if ((v - q.plant.c).norm() < p_.open_plant_keepout_m) return false;
    return true;
}

std::optional<ExplorerTarget> FieldExplorer::select(const Eigen::Vector2f &robot, double now_s) const
{
    if (not ready()) return std::nullopt;
    // MARGINAL value: a standpoint is worth what it adds over the null policy (stay and keep looking
    // from here). Scored on the absolute ΔH, the robot's own cell always kept a crumb of value at zero
    // travel, so the inspection could never end. Measured against staying put it ends by construction.
    const float here = gain_at(robot);
    std::optional<ExplorerTarget> best;
    for (int iy = 0; iy < ny_; ++iy)
        for (int ix = 0; ix < nx_; ++ix)
        {
            if (u_[static_cast<std::size_t>(iy * nx_ + ix)] < 0.f) continue;
            const Eigen::Vector2f v = centre(ix, iy);
            if (not standpoint_ok(v)) continue;
            float g = gain_at(v);
            // A standpoint the consumer could not reach is suppressed, and the suppression decays.
            for (const auto &f : failed_)
                if ((f.pos - v).norm() < p_.failed_radius_m)
                    g *= 1.f - static_cast<float>(std::exp(-(now_s - f.t) / p_.failed_suppress_s));
            const float marginal = std::max(0.f, g - here);
            const float score = marginal - p_.travel_cost * (v - robot).norm();
            if (not best or score > best->score)
                best = ExplorerTarget{v, 0.f, marginal, score};
        }
    // EFE sign: when no standpoint is worth its travel, there is nothing left to inspect.
    if (not best or best->score <= 0.f) return std::nullopt;
    best->yaw = facing(best->pos);
    return best;
}

void FieldExplorer::mark_failed(const Eigen::Vector2f &pos, double now_s)
{
    std::erase_if(failed_, [&](const Failed &f) { return now_s - f.t > 5.0 * p_.failed_suppress_s; });
    failed_.push_back({pos, now_s});
}

float FieldExplorer::entropy() const
{
    float h = 0.f;
    for (const float u : u_)
        if (u >= 0.f) h += u * h0_;
    return h;
}

std::vector<std::pair<Eigen::Vector2f, float>> FieldExplorer::cells() const
{
    std::vector<std::pair<Eigen::Vector2f, float>> out;
    for (int iy = 0; iy < ny_; ++iy)
        for (int ix = 0; ix < nx_; ++ix)
            if (const float u = u_[static_cast<std::size_t>(iy * nx_ + ix)]; u >= 0.f)
                out.emplace_back(centre(ix, iy), u);
    return out;
}

// Synthetic field: a 20 x 12 m polygon with two rows. A robot that keeps going to the selected
// standpoint must (1) never pick a standpoint inside a row, (2) reduce the entropy monotonically,
// (3) cover both sides of the rows, and (4) end with select() == nullopt (inspection complete).
bool FieldExplorer::self_test(std::string *report)
{
    FieldExplorer ex;
    ex.set_polygon({{-10.f, -6.f}, {10.f, -6.f}, {10.f, 6.f}, {-10.f, 6.f}});
    ex.set_obstacles({RowRect{{-2.f, 0.f}, 1.5708f, 8.f, 0.8f}, RowRect{{2.f, 0.f}, 1.5708f, 8.f, 0.8f}}, {});
    Eigen::Vector2f robot(0.f, -5.f);
    std::string log;
    bool ok = true;
    float h_prev = ex.entropy();
    int legs = 0;
    bool saw_west = false, saw_east = false;
    for (; legs < 200; ++legs)
    {
        const auto t = ex.select(robot, legs * 10.0);
        if (not t) break;
        for (const auto &r : ex.rows_)
            if (inside_rect(r, t->pos, 0.f)) { ok = false; log += std::format("leg {} inside a row\n", legs); }
        // drive there in 0.5 m steps, looking as it goes, then dwell
        while ((t->pos - robot).norm() > 0.25f)
        {
            robot += (t->pos - robot).normalized() * std::min(0.5f, (t->pos - robot).norm());
            ex.observe(robot, 0.5f);
        }
        ex.observe(robot, ex.params().dwell_s);
        saw_west |= robot.x() < -3.f;
        saw_east |= robot.x() > 3.f;
        const float h = ex.entropy();
        if (h > h_prev + 1e-3f) { ok = false; log += std::format("entropy rose at leg {}\n", legs); }
        h_prev = h;
    }
    const bool finished = legs < 200;
    ok = ok and finished and saw_west and saw_east;

    // A canopy without a trunk in a corner the coverage already explained: the inspection must still go and look
    // at it, and come back with the question answered (b low) — not complete while it is open.
    FieldExplorer ex2;
    ex2.set_polygon({{-10.f, -6.f}, {10.f, -6.f}, {10.f, 6.f}, {-10.f, 6.f}});
    Eigen::Vector2f r2(0.f, 0.f);
    for (int k = 0; k < 400; ++k) ex2.observe(Eigen::Vector2f(-9.f + 18.f * (k % 20) / 19.f, -5.f + 10.f * (k / 20) / 19.f), 1.f);
    ex2.set_open_plants({OpenPlant{"plant_7", {8.5f, 4.5f}, true}});
    float min_d = 1e9f;
    int legs2 = 0;
    for (; legs2 < 50; ++legs2)
    {
        const auto t = ex2.select(r2, legs2 * 10.0);
        if (not t) break;
        while ((t->pos - r2).norm() > 0.25f)
        {
            r2 += (t->pos - r2).normalized() * std::min(0.5f, (t->pos - r2).norm());
            ex2.observe(r2, 0.5f);
        }
        ex2.observe(r2, ex2.params().dwell_s);
        min_d = std::min(min_d, (r2 - Eigen::Vector2f(8.5f, 4.5f)).norm());
    }
    const float b = ex2.questions().front().b;
    const bool looked = min_d < 3.f and min_d >= ex2.params().open_plant_keepout_m - 0.3f;
    const bool answered = b < 0.2f, done = legs2 < 50;
    ok = ok and looked and answered and done;
    log += std::format("open plant: closest approach {:.2f} m, b {:.3f} (prior {:.2f}), legs {}, complete {}\n",
                       min_d, b, ex2.params().part_prior, legs2, done);
    log += std::format("legs={} remaining={:.1f}% finished={} west={} east={}\n", legs,
                       100.f * ex.remaining_fraction(), finished, saw_west, saw_east);
    if (report) *report = log;
    return ok;
}

}  // namespace rc::openfield
