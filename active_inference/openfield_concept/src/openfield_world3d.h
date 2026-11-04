// openfield_world3d.h — the 3D SEMANTIC MAP of the open field, as openfield_concept shows it: a simple
// representation of what the robot believes is out there, built from the shared graph (main thread only).
//
//   ground  — the field polygon (the `world` node's delimiting polygon), tilled-soil brown
//   robot   — a box the size of the Husky at the localised pose, with a lighter block at its front
//   plants  — plant_concept's `plant` nodes (active_inference/conceptos.md): the TRUNK as a cylinder of its measured
//             radius up to the first branch, the CANOPY as a cone (a tall spindle) from there to the plant's height.
//             A part still unseen is shown in red: a red stub where a canopy's trunk should be, a red ring at the foot
//             of a trunk whose canopy has not been seen.
//   rows    — row_metaconcept's rectangles, dark-green outlines on the ground
//   plots   — plot_metaconcept's rectangles, orange outlines
//   field   — the field node's polygon (the frame and the place), a white outline
//   inspection — the explorer's belief: ground still UNKNOWN tinted blue (stronger = less seen), and the offered
//             standpoint as a cyan ring with a bar toward where it will look
#pragma once

#include "openfield_exploration.h"

#include "../../common/vegetation/plant_discs.h"
#include "../../common/viewers/gl_world3d_viewer.h"

#include <Eigen/Dense>
#include <dsr/api/dsr_api.h>
#include <dsr/api/dsr_inner_eigen_api.h>

#include <cmath>
#include <format>
#include <string>
#include <vector>

namespace rc::openfield
{

inline rc::viewers::World3DScene build_world3d(DSR::DSRGraph &G, DSR::InnerEigenAPI &inner_eigen,
                                               const std::string &world_frame, const Eigen::Vector3d &robot_pose,
                                               bool robot_known, const std::vector<Eigen::Vector2f> &field_polygon,
                                               const OpenFieldExploration *exploration = nullptr)
{
    using V = QVector3D;
    rc::viewers::World3DScene s;
    const V soil(0.55f, 0.43f, 0.31f), bark(0.36f, 0.25f, 0.18f), leaf(0.24f, 0.55f, 0.24f), missing(0.85f, 0.15f, 0.15f);
    const V robot_body(0.95f, 0.75f, 0.10f), robot_front(1.0f, 0.92f, 0.55f);
    const V row_c(0.10f, 0.35f, 0.12f), plot_c(0.95f, 0.55f, 0.10f), field_c(0.97f, 0.97f, 0.97f);

    // Ground.
    std::vector<V> poly;
    for (const auto &p : field_polygon) poly.emplace_back(p.x(), p.y(), 0.0f);
    s.ground(poly, soil);

    // The inspection belief: unknown ground tinted toward blue, and the standpoint on offer.
    if (exploration and exploration->explorer().ready())
    {
        const V unknown(0.20f, 0.35f, 0.85f);
        const float h = 0.45f * exploration->explorer().cell_m();
        for (const auto &[c, u] : exploration->explorer().cells())
        {
            if (u < 0.05f) continue;
            const V rgb = soil * (1.f - 0.8f * u) + unknown * (0.8f * u);
            s.quad(V(c.x() - h, c.y() - h, 0.01f), V(c.x() + h, c.y() - h, 0.01f), V(c.x() + h, c.y() + h, 0.01f),
                   V(c.x() - h, c.y() + h, 0.01f), rgb);
        }
        if (const auto &t = exploration->target(); t.has_value())
        {
            const V cyan(0.10f, 0.85f, 0.95f);
            s.ring(t->pos.x(), t->pos.y(), 0.03f, 0.35f, 0.50f, cyan);
            s.box(t->pos.x() + 0.5f * std::cos(t->yaw), t->pos.y() + 0.5f * std::sin(t->yaw), 0.03f, t->yaw, 1.0f, 0.08f,
                  0.05f, cyan);
        }
    }

    // Plants.
    constexpr float kTrunkTop = 0.6f;   // to the first scaffold branch (the trunk band's top, a little above)
    int complete = 0, no_trunk = 0, no_canopy = 0;
    const auto plants = rc::veg::read_plants(G, inner_eigen, world_frame);
    for (const auto &p : plants)
    {
        const float top = std::max(p.height, kTrunkTop + 0.4f);
        if (p.has_trunk)
            s.frustum(p.origin.x(), p.origin.y(), 0.0f, kTrunkTop, p.trunk_r > 0.f ? p.trunk_r : 0.04f,
                      p.trunk_r > 0.f ? p.trunk_r : 0.04f, bark, 10);
        else
            s.frustum(p.origin.x(), p.origin.y(), 0.0f, kTrunkTop, 0.06f, 0.06f, missing, 8);   // trunk not seen
        if (p.has_canopy and p.canopy_r > 0.f)
            s.frustum(p.canopy_c.x(), p.canopy_c.y(), kTrunkTop, top, p.canopy_r, 0.0f, leaf, 14);
        else
            s.ring(p.origin.x(), p.origin.y(), 0.02f, 0.30f, 0.38f, missing);                     // canopy not seen
        if (p.has_trunk and p.has_canopy) ++complete;
        else if (not p.has_trunk) ++no_trunk;
        else ++no_canopy;
    }

    // Rows, plots, the field: their rectangles on the ground.
    const auto rects = [&](std::string_view prefix, float z, float w, const V &rgb)
    {
        int n = 0;
        for (const auto &node : G.get_nodes_by_type("metaconcept"))
        {
            if (not node.name().starts_with(prefix))
                continue;
            const float len = G.get_attrib_by_name<width_m_att>(node).value_or(0.f);
            const float wid = G.get_attrib_by_name<depth_m_att>(node).value_or(0.f);
            const auto M = inner_eigen.get_transformation_matrix(world_frame, node.name(), 0);
            if (not M.has_value() or not (len > 0.f and wid > 0.f))
                continue;
            const auto &m = M.value();
            s.outline(static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3)),
                      static_cast<float>(std::atan2(m(1, 0), m(0, 0))), len, wid, z, w, rgb);
            ++n;
        }
        return n;
    };
    const int n_rows = rects("row_", 0.03f, 0.06f, row_c);
    const int n_plots = rects("plot_", 0.04f, 0.10f, plot_c);
    // The field: its own polygon (the frame node is the field; there is no separate field rectangle any more).
    for (std::size_t i = 0, n = field_polygon.size(); i < n; ++i)
    {
        const auto &a = field_polygon[i], &b = field_polygon[(i + 1) % n];
        const Eigen::Vector2f m = 0.5f * (a + b), d = b - a;
        s.outline(m.x(), m.y(), std::atan2(d.y(), d.x()), d.norm(), 0.0f, 0.05f, 0.07f, field_c);   // a 0.14 m band
    }

    // The robot (Husky A300: ~0.99 x 0.69 m, ~0.4 m tall), at the localised pose.
    if (robot_known)
    {
        const float x = static_cast<float>(robot_pose.x()), y = static_cast<float>(robot_pose.y());
        const float yaw = static_cast<float>(robot_pose.z());
        s.box(x, y, 0.0f, yaw, 0.99f, 0.69f, 0.38f, robot_body);
        s.box(x + 0.40f * std::cos(yaw), y + 0.40f * std::sin(yaw), 0.38f, yaw, 0.18f, 0.50f, 0.10f, robot_front);
    }

    s.caption = std::format("{} plants ({} complete, {} without trunk, {} without canopy) · {} rows · {} plots · {}",
                            plants.size(), complete, no_trunk, no_canopy, n_rows, n_plots,
                            exploration and exploration->mode() == OpenFieldExploration::Mode::Complete
                                ? "field inspected" : "field being inspected");
    if (exploration)
        s.caption += std::format(" · inspection {} ({:.0f}% of the field still unknown)",
                                 OpenFieldExploration::mode_name(exploration->mode()),
                                 100.f * exploration->explorer().remaining_fraction());
    return s;
}

}  // namespace rc::openfield
