/*
 * common/vegetation/plant_discs.h — the plants plant_concept publishes, read as discs. SHARED, header-only.
 *
 * A plant is ONE `plant` node plant_<id> (plant_concept); its parts are ATTRIBUTES of it (active_inference/
 * conceptos.md), never nodes of their own. The world->plant RT is where it stands (its trunk axis, or its canopy
 * centre while the trunk is unseen). Two discs come out of one plant:
 *   * the TRUNK disc  — centre = the plant's origin, r = plant_keepout_radius_m: the space the trunk forbids,
 *                       an obstacle (only if plant_has_trunk);
 *   * the CANOPY disc — centre = origin + plant_canopy_offset, r = plant_canopy_radius_m: the foliage seen from
 *                       above, what can hide things, not an obstacle by itself (only if plant_has_canopy).
 * Both carry h = plant_height_m. Every consumer reads them THE SAME WAY, here:
 *   * controller        — the trunk discs of plants in no row, to plan around and draw;
 *   * residual_concept  — the space a plant explains, so its returns stop being "unexplained" occupancy;
 *   * human_concept     — space that is already taken: no person is born inside a plant, and a LiDAR return
 *                         inside a plant belongs to the plant, not to a person standing next to it.
 *
 * PlantIndex answers "which plant is this point in?" in O(1) for per-point use (a LiDAR sweep is ~20k points
 * at ~17 Hz; testing each against every plant would cost far more than the question is worth).
 */
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <dsr/api/dsr_api.h>
#include <dsr/api/dsr_inner_eigen_api.h>

namespace rc::veg
{

struct PlantDisc
{
    std::string name;                               // the plant's node name (both of its discs carry it)
    Eigen::Vector2f c = Eigen::Vector2f::Zero();   // centre, world frame (m)
    float r = 0.0f;                                 // radius seen from above (m)
    float h = 0.0f;                                 // top above the ground (m); 0 = unknown

    [[nodiscard]] bool contains(const Eigen::Vector2f& p) const { return (p - c).squaredNorm() <= r * r; }
};

inline constexpr std::string_view kPlantPrefix = "plant_";
inline constexpr std::string_view kPlantType   = "plant";

enum class PlantPart { Trunk, Canopy, Any };

// One plant as consumers see it: where it stands and what of it has been perceived (conceptos.md, `plant`).
struct PlantView
{
    std::string name;
    std::uint64_t id = 0;
    Eigen::Vector2f origin = Eigen::Vector2f::Zero();   // world→plant RT: its trunk axis, else its canopy centre
    bool has_trunk = false;
    float trunk_r = 0.0f;                               // physical trunk radius (LiDAR); 0 = not measured yet
    float keepout_r = 0.0f;                             // the disc the trunk forbids
    bool has_canopy = false;
    Eigen::Vector2f canopy_c = Eigen::Vector2f::Zero(); // canopy centre, world frame
    float canopy_r = 0.0f;
    float height = 0.0f;
};

// Every plant with a resolvable world→plant transform.
// ★THE RT EDGE IS READ DIRECTLY, NOT THROUGH inner_eigen.transform(). Every plant hangs from the world frame,
// so world->plant IS that one edge — but transform() walks the tree with get_node() copies, and the world
// node it copies carries an RT edge (with its history ring) to EVERY plant and person. Its ts==0 cache does
// not help: any edge under the world frame changing (the robot's, at the localiser rate) invalidates it.
// Measured in the controller with 62 trunks: 48 ms a cycle this way, which alone kept the control loop below
// the LiDAR rate. get_edge(from, to) copies only the one edge. A plant parented elsewhere still resolves
// through the tree.
inline std::vector<PlantView> read_plants(DSR::DSRGraph& G, DSR::InnerEigenAPI& inner_eigen, const std::string& world_frame)
{
    std::vector<PlantView> out;
    const auto world_id = G.get_id_from_name(world_frame);
    const auto rt = G.get_rt_api();
    for (const auto& n : G.get_nodes_by_type(std::string(kPlantType)))
    {
        if (not n.name().starts_with(kPlantPrefix))
            continue;
        std::optional<Eigen::Vector3d> t;
        if (world_id.has_value())
            if (const auto e = G.get_edge(world_id.value(), n.id(), "RT"); e.has_value())
                if (const auto m = rt->get_edge_RT_as_rtmat(e.value()); m.has_value())
                    t = m->translation();
        if (not t.has_value())
            t = inner_eigen.transform(world_frame, n.name(), 0);
        if (not t.has_value() or not t->allFinite())
            continue;
        PlantView v;
        v.name = n.name();
        v.id = n.id();
        v.origin = {static_cast<float>(t->x()), static_cast<float>(t->y())};
        v.has_trunk = G.get_attrib_by_name<plant_has_trunk_att>(n).value_or(false);
        v.trunk_r = G.get_attrib_by_name<plant_trunk_radius_m_att>(n).value_or(0.0f);
        v.keepout_r = G.get_attrib_by_name<plant_keepout_radius_m_att>(n).value_or(0.0f);
        v.has_canopy = G.get_attrib_by_name<plant_has_canopy_att>(n).value_or(false);
        v.canopy_r = G.get_attrib_by_name<plant_canopy_radius_m_att>(n).value_or(0.0f);
        v.height = G.get_attrib_by_name<plant_height_m_att>(n).value_or(0.0f);
        v.canopy_c = v.origin;
        if (const auto o = G.get_attrib_by_name<plant_canopy_offset_att>(n); o.has_value() and o->get().size() >= 2)
            v.canopy_c += Eigen::Vector2f(o->get()[0], o->get()[1]);
        out.push_back(std::move(v));
    }
    return out;
}

// The discs of every plant: its trunk keep-out disc, its canopy disc, or both (Any).
inline std::vector<PlantDisc> read_plant_discs(DSR::DSRGraph& G, DSR::InnerEigenAPI& inner_eigen,
                                               const std::string& world_frame, PlantPart which = PlantPart::Any)
{
    std::vector<PlantDisc> out;
    for (const auto& v : read_plants(G, inner_eigen, world_frame))
    {
        if (which != PlantPart::Canopy and v.has_trunk and v.keepout_r > 0.0f)
            out.push_back({v.name, v.origin, v.keepout_r, v.height});
        if (which != PlantPart::Trunk and v.has_canopy and v.canopy_r > 0.0f)
            out.push_back({v.name, v.canopy_c, v.canopy_r, v.height});
    }
    return out;
}

// Spatial hash of the discs on a coarse grid: each disc is registered in every cell its bounding square touches,
// so a lookup checks only the handful of discs near the point.
class PlantIndex
{
public:
    PlantIndex() = default;
    explicit PlantIndex(std::vector<PlantDisc> discs, float cell_m = 1.0f) : discs_(std::move(discs)), cell_(cell_m)
    {
        for (std::size_t i = 0; i < discs_.size(); ++i)
        {
            const auto& d = discs_[i];
            for (int ix = cell(d.c.x() - d.r); ix <= cell(d.c.x() + d.r); ++ix)
                for (int iy = cell(d.c.y() - d.r); iy <= cell(d.c.y() + d.r); ++iy)
                    buckets_[key(ix, iy)].push_back(i);
        }
    }

    [[nodiscard]] bool empty() const { return discs_.empty(); }
    [[nodiscard]] const std::vector<PlantDisc>& discs() const { return discs_; }

    // The plant whose disc contains p (the nearest centre if two overlap), or nullptr.
    [[nodiscard]] const PlantDisc* containing(const Eigen::Vector2f& p) const
    {
        if (discs_.empty())
            return nullptr;
        const auto it = buckets_.find(key(cell(p.x()), cell(p.y())));
        if (it == buckets_.end())
            return nullptr;
        const PlantDisc* best = nullptr;
        float best_d2 = 0.0f;
        for (const std::size_t i : it->second)
        {
            const float d2 = (p - discs_[i].c).squaredNorm();
            if (discs_[i].contains(p) and (best == nullptr or d2 < best_d2))
            {
                best = &discs_[i];
                best_d2 = d2;
            }
        }
        return best;
    }

private:
    std::vector<PlantDisc> discs_;
    float cell_ = 1.0f;
    std::unordered_map<std::int64_t, std::vector<std::size_t>> buckets_;

    [[nodiscard]] int cell(float v) const { return static_cast<int>(std::floor(v / cell_)); }
    static std::int64_t key(int ix, int iy)
    {
        return (static_cast<std::int64_t>(ix) << 32) ^ static_cast<std::int64_t>(static_cast<std::uint32_t>(iy));
    }
};

}  // namespace rc::veg
