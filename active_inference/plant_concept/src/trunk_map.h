// trunk_map.h — tree TRUNKS from two sensors: what the ZED calls "tree" low down, confirmed by the LiDAR.
//
// PURE Eigen (+ the header-only common/existence_belief): no DSR, no Qt, no Ice — unit-tested by self_test().
//
// ── WHY THE LOW BAND OF THE 'tree' MASK IS A TRUNK ──────────────────────────────────────────────────────
//   ADE20K has no trunk class: its 'tree' is trunk and canopy together. But a tree's canopy starts above its
//   trunk, so the 'tree' pixels whose 3D point lies in a band just above the ground and below the first
//   branch are trunk (and its stake). MEASURED in husky_apple.wbt (2026-10-14, 2 min, 2.2 M vegetation points):
//   in z 0.30-0.55 m, 93.5% of 'tree' points lie within 0.12 m of a real trunk and 99.2% within 0.25 m, while
//   in 0.70-1.20 m only 32% do (that is canopy). 56 of the 60 trunks showed up there, ~17 frames each.
//
// ── TWO SENSORS, TWO ROLES ──────────────────────────────────────────────────────────────────────────────
//   * ZED (semantic): says WHAT — a trunk candidate is a compact cluster of band points on a ground grid.
//   * LiDAR (geometry): says THAT something solid is there, and WHERE to ~2 cm. A candidate is born as a trunk
//     only once the LiDAR returns band points on it (multimodal birth): a semantic mistake alone cannot
//     create an obstacle the robot will never be allowed to cross.
//
// ── A TRUNK IS PERMANENT UNTIL VERIFIED GONE (active inference) ─────────────────────────────────────────
//   * It MOVES: every observation (ZED band points, LiDAR band returns) pulls its estimate with a precision
//     weight; a small process noise lets a localisation correction carry it.
//   * It is REMOVED only through its existence belief, and absence is charged ONLY when the trunk should
//     have been seen: inside the camera's field of view and range, and with nothing in between — a foliage
//     disc of ANOTHER plant (vegetation_concept) or a person. An occluded trunk is not evidence of absence:
//     HOLD. Absence is the CAMERA's verdict only: the LiDAR places, confirms and moves trunks but never
//     removes one (carving a box per trunk on every sweep would cost more than this verdict is worth).
#pragma once

#include "../../common/existence_belief/existence_belief.h"

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rc::trunk
{

struct TrunkParams
{
    float band_z_min = 0.30f;        // trunk band, world z (m): above the ground/beds ...
    float band_z_max = 0.55f;        // ... and below the first scaffold branch
    float cell_m = 0.05f;            // candidate evidence grid
    float cluster_r = 0.15f;         // mean-shift kernel radius for a candidate (a trunk + its stake)
    float birth_mass = 8.0f;         // cell-frames in a candidate cluster before the LiDAR is asked to confirm
    int   lidar_confirm_pts = 3;     // LiDAR band returns on a candidate, IN ONE SWEEP, needed for the birth ...
    float lidar_confirm_dz = 0.08f;  // ... spread over at least this much height: a trunk is vertical, ground noise
                                     // grazing the band is not (summed over sweeps, noise confirmed anything)
    float assoc_gate_m = 0.25f;      // a band point/return within this of a trunk belongs to that trunk
    float min_separation_m = 0.5f;   // two trunks cannot stand closer (the older one keeps the space)
    float trunk_radius_m = 0.04f;    // surface -> axis: returns lie on the near face
    float zed_sigma0 = 0.05f, zed_sigma_k = 0.01f;   // ZED point-cloud centroid σ = s0 + k·range² (m)
    float lidar_sigma = 0.03f;       // LiDAR centroid σ (m)
    float drift_q = 0.0004f;         // m² added per observation: lets a localisation correction move it
    float llr_hit = 1.0f, llr_miss = -0.6f;   // ZED evidence per look (log-odds), before p_vis weighting
    // ★PER-LOOK numbers are per REFERENCE look. birth_mass, llr_hit/llr_miss and drift_q were tuned with the semantic
    // model on one frame in six (~1.9 looks/s); every frame (and a faster model) gives 7x+ more looks, and frames
    // 80 ms apart are not independent looks — at face value trunks were born and removed that much faster. Each
    // look is weighted by min(1, time since the previous look / look_period_ms), so the evidence per SECOND is the
    // same at any semantic rate, and these numbers keep their meaning.
    float look_period_ms = 530.0f;
    float removal_prob = 0.1f;       // removed once P(exists) falls below this
    float zed_max_range = 4.0f;      // absence is charged only this close: a 7 cm trunk is ~7 px at 5 m and the
                                     // mask's 5 px morphological opening erases it — not seeing it there is no news
    float zed_min_range = 1.0f;      // ... and not closer: the band can fall below the image
    float zed_half_fov = 0.85f;      // rad, horizontal half field of view used for the visibility test
    float candidate_ttl_ms = 10000;  // candidate evidence not refreshed for this long is forgotten
    // ── Ablation switches. Defaults = the model above; false = the ablated model. ──
    bool lidar_confirm = true;       // false: a ZED candidate is born as a trunk on its own (semantic-only birth)
    bool visibility_model = true;    // false: every in-view look without the trunk is a full miss — no occlusion
                                     // test and no range-dependent p_vis (absence charged as if always resolvable)
};

struct Occluder { Eigen::Vector2f c; float r; };   // a disc that blocks the view (foliage, person)

struct Trunk
{
    int id = 0;
    Eigen::Vector2f c = Eigen::Vector2f::Zero();   // trunk axis, world frame (m)
    float var = 0.01f;                             // position variance (m², isotropic)
    rc::exist::ExistenceBelief existence{2.0f, 4.0f};
    int zed_hits = 0, zed_misses = 0, lidar_hits = 0;
    // Physical trunk radius, MEASURED by the LiDAR (one estimate per sweep that sees it, averaged): the helios
    // rays are uniform in bearing, so across the trunk's silhouette its returns are uniform in the lateral
    // coordinate, and for a uniform spread over [-R, R] the median |offset| is R/2 -> R = 2 · median |offset|.
    // The stake beside the trunk is cut off first (it is separated from the silhouette by a gap). 0 = unmeasured.
    float radius = 0.0f;
    int radius_n = 0;
    [[nodiscard]] float radius_or(float fallback) const { return radius_n > 0 ? radius : fallback; }
};

class TrunkMap
{
public:
    explicit TrunkMap(TrunkParams p = {}) : p_(p) {}
    void set_params(const TrunkParams& p) { p_ = p; }
    [[nodiscard]] const TrunkParams& params() const { return p_; }

    // One ZED frame: the band points of its 'tree' masks (world frame), the camera position and optical-axis
    // heading, and what could block its view. `looked` = the semantic model actually ran on this frame with
    // vegetation in view, i.e. a missing trunk on it is a real miss and not a skipped frame.
    // `people` are also excluded: a leg in the band is not a trunk, for either sensor.
    void add_zed_frame(const std::vector<Eigen::Vector3f>& band_pts, const Eigen::Vector2f& cam, float cam_heading,
                       const std::vector<Occluder>& occluders, const std::vector<Occluder>& people, bool looked,
                       std::int64_t now_ms);

    // One LiDAR sweep (world frame) and its origin.
    void add_lidar_sweep(const std::vector<Eigen::Vector3f>& sweep, const Eigen::Vector3f& origin,
                         const std::vector<Occluder>& people = {});

    // Births (LiDAR-confirmed candidates) and removals (existence verified gone).
    struct Changes { std::vector<int> born, removed; };
    Changes update(std::int64_t now_ms);

    [[nodiscard]] const std::vector<Trunk>& trunks() const { return trunks_; }
    [[nodiscard]] std::size_t pending() const { return candidates_.size(); }

    static bool self_test(std::string* report = nullptr);

private:
    TrunkParams p_;
    std::vector<Trunk> trunks_;
    std::int64_t last_look_ms_ = -1;   // previous ZED look, for the per-look weight (see look_period_ms)
    int next_id_ = 0;

    struct Cell { float evidence = 0.0f; std::int64_t last_ms = 0; };
    std::unordered_map<std::int64_t, Cell> grid_;   // ZED band evidence NOT explained by a trunk
    struct Candidate
    {
        Eigen::Vector2f c;                   // ZED cluster centre
        float mass = 0.0f;
        bool confirmed = false;              // one sweep met the LiDAR test (count + vertical spread)
        Eigen::Vector2f lidar_pos = Eigen::Vector2f::Zero();   // that sweep's trunk-axis estimate
        std::int64_t last_ms = 0;
    };
    std::vector<Candidate> candidates_;

    [[nodiscard]] int index(float v) const { return static_cast<int>(std::floor(v / p_.cell_m)); }
    [[nodiscard]] float centre_of(int i) const { return (static_cast<float>(i) + 0.5f) * p_.cell_m; }
    static std::int64_t key(int ix, int iy)
    {
        return (static_cast<std::int64_t>(ix) << 32) ^ static_cast<std::int64_t>(static_cast<std::uint32_t>(iy));
    }
    [[nodiscard]] Eigen::Vector2f cell_xy(std::int64_t k) const;
    [[nodiscard]] Trunk* nearest_trunk(const Eigen::Vector2f& q, float gate);
    void observe(Trunk& t, const Eigen::Vector2f& m, float sigma, float w = 1.0f);
    static Eigen::Vector2f median_xy(std::vector<Eigen::Vector2f>& pts);
    static bool inside(const Eigen::Vector2f& q, const std::vector<Occluder>& discs);
    [[nodiscard]] bool occluded(const Eigen::Vector2f& cam, const Trunk& t, const std::vector<Occluder>& occ) const;
    void refresh_candidates(std::int64_t now_ms);
};

}  // namespace rc::trunk
