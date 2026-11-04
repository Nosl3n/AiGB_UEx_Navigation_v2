// field_explorer.h — the open field's INSPECTION planner: where should the robot go to learn the most
// about the field it has not yet seen? The open-field counterpart of room_concept's EpistemicPlanner.
//
// PURE Eigen: no DSR, no Qt, so it is unit-tested in isolation (self_test()).
//
// ── GENERATIVE MODEL ────────────────────────────────────────────────────────────────────────────
// The field is a grid of cells. Each cell carries the belief u_c in [0, 1] that its content (a plant or
// not) is still UNKNOWN; its entropy is H_c = u_c · H0, with H0 the prior entropy of one cell's content
// (binary entropy of the prior plant density). The perception stack (ZED masks confirmed by the LiDAR)
// detects what is in a cell at distance d with probability
//
//     p_det(d) = exp(-(d / sensor_range)²) · Π_rows τ          (τ = canopy transmittance per row crossed)
//
// and while the robot looks the unknown belief decays as a Poisson process of rate κ·p_det:
//
//     u_c ← u_c · exp(-κ · p_det · dt)
//
// A viewpoint v is worth the EXPECTED ENTROPY REDUCTION of looking from it for a dwell T:
//
//     ΔH(v) = Σ_c H_c · (1 − exp(−κ · p_det(v, c) · T))          [nats]
//
// and the planner picks argmax ΔH(v) − λ·dist(robot, v) (travel is a cost in the same currency, as in
// the controller's selection). The inspection is COMPLETE when no reachable viewpoint is worth its
// travel: max_v [ΔH(v) − λ·dist] ≤ 0. That is the sign of the expected free energy, not a tuned cutoff.
//
// ── WHAT IS A VALID STANDPOINT ──────────────────────────────────────────────────────────────────
// A cell centre inside the field polygon, at least `boundary_clearance` from its edge, outside every row
// rectangle grown by `row_clearance`, and outside every plant keep-out disc grown by `plant_clearance`.
#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rc::openfield
{

struct ExplorerParams
{
    float cell_m = 1.0f;               // grid resolution (m)
    float sensor_range_m = 5.0f;       // e-folding range² of p_det (trunk detection range of ZED + LiDAR)
    float canopy_transmittance = 0.25f;// fraction of a look that passes through one row of canopy
    float detect_rate = 1.0f;          // κ: looks per second at p_det = 1
    float dwell_s = 3.0f;              // T: how long the robot looks from a standpoint (≈ controller dwell)
    float prior_plant_p = 0.3f;        // prior probability that a cell holds a plant → H0 = H_bin(p)
    float travel_cost = 0.2f;          // λ: nats per metre (same as the controller's AffordanceLambdaCost)
    float boundary_clearance = 1.0f;   // m from the field polygon
    float row_clearance = 0.75f;       // m around a row rectangle (robot half-width + margin)
    float plant_clearance = 0.6f;      // m around a plant keep-out disc
    float failed_suppress_s = 60.0f;   // decay time of the suppression of an unreachable standpoint
    float failed_radius_m = 1.5f;      // ... and its reach: the neighbouring cells fail for the same reason
};

// An oriented rectangle (a row's footprint) and a disc (a plant's keep-out) in the world frame.
struct RowRect
{
    Eigen::Vector2f c{0.f, 0.f};
    float yaw = 0.f, length = 0.f, width = 0.f;   // length along the row, width across it
};
struct Disc
{
    Eigen::Vector2f c{0.f, 0.f};
    float r = 0.f;
};

struct ExplorerTarget
{
    Eigen::Vector2f pos{0.f, 0.f};
    float yaw = 0.f;       // facing direction (atan2 convention), toward what is still unknown
    float gain = 0.f;      // ΔH (nats), the value published on the affordance
    float score = 0.f;     // ΔH − λ·dist
};

class FieldExplorer
{
public:
    explicit FieldExplorer(ExplorerParams p = {}) : p_(p) {}

    void set_params(const ExplorerParams &p) { p_ = p; }
    [[nodiscard]] const ExplorerParams &params() const { return p_; }

    // (Re)build the grid over the polygon. Resets the belief only if the polygon changed.
    void set_polygon(const std::vector<Eigen::Vector2f> &polygon);
    void set_obstacles(std::vector<RowRect> rows, std::vector<Disc> plants);

    // The robot looked for dt seconds from `pos`: decay the unknown belief of every cell it can see.
    void observe(const Eigen::Vector2f &pos, float dt);

    // Best standpoint from `robot`, or nullopt when nothing is worth the travel (inspection complete).
    [[nodiscard]] std::optional<ExplorerTarget> select(const Eigen::Vector2f &robot, double now_s) const;
    // ΔH of looking from `v` (nats), recomputed live (the belief keeps changing while the robot drives).
    [[nodiscard]] float gain_at(const Eigen::Vector2f &v) const;

    // The consumer could not get there: suppress that standpoint for a while (decaying, not a ban).
    void mark_failed(const Eigen::Vector2f &pos, double now_s);

    [[nodiscard]] bool ready() const { return nx_ > 0 and ny_ > 0; }
    [[nodiscard]] float entropy() const;           // Σ H_c (nats)
    [[nodiscard]] float initial_entropy() const { return h_init_; }
    [[nodiscard]] float remaining_fraction() const { return h_init_ > 0.f ? entropy() / h_init_ : 1.f; }

    // For the viewers: cell centres and their unknown belief u_c.
    [[nodiscard]] std::vector<std::pair<Eigen::Vector2f, float>> cells() const;
    [[nodiscard]] float cell_m() const { return p_.cell_m; }

    static bool self_test(std::string *report = nullptr);

private:
    ExplorerParams p_;
    std::vector<Eigen::Vector2f> polygon_;
    std::vector<RowRect> rows_;
    std::vector<Disc> plants_;
    Eigen::Vector2f origin_{0.f, 0.f};   // centre of cell (0, 0)
    int nx_ = 0, ny_ = 0;
    std::vector<float> u_;               // unknown belief per cell, -1 = outside the polygon
    float h0_ = 0.f, h_init_ = 0.f;
    struct Failed { Eigen::Vector2f pos; double t; };
    std::vector<Failed> failed_;

    [[nodiscard]] Eigen::Vector2f centre(int ix, int iy) const;
    [[nodiscard]] float p_det(const Eigen::Vector2f &from, const Eigen::Vector2f &to) const;
    [[nodiscard]] bool standpoint_ok(const Eigen::Vector2f &v) const;
    [[nodiscard]] float facing(const Eigen::Vector2f &v) const;
    template <typename F> void for_cells_near(const Eigen::Vector2f &v, F &&f) const;
};

bool point_in_polygon(const Eigen::Vector2f &p, const std::vector<Eigen::Vector2f> &poly);
float distance_to_polygon_edge(const Eigen::Vector2f &p, const std::vector<Eigen::Vector2f> &poly);

}  // namespace rc::openfield
