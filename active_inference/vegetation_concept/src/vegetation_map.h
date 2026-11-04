// vegetation_map.h — from vegetation pixels to plants: an evidence grid on the ground plane and the discs on it.
//
// PURE Eigen: no DSR, no Qt, no Ice, so it can be unit-tested in isolation (self_test()).
//
// ── INPUT ───────────────────────────────────────────────────────────────────────────────────────────
//   One call per camera frame with the 3D points of that frame's VEGETATION masks (ADE20K tree/plant/palm/
//   flower, deprojected by retina, world frame). Points below z_min are dropped: the bottom edge of a mask
//   bleeds onto the soil, and a plant is what stands up from it.
//
// ── EVIDENCE ────────────────────────────────────────────────────────────────────────────────────────
//   A sparse grid on the ground plane. A cell gains ONE unit per frame in which any vegetation point fell in
//   it, however many points did: neighbouring pixels on one leaf are massively correlated, and counting them
//   would let one close-up frame outweigh twenty views. So a cell's evidence is "in how many frames was
//   vegetation seen here". Plants do not move, so evidence only accumulates (no decay).
//
// ── PLANTS ──────────────────────────────────────────────────────────────────────────────────────────
//   A plant is a DISC of fixed radius R (the canopy seen from above). Its centre is where a disc-kernel
//   mean shift over the evidence converges — the canopy's centre of mass as seen so far. Seen from one
//   corridor only the near face is visible, so the centre starts biased towards that corridor and settles
//   as the robot sees the plant from the other side too.
//   * Birth: from the strongest cell not yet inside a plant, mean-shift to a centre; a plant is born there
//     if the disc holds birth_mass cell-frames of evidence and does not overlap an existing plant.
//   * No overlap: two discs closer than 2R are one plant seen twice — the YOUNGER one (higher id) goes,
//     the first one seen keeps the space.
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rc::veg
{

struct VegetationParams
{
    float cell_m = 0.10f;            // evidence grid resolution
    float plant_radius_m = 0.5f;     // canopy radius seen from above (tall-spindle apple: ~1 m across)
    float z_min = 0.30f;             // drop points below this (mask bleed onto the soil / beds)
    float z_max = 6.0f;              // drop points above this (sky misclassified as foliage)
    float birth_mass = 150.0f;       // cell-frames in the disc for a birth: a disc is ~78 cells and one view lights
                                     // 20-50 of them, so this asks for several views, not one lucky frame
    // ★birth_mass counts REFERENCE looks. It was tuned with the semantic model on one frame in six (~1.9 looks/s);
    // every frame (and a faster model) gives 7x+ more, 80 ms apart and far from independent, and plants were born
    // that much sooner. Each frame weighs min(1, time since the previous one / look_period_ms), so the evidence
    // per SECOND does not depend on retina's semantic rate.
    float look_period_ms = 530.0f;
};

struct Plant
{
    int id = 0;
    Eigen::Vector2f c = Eigen::Vector2f::Zero();   // canopy centre (world frame, m)
    float height = 0.0f;                           // highest vegetation point seen in the disc (m)
    float mass = 0.0f;                             // evidence in the disc (cell-frames)
    float sigma_xy = 0.0f;                         // std of the centre estimate (m)
};

class VegetationMap
{
public:
    explicit VegetationMap(VegetationParams p = {}) : p_(p) {}

    void set_params(const VegetationParams& p) { p_ = p; }
    [[nodiscard]] const VegetationParams& params() const { return p_; }

    // One camera frame of vegetation points (world frame), taken at now_ms (< 0: a full reference look, for
    // tests). Returns how many cells gained evidence.
    int add_frame(const std::vector<Eigen::Vector3f>& pts, std::int64_t now_ms = -1);

    // Re-centre the plants on the evidence, drop overlapping duplicates, birth new plants.
    struct Changes { std::vector<int> born, removed; };
    Changes update(int max_births = 10);

    [[nodiscard]] const std::vector<Plant>& plants() const { return plants_; }
    [[nodiscard]] std::size_t cells() const { return evidence_.size(); }

    // Synthetic orchard with known plant centres; returns true on PASS and fills `report`.
    static bool self_test(std::string* report = nullptr);

private:
    VegetationParams p_;
    struct Cell { float evidence = 0.0f; float z_top = 0.0f; };
    std::unordered_map<std::int64_t, Cell> evidence_;
    std::vector<Plant> plants_;
    std::int64_t last_frame_ms_ = -1;   // previous frame, for the per-look weight (see look_period_ms)
    int next_id_ = 0;

    [[nodiscard]] int index(float v) const { return static_cast<int>(std::floor(v / p_.cell_m)); }
    [[nodiscard]] float centre_of(int i) const { return (static_cast<float>(i) + 0.5f) * p_.cell_m; }
    static std::int64_t key(int ix, int iy)
    {
        return (static_cast<std::int64_t>(ix) << 32) ^ static_cast<std::int64_t>(static_cast<std::uint32_t>(iy));
    }

    // Disc-kernel mean shift from `start`; fills the converged disc's mass, top and centre std.
    struct Shift { Eigen::Vector2f c; float mass = 0.0f, top = 0.0f, sigma = 0.0f; };
    [[nodiscard]] Shift mean_shift(Eigen::Vector2f start, int iterations) const;
    [[nodiscard]] Shift disc_stats(const Eigen::Vector2f& c) const;
};

}  // namespace rc::veg
