/*
 * human_lidar_presence.cpp — see human_lidar_presence.h.
 */

#include "human_lidar_presence.h"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <print>

#include "../../common/existence_belief/existence_belief.h"
#include "human_scene_graph.h"

namespace rc {

namespace
{
rc::exist::SensorModel sensor_model(const HumanConfig& c)
{
    rc::exist::SensorModel sm;
    sm.sensor_sigma_m = 0.03f;   // helios range noise (HuskyA300.proto heliosNoiseM = 0.02, plus registration)
    sm.detection_prob = c.exist_detection_prob;
    sm.clutter_prob   = c.exist_clutter_prob;
    return sm;
}

// Rigidly translate the published belief (smoothed command pose + raw fit + Kabsch translation) in xy, so
// the skeleton the graph shows moves with the person and the next camera fit blends from the right place.
void shift_xy(HumanInstance& inst, const Eigen::Vector2f& d)
{
    for (int i = 0; i < human::NUM_KP; ++i)
    {
        inst.cmd_kp(i, 0) += d.x();                         inst.cmd_kp(i, 1) += d.y();
        inst.last_result.kp_pred_aligned(i, 0) += d.x();    inst.last_result.kp_pred_aligned(i, 1) += d.y();
    }
    inst.t_sm.x() += d.x();
    inst.t_sm.y() += d.y();
}
}  // namespace

std::optional<Eigen::Vector2f> HumanLidarPresence::centre_of(const HumanInstance& inst)
{
    if (not inst.has_result)
        return std::nullopt;
    const Eigen::Vector3f p = HumanSceneGraph::pelvis_of(inst.has_cmd ? inst.cmd_kp : inst.last_result.kp_pred_aligned);
    if (not p.allFinite())
        return std::nullopt;
    return Eigen::Vector2f(p.x(), p.y());
}

void HumanLidarPresence::on_camera_seen(HumanInstance& inst, std::int64_t now_ms) const
{
    const auto sm = sensor_model(cfg_);
    // One confident detection's worth: log P(skeleton | person) / P(skeleton | nobody).
    inst.existence.integrate(1.0f, std::log(sm.detection_prob / sm.clutter_prob));
    inst.last_evidence_ms = now_ms;
    inst.last_fix_ms      = now_ms;
    inst.fix_by_camera    = true;
}

int HumanLidarPresence::step(std::unordered_map<std::uint64_t, HumanInstance>& instances,
                             const std::vector<Eigen::Vector3f>& sweep, const Eigen::Vector3f& origin,
                             std::int64_t now_ms, float robot_yaw_rate)
{
    const auto sm = sensor_model(cfg_);
    const float r = cfg_.body_radius_m;

    // Returns inside a plant are the plant's (see set_plants): drop them before anyone can claim them.
    std::vector<Eigen::Vector3f> outside_plants;
    if (not plants_.empty())
    {
        outside_plants.reserve(sweep.size());
        for (const auto& q : sweep)
            if (plants_.containing(Eigen::Vector2f(q.x(), q.y())) == nullptr)
                outside_plants.push_back(q);
    }
    const auto& pts = plants_.empty() ? sweep : outside_plants;

    // Who is where, and how uncertain: the σ of the last fix (camera: stereo depth, ∝ range²; LiDAR: a few cm)
    // plus how far the person could have walked since. The follow gate is r + 2σ, capped.
    constexpr float kLidarFixSigmaM = 0.05f;
    struct P { HumanInstance* inst; Eigen::Vector2f c; float sigma; float gate; Eigen::Vector2f sum = Eigen::Vector2f::Zero(); int n = 0; };
    std::vector<P> persons;
    const Eigen::Vector2f o(origin.x(), origin.y());
    for (auto& [id, inst] : instances)
        if (const auto c = centre_of(inst); c.has_value())
        {
            const float dt = 1e-3f * static_cast<float>(std::max<std::int64_t>(0, now_ms - inst.last_fix_ms));
            const float range = (*c - o).norm();
            const float s_fix = inst.fix_by_camera ? cfg_.cam_depth_sigma_k * range * range : kLidarFixSigmaM;
            // A turning robot misregisters the sweep against its pose by yaw-rate × latency, i.e. a lateral
            // error growing with range: 0.6 rad/s × 50 ms × 12 m = 0.36 m — the whole torso box.
            const float s_turn = std::abs(robot_yaw_rate) * cfg_.lidar_latency_s * range;
            const float sigma = std::min(s_fix + s_turn + cfg_.walk_max_mps * dt, cfg_.follow_gate_max_m);
            persons.push_back({&inst, *c, sigma, std::min(r + 2.0f * sigma, cfg_.follow_gate_max_m)});
        }
    if (persons.empty())
    {
        prev_sweep_ms_ = now_ms;
        return 0;
    }

    // Torso-band returns, each given to the NEAREST person whose gate holds it, so two workers walking side by
    // side cannot both grab the same body.
    for (const auto& q : pts)
    {
        if (q.z() < cfg_.lidar_band_z_min or q.z() > cfg_.lidar_band_z_max)
            continue;
        const Eigen::Vector2f xy(q.x(), q.y());
        P* best = nullptr;
        float best_d = std::numeric_limits<float>::max();
        for (auto& p : persons)
            if (const float d = (xy - p.c).norm(); d < p.gate and d < best_d) { best = &p; best_d = d; }
        if (best) { best->sum += xy; ++best->n; }
    }

    int followed = 0;
    for (auto& p : persons)
    {
        auto& inst = *p.inst;
        inst.lidar_hits = p.n;
        inst.lidar_range_m = (p.c - o).norm();
        if (std::getenv("HUMAN_LIDAR_TRACE"))
            std::print("human_concept: [lidar] {} range={:.1f} m band pts={} gate={:.2f} σ={:.2f} cam_fresh={}\n",
                       inst.node_name, inst.lidar_range_m, p.n, p.gate, p.sigma, inst.last_seen_ms > prev_sweep_ms_);
        // ANY torso-band return inside this person's gate is this person: the band is above the crops, the gate
        // is where they can be, and the helios has no spurious returns worth the name. Requiring 3 was a hole at
        // the ring gap — measured band points vs range (2026-10-01): 7 m 14, 9 m 6, 10 m 4, 11-13 m ~2, 13-14 m 6.
        // The worker was dropped exactly at 11-13 m every lap. One return biases the centre sideways by < r.
        constexpr int kMinCentroidPts = 1;
        // Where to carve, and how sharply. The LiDAR's own torso centre when it found one (its range noise is
        // centimetres); otherwise the believed position, whose uncertainty is what the person could have
        // walked since the last fix. Carving a camera-placed box SHARPLY was a fight the camera lost: at 10-14 m
        // the ZED puts the pelvis 0.15-0.3 m off, beams slip past the box edge as "through", and a worker the
        // camera was fitting 4x/s got removed (2026-10-01, person_1 after 85 fits).
        Eigen::Vector2f carve_c = p.c;
        float carve_sigma = p.sigma;
        if (p.n >= kMinCentroidPts)
        {
            // Returns lie on the near surface; the torso centre is ~r/2 further along the ray.
            const Eigen::Vector2f m = p.sum / static_cast<float>(p.n);
            const Eigen::Vector2f ray = (m - o).norm() > 1e-3f ? Eigen::Vector2f((m - o).normalized()) : Eigen::Vector2f::Zero();
            const Eigen::Vector2f c_lidar = m + 0.5f * r * ray;
            carve_c = c_lidar;
            carve_sigma = kLidarFixSigmaM + std::abs(robot_yaw_rate) * cfg_.lidar_latency_s * (c_lidar - o).norm();
            if (inst.last_seen_ms > prev_sweep_ms_)
                inst.lidar_cam_gap_m = (c_lidar - p.c).norm();   // camera owns the position this sweep: just measure
            else
            {
                shift_xy(inst, c_lidar - p.c);                   // camera blind: the LiDAR carries the person
                p.c = c_lidar;
                inst.last_fix_ms = now_ms;
                inst.fix_by_camera = false;
                ++inst.lidar_follows;
                ++followed;
            }
        }

        // Existence: carve a torso box at the (possibly re-centred) belief.
        rc::exist::Evidence ev = rc::exist::carve_box(origin, pts, carve_c.x(), carve_c.y(), 0.0f, 2.0f * r, 2.0f * r,
                                                      cfg_.lidar_band_z_min, cfg_.lidar_band_z_max, kLidarFixSigmaM, sm);
        // carve_box's own two-outcome ΔL: a return anywhere inside the box is FOR, a beam through it AGAINST.
        // Not solid_delta: the box is a 2r square around a torso ~0.25 m deep whose centre is only known to
        // ~0.1 m, so a return "deep inside" it is the person slightly off-centre, not a wall behind a phantom.
        inst.lidar_e_occ = ev.e_occ + ev.e_interior; inst.lidar_e_free = ev.e_free; inst.lidar_sigma = carve_sigma;
        // A beam through the box only says "not HERE". When the person could be anywhere within σ of it, the
        // box holds them with probability ~(r / (r+σ))², and a beam passing beside them is no evidence of
        // absence. So free space is weighted by that; returns are not (nothing else stands in an open field at
        // torso height). Without this, a far worker the LiDAR briefly lost was "seen through" and removed.
        if (ev.n_reached > 0)
        {
            const float pd = std::clamp(sm.detection_prob, 1e-3f, 1.0f - 1e-3f);
            const float pc = std::clamp(sm.clutter_prob,   1e-3f, 1.0f - 1e-3f);
            const float llr_occ = std::log(pd / pc), llr_free = std::log((1.0f - pd) / (1.0f - pc));
            const float w_free = (r / (r + carve_sigma)) * (r / (r + carve_sigma));
            ev.log_odds_delta = rc::exist::saturate((ev.e_occ + ev.e_interior) * llr_occ + w_free * ev.e_free * llr_free,
                                                    llr_occ);
            inst.existence.integrate(ev, 1.0f);
            inst.last_evidence_ms = now_ms;
        }
    }
    prev_sweep_ms_ = now_ms;
    return followed;
}

}  // namespace rc
