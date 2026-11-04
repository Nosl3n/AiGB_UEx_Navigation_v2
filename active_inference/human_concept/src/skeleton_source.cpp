/*
 * skeleton_source.cpp — CSV replay backend + factory.
 */

#include "skeleton_source.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <print>
#include <sstream>

#include <dsr/api/dsr_api.h>
#include <dsr/api/dsr_eigen_defs.h>

#include "csv_parse.h"   // ★locale-independent parsing — NEVER strtof here (see the header)

namespace rc {

namespace
{
std::vector<std::string> split(const std::string& line)
{
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) cols.push_back(cell);
    return cols;
}
}  // namespace

ReplaySkeletonSource::ReplaySkeletonSource(std::string path, bool loop)
    : loop_(loop)
{
    if (path.empty())
    {
        std::print("human_concept: [replay] no replay path configured\n");
        return;
    }
    std::ifstream f(path);
    if (not f.is_open())
    {
        std::print("human_concept: [replay] cannot open '{}'\n", path);
        return;
    }
    std::string line;
    int line_no = 0;
    while (std::getline(f, line))
    {
        ++line_no;
        if (line.empty() or line[0] == '#') continue;
        const auto c = split(line);
        // Column-count auto-detect: with/without an id column, with/without confidence.
        int base = -1;       // index of kp0_x
        bool has_id = false;
        if      (c.size() == 1 + 54 or c.size() == 1 + 54 + human::NUM_KP) { base = 1; has_id = false; }
        else if (c.size() == 2 + 54 or c.size() == 2 + 54 + human::NUM_KP) { base = 2; has_id = true; }
        else
        {
            std::print("human_concept: [replay] line {} has {} cols (skipped)\n", line_no, c.size());
            continue;
        }

        SkeletonBody b;
        b.id = has_id ? csv::parse_int(c[1]) : 0;
        b.kp.resize(human::NUM_KP, 3);
        for (int i = 0; i < human::NUM_KP; ++i)
            for (int k = 0; k < 3; ++k)
                b.kp(i, k) = csv::parse_float(c[base + i * 3 + k]);

        if (static_cast<int>(c.size()) >= base + 54 + human::NUM_KP)
        {
            std::array<float, human::NUM_KP> cf{};
            for (int i = 0; i < human::NUM_KP; ++i)
                cf[i] = csv::parse_float(c[base + 54 + i]);
            b.conf = cf;
        }
        rows_.push_back(std::move(b));
    }
    std::print("human_concept: [replay] loaded {} frames from '{}'\n", rows_.size(), path);
}

std::vector<SkeletonBody> ReplaySkeletonSource::poll()
{
    if (rows_.empty())
        return {};
    if (cursor_ >= rows_.size())
    {
        if (not loop_)
            return {};
        cursor_ = 0;
    }
    return { rows_[cursor_++] };
}

// ── DSR backend: retina 'skeleton' node → BODY_18 bodies (CAMERA→world transformed) ──────────

DsrSkeletonSource::DsrSkeletonSource(std::shared_ptr<DSR::DSRGraph> graph,
                                     DSR::InnerEigenAPI* inner_eigen,
                                     std::string world_frame,
                                     std::string camera_frame)
    : G_(std::move(graph)), inner_eigen_(inner_eigen),
      world_frame_(std::move(world_frame)), camera_frame_(std::move(camera_frame))
{}

std::vector<SkeletonBody> DsrSkeletonSource::poll()
{
    if (not G_)
        return {};

    const auto node_opt = G_->get_node("skeleton");
    if (not node_opt.has_value())
        return {};

    // TYPE-ATTRIBUTED reads (CLAUDE.md), compile-checked against dsr_attr_name.h. Optionals held so the
    // reference_wrapper payloads stay alive; skeleton_timestamp_ms is uint64 (read via .value()).
    const auto& node = node_opt.value();
    using VecOpt = std::optional<std::reference_wrapper<const std::vector<float>>>;
    const auto frame_opt = G_->get_attrib_by_name<skeleton_frame_id_att>(node);
    const auto count_opt = G_->get_attrib_by_name<skeleton_count_att>(node);
    const VecOpt ids_opt  = G_->get_attrib_by_name<skeleton_ids_att>(node);
    const VecOpt xyz_opt  = G_->get_attrib_by_name<skeleton_kp_xyz_att>(node);
    const VecOpt conf_opt = G_->get_attrib_by_name<skeleton_kp_conf_att>(node);
    const auto ts_opt     = G_->get_attrib_by_name<skeleton_timestamp_ms_att>(node);   // capture stamp (optional)
    if (not frame_opt or not count_opt or not xyz_opt)
        return {};

    const int frame_id = frame_opt.value();
    if (frame_id == last_frame_id_)   // same frame already consumed
        return {};
    last_frame_id_ = frame_id;

    const int count = std::max(0, count_opt.value());
    const auto& xyz  = xyz_opt.value().get();
    static const std::vector<float> empty_flat;
    const auto& ids  = ids_opt  ? ids_opt->get()  : empty_flat;
    const auto& conf = conf_opt ? conf_opt->get() : empty_flat;

    constexpr int K = human::NUM_KP;   // 18
    if (static_cast<int>(xyz.size()) < count * K * 3)
        return {};

    // Camera→world transform, PINNED to the frame's capture stamp (like the masks consumer) so the
    // keypoints are placed against the camera pose at acquisition time, not the latest pose — matters
    // while the robot is moving. ts==0 ⇒ the API falls back to the nearest/latest pose. Identity
    // fallback keeps camera-frame data flowing if the RT tree isn't ready yet (better than dropping
    // the frame); placement just won't be world-correct.
    const std::uint64_t capture_ts = ts_opt ? ts_opt.value() : 0;
    Mat::RTMat world_T_cam = Mat::RTMat::Identity();
    bool have_tf = false;
    if (inner_eigen_)
        if (const auto T = inner_eigen_->get_transformation_matrix(world_name(), camera_frame_, capture_ts); T.has_value())
        {
            world_T_cam = T.value();
            have_tf = true;
        }

    std::vector<SkeletonBody> bodies;
    bodies.reserve(static_cast<std::size_t>(count));
    for (int b = 0; b < count; ++b)
    {
        SkeletonBody body;
        body.id = (b < static_cast<int>(ids.size())) ? static_cast<int>(std::lround(ids[b])) : b;
        body.kp.resize(K, 3);

        std::array<float, K> cf{};
        for (int j = 0; j < K; ++j)
        {
            const std::size_t base = (static_cast<std::size_t>(b) * K + j) * 3;
            const float x = xyz[base + 0], y = xyz[base + 1], z = xyz[base + 2];
            if (std::isfinite(x) and std::isfinite(y) and std::isfinite(z))
            {
                const Mat::Vector3d p = world_T_cam * Mat::Vector3d(x, y, z);
                body.kp(j, 0) = static_cast<float>(p.x());
                body.kp(j, 1) = static_cast<float>(p.y());
                body.kp(j, 2) = static_cast<float>(p.z());
            }
            else
            {
                body.kp(j, 0) = body.kp(j, 1) = body.kp(j, 2) = std::numeric_limits<float>::quiet_NaN();
            }
            // Producer conf is [0,1]; the fitter convention (and replay) is [0,100].
            const std::size_t cidx = static_cast<std::size_t>(b) * K + j;
            cf[j] = (cidx < conf.size()) ? conf[cidx] * 100.0f : 0.0f;
        }
        body.conf = cf;
        bodies.push_back(std::move(body));
    }

    if (have_tf)   // ids only mean something in the world frame
        assign_track_ids(bodies, static_cast<std::int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch()).count()));
    if (not have_tf and not bodies.empty())
        std::print("human_concept: [dsr] no {}<-{} transform yet — skeletons left in camera frame\n",
                   world_name(), camera_frame_);
    return bodies;
}

std::unique_ptr<SkeletonSource> make_skeleton_source(const std::string& kind,
                                                     const std::string& replay_path,
                                                     bool replay_loop,
                                                     std::shared_ptr<DSR::DSRGraph> graph,
                                                     DSR::InnerEigenAPI* inner_eigen)
{
    if (kind == "dsr" or kind == "live")
    {
        if (graph)
            return std::make_unique<DsrSkeletonSource>(std::move(graph), inner_eigen);
        std::print("human_concept: [source] kind '{}' needs the DSR graph — falling back to replay\n", kind);
        return std::make_unique<ReplaySkeletonSource>(replay_path, replay_loop);
    }
    if (kind != "replay")
        std::print("human_concept: [source] unknown kind '{}' — defaulting to replay\n", kind);
    return std::make_unique<ReplaySkeletonSource>(replay_path, replay_loop);
}

// The frame the skeletons are expressed in: the configured one, or the graph's world frame if none.
std::string DsrSkeletonSource::world_name() const
{
    return world_frame_.empty() and G_ ? rc::world::frame_name(*G_) : world_frame_;
}

void DsrSkeletonSource::refresh_track(int id, const Eigen::Vector2f& xy, std::int64_t now_ms)
{
    if (const auto it = std::ranges::find(tracks_, id, &Track::id); it != tracks_.end())
    {
        it->xy = xy;
        it->seen_ms = now_ms;
    }
    else
        tracks_.push_back({id, xy, now_ms});   // retired while the camera was blind; the LiDAR kept the person
}

void DsrSkeletonSource::forget_track(int id)
{
    std::erase_if(tracks_, [id](const Track& t) { return t.id == id; });
}

// Greedy nearest-neighbour association on the body centre (hips, else neck, else the mean of the valid
// joints), cheapest pair first, so two people in view never trade identities. Tracks not seen for 3 s retire
// (their person node has long been pruned by human_concept's own absence counter by then).
void DsrSkeletonSource::assign_track_ids(std::vector<SkeletonBody> &bodies, std::int64_t now_ms)
{
    constexpr float kWalkMaxMps = 2.0f;     // a brisk walk / jog: the most a farm worker covers per second
    constexpr float kNoiseM     = 0.35f;    // centre jitter from keypoint depth noise and partial occlusion
    constexpr std::int64_t kRetireMs = 3000;
    std::erase_if(tracks_, [&](const Track &t) { return now_ms - t.seen_ms > kRetireMs; });

    const auto centre = [](const SkeletonBody &b) -> std::optional<Eigen::Vector2f>
    {
        const auto ok = [&](int j) { return std::isfinite(b.kp(j, 0)) and std::isfinite(b.kp(j, 1)); };
        if (ok(human::KP::R_HIP) and ok(human::KP::L_HIP))
            return Eigen::Vector2f(0.5f * (b.kp(human::KP::R_HIP, 0) + b.kp(human::KP::L_HIP, 0)),
                                   0.5f * (b.kp(human::KP::R_HIP, 1) + b.kp(human::KP::L_HIP, 1)));
        if (ok(human::KP::NECK)) return Eigen::Vector2f(b.kp(human::KP::NECK, 0), b.kp(human::KP::NECK, 1));
        Eigen::Vector2f s = Eigen::Vector2f::Zero(); int n = 0;
        for (int j = 0; j < human::NUM_KP; ++j) if (ok(j)) { s += Eigen::Vector2f(b.kp(j, 0), b.kp(j, 1)); ++n; }
        return n ? std::optional<Eigen::Vector2f>(s / static_cast<float>(n)) : std::nullopt;
    };

    struct Pair { float d; std::size_t b; std::size_t t; };
    std::vector<Pair> pairs;
    std::vector<std::optional<Eigen::Vector2f>> c(bodies.size());
    for (std::size_t b = 0; b < bodies.size(); ++b)
    {
        c[b] = centre(bodies[b]);
        if (not c[b]) continue;
        for (std::size_t t = 0; t < tracks_.size(); ++t)
        {
            const float gate = kNoiseM + kWalkMaxMps * 1e-3f * static_cast<float>(now_ms - tracks_[t].seen_ms);
            if (const float d = (*c[b] - tracks_[t].xy).norm(); d < gate) pairs.push_back({d, b, t});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair &a, const Pair &b) { return a.d < b.d; });
    std::vector<bool> body_done(bodies.size(), false), track_done(tracks_.size(), false);
    for (const auto &p : pairs)
    {
        if (body_done[p.b] or track_done[p.t]) continue;
        body_done[p.b] = track_done[p.t] = true;
        bodies[p.b].id = tracks_[p.t].id;
        tracks_[p.t].xy = *c[p.b];
        tracks_[p.t].seen_ms = now_ms;
    }
    // Two torsos cannot stand closer than ~a body width: an unmatched skeleton that close to one already placed
    // this frame is the SAME person detected twice (YOLO-pose duplicate), not a newcomer. It was births like
    // person_1 at 0.3 m from person_0, alive for the 2 s an orphan takes to die.
    constexpr float kTwoBodiesMinM = 0.5f;
    std::vector<bool> duplicate(bodies.size(), false);
    for (std::size_t b = 0; b < bodies.size(); ++b)
    {
        if (body_done[b] or not c[b]) continue;
        for (std::size_t o = 0; o < bodies.size(); ++o)
            if (o != b and body_done[o] and c[o] and (*c[b] - *c[o]).norm() < kTwoBodiesMinM)
                { duplicate[b] = true; break; }
        if (duplicate[b]) continue;
        bodies[b].id = next_track_id_++;
        tracks_.push_back({bodies[b].id, *c[b], now_ms});
        body_done[b] = true;
    }
    std::size_t k = 0;
    std::erase_if(bodies, [&](const SkeletonBody&) { return duplicate[k++]; });
}

}  // namespace rc
