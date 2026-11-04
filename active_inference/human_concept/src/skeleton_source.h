/*
 * skeleton_source.h
 *
 * Decoupled body-keypoint input for human_concept. The agent consumes BODY_18 3D
 * skeletons through this abstract interface, so the live source (ZED SDK / a media-
 * plane skeleton frame / a DSR producer) can be swapped later without touching the
 * fitter or scene graph. The first backend is a CSV replay (reuses the cpp/harness
 * format), so the full agent can be brought up and tested without a camera.
 */

#pragma once

#include "../../common/world_frame/world_frame.h"   // rc::world:: — room indoors, field outdoors
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "body18.h"                  // rc::human::NUM_KP
#include "human_kinematic_model.h"   // rc::human::KpArray

namespace DSR { class DSRGraph; class InnerEigenAPI; }

namespace rc {

// One tracked body in a frame: a stable track id + its 18×3 keypoints (+ optional per-joint conf).
struct SkeletonBody
{
    int     id = 0;
    human::KpArray kp;                                        // (18,3); NaN rows = missing
    std::optional<std::array<float, human::NUM_KP>> conf;     // per-joint confidence in [0,100]
};

// Abstract source. poll() returns the bodies for the current step and advances internally.
class SkeletonSource
{
public:
    virtual ~SkeletonSource() = default;
    virtual std::vector<SkeletonBody> poll() = 0;
    virtual bool ok() const = 0;
    // Feedback from the person beliefs: a track the LiDAR still holds (keep it alive, at its new place) and a
    // track whose person was removed (drop it, so a new arrival there is a new person).
    virtual void refresh_track(int /*id*/, const Eigen::Vector2f& /*xy*/, std::int64_t /*now_ms*/) {}
    virtual void forget_track(int /*id*/) {}
};

// CSV replay backend. Row format (lines starting with '#' ignored):
//   frame[,id], kp0_x..kp17_z (54) [, c0..c17 (18)]
// id column is auto-detected from the column count; absent → id 0. Each row is one body/step.
class ReplaySkeletonSource : public SkeletonSource
{
public:
    ReplaySkeletonSource(std::string path, bool loop);
    std::vector<SkeletonBody> poll() override;
    bool ok() const override { return not rows_.empty(); }

private:
    std::vector<SkeletonBody> rows_;
    std::size_t cursor_ = 0;
    bool loop_ = true;
};

// Live DSR backend: reads the retina's 'skeleton' node (BODY_18 keypoints in the CAMERA frame,
// metres) and returns one SkeletonBody per detected person. Keypoints are transformed CAMERA→world
// (room) via the live camera RT so the existing room-frame fitter places people correctly; the
// on-wire data stays localization-clean, leaving the interaction-time re-parent (person→robot for
// visual servoing) to a future path. Returns empty until a new frame id appears (no reprocessing).
class DsrSkeletonSource : public SkeletonSource
{
public:
    DsrSkeletonSource(std::shared_ptr<DSR::DSRGraph> graph,
                      DSR::InnerEigenAPI* inner_eigen,
                      std::string world_frame = "",   // "" = the live world frame (field outdoors, room indoors)
                      std::string camera_frame = "zed");
    std::vector<SkeletonBody> poll() override;
    bool ok() const override { return static_cast<bool>(G_); }
    void refresh_track(int id, const Eigen::Vector2f& xy, std::int64_t now_ms) override;
    void forget_track(int id) override;

private:
    std::shared_ptr<DSR::DSRGraph> G_;
    DSR::InnerEigenAPI* inner_eigen_ = nullptr;
    std::string world_frame_;   // fixed name, or "" to follow rc::world (see world_name())
    std::string world_name() const;
    std::string camera_frame_;
    int last_frame_id_ = -1;

    // ── TRACK IDs: retina's skeleton_ids are per-frame DETECTION indices, not tracks ───────────────────
    // YOLO-pose numbers the people it sees 0, 1, … in each frame, in no stable order. Used as identities
    // (fitter: body.id == inst.track_id), two people in view swapped between frames, person_0 was fed both
    // skeletons and its fitted position settled BETWEEN them (3 m off on 2026-10-01), then went "unseen" and
    // was re-born. Here each body is associated to the nearest live track in the WORLD frame; the gate is
    // what a person can physically cover since that track was last seen (walking speed x elapsed time, plus
    // the keypoint noise), not a tuned number. No match => a new track id.
    struct Track { int id; Eigen::Vector2f xy; std::int64_t seen_ms; };
    std::vector<Track> tracks_;
    int next_track_id_ = 0;
    void assign_track_ids(std::vector<SkeletonBody> &bodies, std::int64_t now_ms);
};

// Factory from config. kind = "replay" (CSV) | "dsr"/"live" (retina 'skeleton' node). The DSR
// backend needs the graph + inner-eigen API; if absent it falls back to replay.
std::unique_ptr<SkeletonSource> make_skeleton_source(const std::string& kind,
                                                     const std::string& replay_path,
                                                     bool replay_loop,
                                                     std::shared_ptr<DSR::DSRGraph> graph = nullptr,
                                                     DSR::InnerEigenAPI* inner_eigen = nullptr);

}  // namespace rc
