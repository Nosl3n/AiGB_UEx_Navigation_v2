/*
 * human_config.h
 *
 * Plain-data configuration for the human_concept agent + a loader that fills it
 * from a RoboComp ConfigLoader. Mirrors bottle_config.h, but the tunables are the
 * active-inference pose-estimator knobs (the SDF/mask/support-bank knobs do not apply —
 * human_concept fits a kinematic model to BODY_18 keypoints, see cpp/core).
 */

#pragma once

#include <string>
#include <vector>

class ConfigLoader;   // RoboComp config façade (defined in genericworker.h)

namespace rc {

struct HumanConfig
{
    std::string priors_path = "etc/object_priors.toml";

    // ── Skeleton input source (decoupled; replay-first, see skeleton_source.h) ──────────────────────
    std::string source_kind = "replay";   // "replay" (CSV) | "live" (future: ZED / media plane)
    std::string replay_path = "";          // CSV of keypoints (frame[,id],54 kp[,18 conf]); empty = none
    bool        replay_loop = true;        // wrap to the first frame at EOF

    // ── AInfLaplacePoseEstimator knobs (forwarded to rc::human::InferenceConfig) ────────────────────
    std::vector<int> anchors = {1, 2, 5, 8, 11};   // neck, shoulders, hips
    float sigma_obs  = 0.06f;
    float sigma_dyn  = 0.25f;
    float sigma_min  = 0.02f;
    float sigma_max  = 0.15f;
    float min_kp_conf = 15.0f;   // [0,100] hard floor: drop keypoints below this from fit + calibration
    float w_limits   = 5.0f;
    float w_sym      = 1.0f;
    int   gn_steps   = 2;
    float damping    = 1e-3f;

    // Anti arm-cross (sidedness) prior — filters YOLO L/R swaps. Soft; allows folded arms.
    float w_cross          = 3.0f;
    float arm_cross_margin = 0.05f;   // m past the opposite shoulder before it penalises
    // Neutral-pose prior — weak pull of arm DOFs toward rest; pins under-observed DOFs (stops drift).
    float w_neutral        = 1.0f;
    // Innovation gate (rad): reject + hold on a per-frame angle jump bigger than this (glitch filter).
    float max_innovation   = 1.0f;

    // Velocity/accel limits — applied by the OUTPUT CONTROLLER (human_controller.h), NOT inside the
    // fit. The in-fit penalty weights stay 0 (the controller replaced them). omega/alpha = angle DOFs;
    // vlin/alin = lower-body translation. dt = measured inter-fit interval (fallback from Period.Compute).
    float dt         = 0.05f;   // s between fits (fallback; runtime uses the measured interval)
    float w_vel      = 0.0f;    // in-fit velocity penalty (OFF — controller owns this)
    float w_acc      = 0.0f;    // in-fit acceleration penalty (OFF)
    float omega_max  = 3.0f;    // rad/s  (angle DOFs) — natural limb speed
    float alpha_max  = 12.0f;   // rad/s² (angle DOFs) — gentle ease-in/out
    float pose_smooth = 0.2f;   // global-pose EMA weight on the new Kabsch R,t (lower = steadier facing)
    bool  calibrate_bones = true;   // online per-person bone-length calibration (matches proportions → lower FE)
    float calib_smooth    = 0.05f;  // EMA weight on each new measured segment length (lower = steadier)
    float vlin_max   = 3.0f;    // m/s    (lb_x, lb_z)
    float alin_max   = 30.0f;   // m/s²   (lb_x, lb_z)
    std::string fit_csv_path = "";   // non-empty → per-cycle fit-diagnostics CSV (gate)

    // ── Simple active-inference action policy (the prototype's "action hint") ────────────────────────
    int   death_frames       = 60;     // (legacy, unused: see death_timeout_s)
    // Seconds a person may go UNSEEN before its node is removed; while seen it lives on. A wall clock, not a
    // cycle count: frames_since_detection only advanced when fresh data arrived, so a person who stopped being
    // seen was never counted as absent and lived forever (2026-10-01: 35 nodes for 2 workers).
    float death_timeout_s    = 2.0f;   // now: seconds with NO evidence at all (neither camera nor LiDAR)
    // A new track becomes a person node only after this many camera fits on the SAME track (a tentative track
    // until then). A skeleton placed in a crop row by an occluding leaf jumps from leaf to leaf as the robot
    // moves, so it never collects them; a real worker does within a fraction of a second.
    int   birth_min_fits     = 3;

    // ── LiDAR "second look" (existence + follow; see human_lidar_presence.h) ─────────────────────────
    bool  lidar_enabled        = true;
    float lidar_band_z_min     = 1.20f;  // torso band, field frame (m above ground). Above the young palms'
    float lidar_band_z_max     = 1.85f;  //   crown top (0.95 m) so a palm can neither confirm nor occlude a person
    float body_radius_m        = 0.25f;  // torso half-width: the carved box is 2r × 2r
    float walk_max_mps         = 2.0f;   // follow gate grows with this × time since the last position fix
    float follow_gate_max_m    = 1.0f;   // ...up to this
    float lidar_latency_s      = 0.05f;  // sweep-stamp vs robot-pose misregistration; × yaw rate × range = σ
    float cam_depth_sigma_k    = 0.002f; // camera-placed position σ = k·range² (stereo depth); 0.29 m at 12 m —
                                         //   measured |LiDAR − camera| 0.15-0.3 m at 10-14 m (2026-10-01)
    float exist_detection_prob = 0.85f;  // P(LiDAR/camera sees the person | there)
    float exist_clutter_prob   = 0.05f;  // P(returns in the torso box | nobody there)
    float exist_l0             = 2.0f;   // log-odds at birth (one skeleton is real evidence)
    float exist_l_max          = 4.0f;
    float exist_removal_prob   = 0.2f;   // remove once P(exists) falls below this
    int   min_valid          = 12;     // fewer valid joints ⇒ raise the look affordance
    float uncertainty_thresh = 0.05f;  // tr(cov) above this ⇒ raise the look affordance

    // ── Epistemic "reduce-occlusion" affordance ─────────────────────────────────────────────────────
    float epistemic_obs_distance    = 1.5f;   // stand-off (m) from the person at the look viewpoint
    float epistemic_view_info       = 50.0f;  // Fisher precision a clearer view is expected to add (ΔH scale)
    int   epistemic_cooldown_cycles = 120;    // post-completion hold (cycles) during which the gain is suppressed
    std::string epistemic_csv_path  = "";     // non-empty → per-cycle epistemic/affordance CSV

    // ── Observation-stillness (affordance contract .still): hold base speed below these for a clean look ─
    float still_vel   = 0.10f;   // m/s
    float still_omega = 0.15f;   // rad/s

    // ── Fisher CSV (per-cycle belief evolution; gated) ──────────────────────────────────────────────
    std::string fisher_csv_path = "";

    // Pose covariance written on the room→person RT edge: a diagonal scaled by tr(cov) (m²). The
    // estimator's Lambda is over joint angles, not the global pose, so this is a coarse proxy until a
    // full free-flyer reformulation folds the pose into the belief (see plan / Pinocchio note).
    float pose_cov_scale = 0.02f;
};

// Fill a HumanConfig from a RoboComp ConfigLoader (all keys + defaults).
HumanConfig load_human_config(const ConfigLoader& cfg);

}  // namespace rc
