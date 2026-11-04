// openfield_exploration.h — the DSR side of the open field's INSPECTION mode: openfield_concept's port of
// room_concept's exploration (EpistemicPlanner + afford_room), for an orchard instead of a room.
//
//   * field_explorer.h decides WHERE to look next (expected entropy reduction of the field map).
//   * This class offers that standpoint to the controller as the affordance `afford_openfield` (type
//     `affordance`, parented to the `world` node; affordance_protocol: Offered → Executing → Completed), and
//     follows the consumer's outcome: a standpoint it could not reach is suppressed, one it reached is done.
//   * It publishes the inspection state on the `world` node: model_stable = the inspection is COMPLETE (no
//     standpoint left is worth its travel), model_uncertainty = the fraction of the field's entropy left.
//     field_metaconcept closes the field (field_0) on model_stable.
//
// Modes: Waiting (no pose / no field polygon yet) → Inspecting → Complete. Complete is sticky for the run.
//
// MAIN THREAD ONLY for the graph work (tick()); the standpoint search runs on a std::async worker over a
// COPY of the explorer (pure data, no graph), because one search over the field takes ~150 ms.
#pragma once

#include "field_explorer.h"

#include "../../common/affordance_manager/affordance_manager.h"

#include <dsr/api/dsr_api.h>
#include <dsr/api/dsr_inner_eigen_api.h>

#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rc::openfield
{

class OpenFieldExploration
{
public:
    struct Config
    {
        bool enabled = true;
        ExplorerParams explorer;
        int period_ms = 200;                 // belief update + protocol rate
        int obstacles_period_ms = 1000;      // re-read rows and plants from the graph
        std::int64_t offer_unclaimed_ms = 10000;   // an offer nobody claims is retired after this
        std::int64_t lease_ms = 180000;      // a claim held this long without completing is retired (backstop)
        // ★PROGRESS WATCHDOG (room_concept's break_execution_stall): a claimed standpoint the robot has not
        // come closer to for this long is released and suppressed. A threshold on purpose, like room's: it is
        // a liveness guarantee about another agent, not a belief, so there is no model term it could live in.
        std::int64_t stall_ms = 30000;
        float stall_progress_m = 0.3f;       // what counts as "came closer"
    };
    enum class Mode { Waiting, Inspecting, Complete };

    OpenFieldExploration(std::shared_ptr<DSR::DSRGraph> G, Config cfg);
    ~OpenFieldExploration();

    // One step. `robot` = localized position (nullopt until the filter is initialized).
    void tick(std::int64_t now_ms, const std::vector<Eigen::Vector2f> &polygon,
              const std::optional<Eigen::Vector2f> &robot, std::uint64_t world_id, const std::string &world_frame);

    // Delete our affordance node (graceful shutdown).
    void cleanup();

    [[nodiscard]] Mode mode() const { return mode_; }
    [[nodiscard]] static const char *mode_name(Mode m);
    [[nodiscard]] const FieldExplorer &explorer() const { return explorer_; }
    [[nodiscard]] const std::optional<ExplorerTarget> &target() const { return target_; }
    [[nodiscard]] int legs_done() const { return legs_done_; }
    [[nodiscard]] int legs_failed() const { return legs_failed_; }

    static constexpr const char *kNodeName = "afford_openfield";

private:
    std::shared_ptr<DSR::DSRGraph> G_;
    std::unique_ptr<DSR::InnerEigenAPI> inner_eigen_;
    Config cfg_;
    FieldExplorer explorer_;
    rc::AffordanceManager aff_{kNodeName};
    Mode mode_ = Mode::Waiting;

    std::int64_t last_tick_ms_ = 0, last_obstacles_ms_ = 0, last_world_ms_ = 0, last_log_ms_ = 0;
    std::optional<ExplorerTarget> target_;
    std::int64_t armed_ms_ = 0, executing_since_ms_ = 0, progress_ms_ = 0;
    float best_dist_ = 0.f;
    std::future<std::optional<ExplorerTarget>> search_;
    int legs_done_ = 0, legs_failed_ = 0;

    void refresh_obstacles(const std::string &world_frame);
    void follow_protocol(std::int64_t now_ms, const Eigen::Vector2f &robot);
    void select_and_offer(std::int64_t now_ms, const Eigen::Vector2f &robot, std::uint64_t world_id);
    void publish_world_state(std::uint64_t world_id);
    void retire(const char *why, bool failed, std::int64_t now_ms);
    void release_and_retire(const char *why, std::int64_t now_ms);
};

}  // namespace rc::openfield
