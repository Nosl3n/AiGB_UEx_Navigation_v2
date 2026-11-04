// openfield_exploration.cpp — see openfield_exploration.h.
#include "openfield_exploration.h"

#include "../../common/affordance_protocol/affordance_goal_parse.h"
#include "../../common/vegetation/plant_discs.h"

#include <chrono>
#include <set>
#include <cmath>
#include <print>

namespace rc::openfield
{

OpenFieldExploration::OpenFieldExploration(std::shared_ptr<DSR::DSRGraph> G, Config cfg)
    : G_(std::move(G)), cfg_(cfg), explorer_(cfg.explorer)
{
    inner_eigen_ = G_->get_inner_eigen_api();   // main thread: its ts==0 cache is invalidated by our event loop
}

OpenFieldExploration::~OpenFieldExploration()
{
    if (search_.valid()) search_.wait();
}

const char *OpenFieldExploration::mode_name(Mode m)
{
    switch (m)
    {
        case Mode::Inspecting: return "Inspecting";
        case Mode::Complete:   return "Complete";
        default:               return "Waiting";
    }
}

void OpenFieldExploration::tick(std::int64_t now_ms, const std::vector<Eigen::Vector2f> &polygon,
                                const std::optional<Eigen::Vector2f> &robot, std::uint64_t world_id,
                                const std::string &world_frame)
{
    if (not cfg_.enabled or now_ms - last_tick_ms_ < cfg_.period_ms)
        return;
    const float dt = last_tick_ms_ == 0 ? 0.f : std::min(1.f, (now_ms - last_tick_ms_) / 1000.f);
    last_tick_ms_ = now_ms;

    explorer_.set_polygon(polygon);
    if (not robot.has_value() or not explorer_.ready() or world_id == 0)
        return;
    if (now_ms - last_obstacles_ms_ >= cfg_.obstacles_period_ms)
    {
        last_obstacles_ms_ = now_ms;
        refresh_obstacles(world_frame);
    }

    // What the robot sees from where it is, every step (also while it drives between standpoints).
    explorer_.observe(*robot, dt);

    follow_protocol(now_ms, *robot);
    if (mode_ != Mode::Complete)
        select_and_offer(now_ms, *robot, world_id);
    if (now_ms - last_world_ms_ >= 1000)
    {
        last_world_ms_ = now_ms;
        publish_world_state(world_id);
    }
    if (now_ms - last_log_ms_ >= 5000)
    {
        last_log_ms_ = now_ms;
        std::println("[explore] {} · field entropy left {:.1f}% ({:.0f} of {:.0f} nats) · open plant questions {} "
                     "({:.2f} nats) · legs done {} failed {}{}",
                     mode_name(mode_), 100.f * explorer_.remaining_fraction(), explorer_.entropy(),
                     explorer_.initial_entropy(), explorer_.questions().size(), explorer_.concept_entropy(),
                     legs_done_, legs_failed_,
                     target_ ? std::format(" · target ({:.2f},{:.2f}) gain {:.1f} nats", target_->pos.x(),
                                           target_->pos.y(), target_->gain)
                             : std::string());
    }
}

// Rows (row_metaconcept rectangles) occlude and forbid standpoints; plants' trunk keep-outs forbid them.
void OpenFieldExploration::refresh_obstacles(const std::string &world_frame)
{
    std::vector<RowRect> rows;
    for (const auto &node : G_->get_nodes_by_type("metaconcept"))
    {
        if (not node.name().starts_with("row_"))
            continue;
        const float len = G_->get_attrib_by_name<width_m_att>(node).value_or(0.f);   // along the row
        const float wid = G_->get_attrib_by_name<depth_m_att>(node).value_or(0.f);   // across it
        const auto M = inner_eigen_->get_transformation_matrix(world_frame, node.name(), 0);
        if (not M.has_value() or not (len > 0.f and wid > 0.f))
            continue;
        const auto &m = M.value();
        rows.push_back(RowRect{{static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3))},
                               static_cast<float>(std::atan2(m(1, 0), m(0, 0))), len, wid});
    }
    std::vector<Disc> plants;
    std::vector<OpenPlant> open;
    for (const auto &v : rc::veg::read_plants(*G_, *inner_eigen_, world_frame))
    {
        if (v.has_trunk and v.keepout_r > 0.f)
            plants.push_back(Disc{v.origin, v.keepout_r});
        // An incomplete concept is an open question: a canopy without a trunk, or a trunk without a canopy.
        if (v.has_canopy and not v.has_trunk)
            open.push_back(OpenPlant{v.name, v.origin, true});
        else if (v.has_trunk and not v.has_canopy)
            open.push_back(OpenPlant{v.name, v.origin, false});
    }
    explorer_.set_obstacles(std::move(rows), std::move(plants));
    // Report questions that opened, and those that closed because the missing part was FOUND (the plant is no longer
    // listed as incomplete) — the answer the exploration went looking for.
    std::set<std::string> now;
    for (const auto &o : open) now.insert(o.name);
    for (const auto &o : open)
        if (not open_names_.contains(o.name))
            std::println("[explore] open question: {} has a {} but no {} — will look again", o.name,
                         o.missing_trunk ? "canopy" : "trunk", o.missing_trunk ? "trunk" : "canopy");
    for (const auto &n : open_names_)
        if (not now.contains(n))
            std::println("[explore] question closed: {} is complete now (its missing part was found)", n);
    open_names_ = std::move(now);
    explorer_.set_open_plants(open);
}

// The consumer's side of the protocol: did it finish, is it still on it, did it ever take it?
void OpenFieldExploration::follow_protocol(std::int64_t now_ms, const Eigen::Vector2f &robot)
{
    aff_.monitor_execution(G_);
    if (aff_.consume_completion_event())
    {
        if (not target_) return;
        const auto outcome = aff_.last_outcome();
        // A completion that observed nothing (unreachable, infeasible, outside the field) is the consumer saying
        // "I cannot stand there": suppress that standpoint. A satisfied one is simply done — what the robot saw
        // there is already in the belief, observed on the way and during the dwell.
        const bool failed = outcome != rc::affordance::Outcome::None
                            and not rc::affordance::observation_happened(outcome);
        retire(failed ? std::format("completed: {}", rc::affordance::to_string(outcome)).c_str() : "reached",
               failed, now_ms);
        return;
    }
    if (not target_) return;
    if (aff_.is_executing(G_))
    {
        const float d = (target_->pos - robot).norm();
        if (executing_since_ms_ == 0)
        {
            executing_since_ms_ = progress_ms_ = now_ms;
            best_dist_ = d;
        }
        if (d < best_dist_ - cfg_.stall_progress_m)
        {
            best_dist_ = d;
            progress_ms_ = now_ms;
        }
        // ★A CLAIM IS A LEASE (room_concept's rule 3): never wait on a consumer for ever. Two clocks: no progress
        // toward the standpoint (the consumer is holding with no route), and a backstop on the whole claim.
        if (now_ms - progress_ms_ > cfg_.stall_ms)
            release_and_retire(std::format("no progress for {:.0f} s (best {:.2f} m, now {:.2f} m)",
                                           (now_ms - progress_ms_) / 1000.0, best_dist_, d).c_str(), now_ms);
        else if (now_ms - executing_since_ms_ > cfg_.lease_ms)
            release_and_retire("claim held too long without completing (lease expired)", now_ms);
    }
    else
    {
        executing_since_ms_ = 0;
        if (now_ms - armed_ms_ > cfg_.offer_unclaimed_ms)
            retire("offered and never claimed", true, now_ms);
    }
}

// The consumer holds the claim but is not getting there: take it back (as room_concept does), so the node can be
// re-armed with another standpoint, and suppress this one.
void OpenFieldExploration::release_and_retire(const char *why, std::int64_t now_ms)
{
    const bool released = aff_.release_execution_claim(G_);
    retire(std::format("{}{}", why, released ? " — execution claim released" : "").c_str(), true, now_ms);
}

void OpenFieldExploration::retire(const char *why, bool failed, std::int64_t now_ms)
{
    if (target_)
    {
        if (failed)
        {
            explorer_.mark_failed(target_->pos, now_ms / 1000.0);
            ++legs_failed_;
        }
        else
            ++legs_done_;
        std::println("[explore] standpoint ({:.2f},{:.2f}) retired — {}", target_->pos.x(), target_->pos.y(), why);
    }
    target_.reset();
    armed_ms_ = executing_since_ms_ = progress_ms_ = 0;
}

void OpenFieldExploration::select_and_offer(std::int64_t now_ms, const Eigen::Vector2f &robot, std::uint64_t world_id)
{
    if (target_)
        return;
    if (not search_.valid())
    {
        // Search on a COPY: the belief keeps being updated here while the worker scores the field.
        search_ = std::async(std::launch::async, [ex = explorer_, robot, t = now_ms / 1000.0]
                             { return ex.select(robot, t); });
        return;
    }
    if (search_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return;
    const auto best = search_.get();
    if (not best)
    {
        if (mode_ != Mode::Complete)
        {
            std::println("[explore] INSPECTION COMPLETE — no standpoint left is worth its travel; field entropy "
                         "left {:.1f}% · legs done {} failed {}",
                         100.f * explorer_.remaining_fraction(), legs_done_, legs_failed_);
            // The plants that stay incomplete after being looked at again: reviewed, kept as they are in the map.
            for (const auto &q : explorer_.questions())
                std::println("[explore]   reviewed: {} still has no {} — P(it is there, unseen) = {:.3f} after {:.1f} "
                             "looks; kept as it is in the semantic map", q.plant.name,
                             q.plant.missing_trunk ? "trunk" : "canopy", q.b, q.looked_s);
        }
        mode_ = Mode::Complete;
        publish_world_state(world_id);
        return;
    }
    mode_ = Mode::Inspecting;
    if (aff_.publish_target(G_, world_id, best->pos.x(), best->pos.y(), best->yaw, best->gain))
    {
        target_ = best;
        armed_ms_ = now_ms;
        std::println("[explore] offering standpoint ({:.2f},{:.2f}) facing {:.0f}° · gain {:.2f} nats · "
                     "score {:.2f}", best->pos.x(), best->pos.y(), best->yaw * 180.0 / M_PI, best->gain, best->score);
    }
}

void OpenFieldExploration::publish_world_state(std::uint64_t world_id)
{
    auto world = G_->get_node(world_id);
    if (not world.has_value())
        return;
    const bool complete = mode_ == Mode::Complete;
    const float left = explorer_.remaining_fraction();
    const auto was = G_->get_attrib_by_name<model_stable_att>(world.value());
    const auto unc = G_->get_attrib_by_name<model_uncertainty_att>(world.value());
    if (was.has_value() and was.value() == complete and unc.has_value() and std::abs(unc.value() - left) < 1e-3f)
        return;
    G_->add_or_modify_attrib_local<model_stable_att>(world.value(), complete);
    G_->add_or_modify_attrib_local<model_uncertainty_att>(world.value(), left);
    G_->update_node(world.value());
}

void OpenFieldExploration::cleanup()
{
    if (const auto n = G_->get_node(kNodeName); n.has_value())
        G_->delete_node(n->id());
}

}  // namespace rc::openfield
