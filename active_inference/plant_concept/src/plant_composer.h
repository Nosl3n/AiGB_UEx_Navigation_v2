#pragma once
// plant_concept/src/plant_composer.h — composes PLANT individuals from their perceived PARTS. Pure, no DSR.
//
// A plant is one individual with parts; the parts are ATTRIBUTES of its one `plant` node, not nodes of their own
// (active_inference/conceptos.md). Two parts are perceived today, by this agent's two sub-models:
//   * trunk  (trunk_map.h)      — a precise position (ZED-proposed, LiDAR-confirmed, ~cm), the plant's anchor;
//   * canopy (vegetation_map.h) — a disc of foliage seen from above, centred roughly over the trunk.
// A trunk and a canopy are the same plant when the trunk stands INSIDE the canopy's disc: the canopy is the
// space the plant's foliage occupies, so a trunk inside it is that plant's trunk. If one disc holds two trunks
// (or two discs one trunk), the pairing is one-to-one by distance — a plant has one trunk and one canopy.
//
// A plant may be INCOMPLETE: a trunk whose canopy has not been seen, or a canopy whose trunk has not (yet) been
// seen (foliage hides trunks). Both are plants. When the missing part appears, it JOINS the existing plant, so
// a plant keeps its id while it is completed — identity follows the parts, never the order they were seen in.
//
// Later parts (leaves, fruit, height, health) are further fields of Plant, filled by the agents that see them.

#include <Eigen/Dense>

#include <algorithm>
#include <cstdint>
#include <format>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace rc::plants
{

struct Part
{
    std::uint64_t key = 0;                          // the part's identity in its sub-model (never 0)
    Eigen::Vector2f p = Eigen::Vector2f::Zero();    // centre, world frame (m)
    float r = 0.0f;                                 // disc radius (canopy); 0 for a trunk
};

struct Plant
{
    int id = -1;
    std::uint64_t trunk = 0;    // trunk part key, 0 = not seen
    std::uint64_t canopy = 0;   // canopy part key, 0 = not seen
    Eigen::Vector2f c = Eigen::Vector2f::Zero();    // where the plant stands: its trunk, else its canopy centre
    [[nodiscard]] bool empty() const { return trunk == 0 and canopy == 0; }
};

class PlantComposer
{
public:
    struct Changes { std::vector<int> created, removed; };

    Changes update(const std::vector<Part>& trunks, const std::vector<Part>& canopies)
    {
        Changes ch;
        std::unordered_map<std::uint64_t, Eigen::Vector2f> trunk_at, canopy_at;
        for (const auto& t : trunks) trunk_at[t.key] = t.p;
        for (const auto& c : canopies) canopy_at[c.key] = c.p;

        // 1. Parts that no longer exist leave their plant.
        for (auto& [id, pl] : plants_)
        {
            if (pl.trunk != 0 and not trunk_at.contains(pl.trunk)) pl.trunk = 0;
            if (pl.canopy != 0 and not canopy_at.contains(pl.canopy)) pl.canopy = 0;
        }

        // 2. Pair trunks with canopies: trunk inside the disc, one-to-one, nearest first.
        std::vector<std::tuple<float, std::uint64_t, std::uint64_t>> cand;
        for (const auto& t : trunks)
            for (const auto& c : canopies)
                if (const float d = (t.p - c.p).norm(); d <= c.r)
                    cand.emplace_back(d, t.key, c.key);
        std::ranges::sort(cand);
        std::unordered_map<std::uint64_t, std::uint64_t> canopy_of;   // trunk -> canopy
        std::set<std::uint64_t> paired_canopy;
        for (const auto& [d, t, c] : cand)
            if (not canopy_of.contains(t) and not paired_canopy.contains(c))
            {
                canopy_of[t] = c;
                paired_canopy.insert(c);
            }

        // 3. Plants that have a trunk drop a canopy that is no longer paired with it.
        const auto plant_with = [this](auto member, std::uint64_t node) -> Plant* {
            for (auto& [id, pl] : plants_)
                if (pl.*member == node) return &pl;
            return nullptr;
        };
        for (auto& [id, pl] : plants_)
            if (pl.trunk != 0)
                if (const auto it = canopy_of.find(pl.trunk); it == canopy_of.end() or it->second != pl.canopy)
                    pl.canopy = 0;

        // 4. Each trunk joins its plant: the one it is already in; else the canopy-only plant of its canopy (that
        //    plant is completed and keeps its id); else a new plant. A canopy-only plant whose canopy now pairs
        //    with a trunk that already has a plant is absorbed by it.
        for (const auto& t : trunks)
        {
            const auto it = canopy_of.find(t.key);
            const std::uint64_t c = it == canopy_of.end() ? 0 : it->second;
            Plant* pl = plant_with(&Plant::trunk, t.key);
            if (c != 0)
                if (Plant* q = plant_with(&Plant::canopy, c); q != nullptr and q != pl and q->trunk == 0)
                {
                    if (pl == nullptr)
                        pl = q;            // the canopy-only plant gets its trunk
                    else
                        q->canopy = 0;     // absorbed: emptied, removed below
                }
            if (pl == nullptr)
            {
                pl = &plants_[next_id_];
                pl->id = next_id_++;
                ch.created.push_back(pl->id);
            }
            pl->trunk = t.key;
            pl->canopy = c;
        }

        // 5. A canopy in no plant is a plant whose trunk has not been seen.
        for (const auto& c : canopies)
            if (plant_with(&Plant::canopy, c.key) == nullptr)
            {
                Plant& pl = plants_[next_id_];
                pl.id = next_id_++;
                pl.canopy = c.key;
                ch.created.push_back(pl.id);
            }

        // 6. Where each plant stands; plants left with no part are gone.
        for (auto it = plants_.begin(); it != plants_.end();)
        {
            auto& pl = it->second;
            if (pl.empty())
            {
                if (std::ranges::find(ch.created, pl.id) == ch.created.end()) ch.removed.push_back(pl.id);
                else std::erase(ch.created, pl.id);
                it = plants_.erase(it);
                continue;
            }
            pl.c = pl.trunk != 0 ? trunk_at.at(pl.trunk) : canopy_at.at(pl.canopy);
            ++it;
        }
        return ch;
    }

    [[nodiscard]] const std::map<int, Plant>& plants() const { return plants_; }

    // Composition, completion of an incomplete plant (id kept), absorption, loss of a part.
    static bool self_test(std::string* report)
    {
        PlantComposer pc;
        bool ok = true;
        const auto fail = [&](const std::string& m) { ok = false; if (report) *report += "  FAIL " + m + "\n"; };
        // Canopy 10 seen first (no trunk yet), trunk 1 alone, canopy 11 + trunk 2 together.
        pc.update({{1, {0, 0}}}, {{10, {5.0f, 0.1f}, 0.5f}});
        if (pc.plants().size() != 2) fail("trunk-only + canopy-only = 2 plants");
        const int canopy_plant = std::ranges::find_if(pc.plants(), [](auto& kv) { return kv.second.canopy == 10; })->first;
        // The hidden trunk of canopy 10 appears: the same plant is completed.
        pc.update({{1, {0, 0}}, {2, {5.0f, 0.0f}}}, {{10, {5.0f, 0.1f}, 0.5f}});
        if (pc.plants().size() != 2) fail("completing a plant must not add one");
        if (const auto& p = pc.plants().at(canopy_plant); p.trunk != 2) fail("trunk 2 joins canopy 10's plant (id kept)");
        // Trunk 1's canopy appears: joins plant of trunk 1; no new plant.
        pc.update({{1, {0, 0}}, {2, {5.0f, 0.0f}}}, {{10, {5.0f, 0.1f}, 0.5f}, {11, {0.1f, 0.0f}, 0.5f}});
        if (pc.plants().size() != 2) fail("canopy of an existing trunk must join it");
        // Trunk 2 disappears: its plant keeps the canopy (still a plant).
        const auto ch = pc.update({{1, {0, 0}}}, {{10, {5.0f, 0.1f}, 0.5f}, {11, {0.1f, 0.0f}, 0.5f}});
        if (pc.plants().size() != 2 or not ch.removed.empty()) fail("a plant that lost its trunk keeps its canopy");
        // Everything gone: no plants.
        const auto gone = pc.update({}, {});
        if (not pc.plants().empty() or gone.removed.size() != 2) fail("no parts, no plants");
        if (report) *report = std::format("PlantComposer::self_test {}\n", ok ? "PASS" : "FAIL") + *report;
        return ok;
    }

private:
    std::map<int, Plant> plants_;
    int next_id_ = 0;
};

}  // namespace rc::plants
