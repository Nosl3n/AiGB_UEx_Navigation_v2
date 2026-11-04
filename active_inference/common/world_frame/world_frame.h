#pragma once
/*
 * common/world_frame/world_frame.h — which node is "the world" in this graph: a `room` indoors, a `world`
 * in the open field (openfield_concept, agricultural worlds).
 *
 * ★2026-11-03: the open-field frame was RENAMED `field` -> `world` (type and name). `field` is now a METACONCEPT
 * there (field_<id>: the set of plots, active_inference/conceptos.md), not a frame — never resolve it here.
 *
 * WHY THIS EXISTS. Every consumer used to find the world frame with get_nodes_by_type("room") and, worse,
 * by the LITERAL name "room" in transform queries (inner_eigen->get_transformation_matrix("room", ...),
 * LidarPlaneReader::poll("room")). That hard-wired the whole fleet to one localizer. The open-field
 * localizer publishes a node of type `world` (registered in cortex's dsr_node_type.h) with the same contract
 * — delimiting_polygon_x/y, room_height, the robot->world RT edge — so a consumer only needs to ask which of
 * the two is present, and use that node's NAME as its frame.
 *
 * RESOLUTION ORDER: `world` first, then `room`. A graph never carries both in normal operation; if it does
 * (a stale node from a previous run), the open-field one wins because only openfield_concept publishes a
 * world, and it would not be running indoors.
 *
 * Header-only, no state. The lookups walk the node map, so CACHE the result per cycle in hot loops — the
 * name of the world frame does not change while its node exists.
 */
#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <dsr/api/dsr_api.h>

namespace rc::world
{

inline constexpr std::array<std::string_view, 2> kFrameTypes = {"world", "room"};

// True for a node type that IS a world frame (so it is never an obstacle, never drawn as furniture, ...).
inline bool is_frame_type(std::string_view type)
{
    return type == "world" or type == "room";
}

// The world-frame node, if any.
inline std::optional<DSR::Node> frame_node(DSR::DSRGraph &G)
{
    for (const auto t : kFrameTypes)
        if (auto v = G.get_nodes_by_type(std::string(t)); not v.empty())
            return v.front();
    return std::nullopt;
}

// The same, as the vector get_nodes_by_type() returns (drop-in for get_nodes_by_type("room")).
inline std::vector<DSR::Node> frame_nodes(DSR::DSRGraph &G)
{
    for (const auto t : kFrameTypes)
        if (auto v = G.get_nodes_by_type(std::string(t)); not v.empty())
            return v;
    return {};
}

// Its name, for transform queries. `fallback` when the graph has none yet ("room" keeps the indoor
// behaviour of every caller that used to hard-code it).
inline std::string frame_name(DSR::DSRGraph &G, std::string fallback = "room")
{
    if (const auto n = frame_node(G); n.has_value())
        return n->name();
    return fallback;
}

// Per-object cache for hot loops: re-resolves only when the cached node has disappeared (or was never
// found), so the common case is one id lookup instead of a walk over every node.
class FrameCache
{
public:
    // const: resolving the name is a cache refresh, not a state change of the owner (the members are mutable).
    const std::string &name(DSR::DSRGraph &G) const
    {
        if (id_ == 0 or not G.get_node(id_).has_value())
        {
            if (const auto n = frame_node(G); n.has_value()) { id_ = n->id(); name_ = n->name(); }
            else                                              { id_ = 0;       name_ = "room"; }
        }
        return name_;
    }
    std::uint64_t id(DSR::DSRGraph &G) const { name(G); return id_; }

private:
    mutable std::uint64_t id_ = 0;
    mutable std::string name_ = "room";
};

}  // namespace rc::world
