/*
 *    Copyright (C) 2026 by RoboLab at the University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

// row_metaconcept — the ROWS of the field: field -> row -> plant. A row is a METACONCEPT (a group of individuals);
// the plants are concepts (plant_concept: one `plant` node each, its trunk and canopy as attributes). A row is not
// seen; it is inferred from where the plants' TRUNKS stand (common/vegetation/rows.h): trunks close to each other
// on a line form a row, a RECTANGLE that grows in length as trunks near its line join it. There can be several.
// Each row is a `metaconcept` node row_<id> with a world→row RT (rectangle centre, heading as yaw), width_m = its
// length along the line, depth_m = its width across it, and a `group_member` edge to every PLANT in it: those whose
// trunks make it, and the canopy-only plants (trunk not seen yet) whose canopy lies inside the rectangle.
// The controller marks the rows (not the individual trunks) as the space the robot may not enter.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include "../../common/agent_presence_coordinator/agent_presence_coordinator.h"
#include "../../common/concept_presence/concept_presence.h"
#include "../../common/vegetation/rows.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <ranges>
#include <utility>
#include <set>
#include <string>
#include <unordered_map>

#include <fps/fps.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	~SpecificWorker();

public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();

	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};

private:
	bool startup_check_flag = false;

	struct Config
	{
		rc::rows::RowParams rows;
		float row_height_m = 0.6f;     // published height_m (the trunks the row is made of)
		int update_period_ms = 1000;
	} cfg_;
	void load_config();

	// ── Presence (shared protocol, common/concept_presence) ──────────────────────────────────────
	AgentPresenceCoordinator presence_coordinator_;
	rc::presence::ConceptProtocol presence_protocol_;
	std::atomic<bool> shutting_down_{false};
	bool owned_nodes_cleaned_ = false;
	void waiting_enter()   { presence_coordinator_.waiting_enter(); }
	void waiting_loop()    { presence_coordinator_.waiting_loop(); }
	void operating_enter() { presence_coordinator_.operating_enter(); }
	void operating_loop()  { presence_coordinator_.operating_loop(); }
	void degraded_enter()  { presence_coordinator_.degraded_enter(); }
	void degraded_loop()   { presence_coordinator_.degraded_loop(); }
	void request_shutdown();
	void terminal_shutdown();

	// ── Rows ─────────────────────────────────────────────────────────────────────────────────────
	std::unique_ptr<DSR::RT_API> rt_api_;
	std::unique_ptr<DSR::InnerEigenAPI> inner_eigen_;
	std::uint64_t world_node_id_ = 0;
	std::string world_frame_;
	rc::rows::RowTracker tracker_;
	std::unordered_map<int, std::uint64_t> row_nodes_;               // row id -> DSR node id
	std::unordered_map<int, std::set<std::uint64_t>> row_edges_;     // row id -> trunk node ids it points to
	struct Published { Eigen::Vector2f c; float yaw, length, width; };
	std::unordered_map<int, Published> published_;
	std::unordered_map<int, std::uint64_t> trunk_node_of_;           // member id (plant obj_id) -> plant node id
	std::unordered_map<std::uint64_t, int> plant_obj_of_;            // plant node id -> plant obj_id
	std::vector<std::pair<int, Eigen::Vector2f>> canopy_only_;       // plants with no trunk yet: obj_id, canopy centre
	std::unordered_map<int, std::vector<int>> member_ids_;           // row id -> its member plants' obj_ids
	std::unordered_map<int, Eigen::Vector2f> trunk_pos_;             // member id -> world xy (trunk, or canopy centre)
	std::unordered_map<std::uint64_t, std::pair<float, float>> laid_out_;   // node id -> viewer slot it was given
	std::int64_t last_update_ms_ = 0;
	bool resolve_world_frame();
	std::vector<rc::rows::Member> read_trunks();
	void publish(const rc::rows::RowTracker::Changes& changes);
	void sync_members(int row_id, std::uint64_t row_node, const std::vector<int>& members);
	void delete_stale_row_nodes();
	void layout_graph(const DSR::Node& world);

	std::int64_t last_status_ms_ = 0;
	void log_status(std::size_t trunks);
	FPSCounter fps_;

signals:
	void presenceReady();
	void presenceLost();
};

#endif
