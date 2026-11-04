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

// plot_metaconcept — the PLOTS (parcelas) of the field: world -> field -> plot -> row -> plant. A plot is a
// METACONCEPT: a set of rows planted together — parallel, side by side and at the planting spacing, which is read
// from the rows themselves (common/vegetation/plots.h). A wider gap between neighbouring rows (a headland, a track,
// another plantation) starts another plot. Each plot is a `metaconcept` node plot_<id> with a world→plot RT
// (rectangle centre, heading = the rows' direction as yaw), width_m = its length along the rows, depth_m = its
// width across them, and a `group_member` edge to every row in it. Reads only row_metaconcept's rows.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include "../../common/agent_presence_coordinator/agent_presence_coordinator.h"
#include "../../common/concept_presence/concept_presence.h"
#include "../../common/vegetation/plots.h"

#include <atomic>
#include <cstdint>
#include <memory>
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
		rc::plots::PlotParams plots;
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

	// ── Plots ────────────────────────────────────────────────────────────────────────────────────
	std::unique_ptr<DSR::RT_API> rt_api_;
	std::unique_ptr<DSR::InnerEigenAPI> inner_eigen_;
	std::uint64_t world_node_id_ = 0;
	std::string world_frame_;
	rc::plots::PlotTracker tracker_;
	std::unordered_map<int, std::uint64_t> plot_nodes_;             // plot id -> DSR node id
	std::unordered_map<int, std::set<std::uint64_t>> plot_edges_;   // plot id -> row node ids it groups
	std::unordered_map<int, rc::plots::Rect> published_;
	std::unordered_map<int, std::uint64_t> row_node_of_;            // row obj_id -> row node id (this update)
	std::unordered_map<std::uint64_t, std::pair<float, float>> laid_out_;
	std::int64_t last_update_ms_ = 0;
	bool resolve_world_frame();
	std::vector<rc::plots::RowGeom> read_rows();
	void publish(const rc::plots::PlotTracker::Changes& changes);
	void sync_members(int plot_id, std::uint64_t plot_node, const std::vector<int>& rows);
	void layout_graph(const DSR::Node& world);
	void delete_stale_plot_nodes();

	std::int64_t last_status_ms_ = 0;
	void log_status(std::size_t rows);
	FPSCounter fps_;

signals:
	void presenceReady();
	void presenceLost();
};

#endif
