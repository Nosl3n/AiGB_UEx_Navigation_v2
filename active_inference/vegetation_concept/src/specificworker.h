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

// vegetation_concept — the PLANTS of the field as a concept, the way human_concept owns the people.
// retina segments vegetation with ADE20K (tree / plant / palm / flower) and publishes each region as a mask
// with its 3D points on the 'masks' node; this agent accumulates those points on a ground-plane evidence grid
// (vegetation_map.h) and owns one `plant` node per plant: a world→plant RT at the canopy centre and a fixed
// canopy radius. Consumers read them as discs (common/vegetation/plant_discs.h): the controller plans
// around them, residual_concept stops reporting their returns as unexplained, and human_concept knows that
// where a plant stands there is no person.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include "vegetation_map.h"

#include "../../common/agent_presence_coordinator/agent_presence_coordinator.h"
#include "../../common/concept_presence/concept_presence.h"
#include "../../common/mask_ingestor/mask_ingestor.h"

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

	// ── Config ───────────────────────────────────────────────────────────────────────────────────
	struct Config
	{
		rc::veg::VegetationParams map;
		std::set<std::string> labels{"tree", "plant", "palm", "flower"};   // ADE20K classes that are plants
		int update_period_ms = 500;      // plant extraction + graph publication rate
		int masks_stall_timeout_ms = 5000;   // primary-input gate: no masks frame for this long -> Waiting
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

	// ── Input: retina's vegetation masks ─────────────────────────────────────────────────────────
	std::unique_ptr<rc::MaskIngestor> masks_;
	std::string world_frame_;            // "field" outdoors (openfield_concept), "room" indoors
	bool ingest_masks();                 // one NEW masks frame -> the evidence grid; false if none
	bool resolve_world_frame();          // field/room node -> world_frame_ + the ingestor's target frame
	std::int64_t masks_age_ms() const;

	// ── Estimation and publication ───────────────────────────────────────────────────────────────
	rc::veg::VegetationMap map_;
	std::unique_ptr<DSR::RT_API> rt_api_;
	std::uint64_t world_node_id_ = 0;
	std::unordered_map<int, std::uint64_t> plant_nodes_;   // plant id -> DSR node id
	struct Published { Eigen::Vector2f c; float height; };
	std::unordered_map<int, Published> published_;
	void delete_plant_node(int plant_id);
	std::int64_t last_update_ms_ = 0;
	void publish_plants(const rc::veg::VegetationMap::Changes& changes);
	void insert_plant_node(const rc::veg::Plant& p);
	void update_plant_node(const rc::veg::Plant& p, std::uint64_t node_id);
	void delete_stale_plant_nodes();     // plant_* left by a previous run that did not exit gracefully

	// ── Diagnostics ──────────────────────────────────────────────────────────────────────────────
	std::uint64_t frames_ = 0, frames_with_veg_ = 0, points_ = 0;
	std::int64_t last_status_ms_ = 0;
	void log_status();
	FPSCounter fps_;

signals:
	void presenceReady();
	void presenceLost();
};

#endif
