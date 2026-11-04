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

// trunk_concept — the TRUNKS of the field as a semantic map, the way human_concept owns the people.
// The ZED's ADE20K 'tree' masks, taken in the band between the ground and the first branch, propose a trunk;
// the helios confirms it with returns in the same band and places it (trunk_map.h). Each trunk is a `plant`
// node named trunk_<id>: a world→trunk RT at the trunk axis and a disc of radius PlantRadiusM that the
// controller treats as forbidden space. A trunk is permanent until VERIFIED gone: it moves when it is seen
// displaced (a localisation correction) and is removed only when the camera looks where it should be, with
// no foliage (vegetation_concept's plant_* discs) or person in between, and does not find it.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include "trunk_map.h"

#include "../../common/agent_presence_coordinator/agent_presence_coordinator.h"
#include "../../common/concept_presence/concept_presence.h"
#include "../../common/lidar_ingestor/concept_lidar_ingestor.h"
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
		rc::trunk::TrunkParams map;
		float plant_radius_m = 0.5f;         // the forbidden disc around a trunk
		float trunk_height_m = 0.6f;         // published height_m (the trunk, not the canopy)
		float person_radius_m = 0.4f;        // a person as an occluder
		std::set<std::string> trunk_labels{"tree", "palm"};                          // ADE20K classes with a trunk
		std::set<std::string> vegetation_labels{"tree", "plant", "palm", "flower"};  // any of them = a real look
		int update_period_ms = 500;
		int lidar_every = 2;                 // use every Nth helios sweep (birth confirmation + position)
		int masks_stall_timeout_ms = 5000;
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

	// ── Inputs ───────────────────────────────────────────────────────────────────────────────────
	std::unique_ptr<rc::MaskIngestor> masks_;
	std::unique_ptr<DSR::InnerEigenAPI> inner_eigen_;
	std::unique_ptr<rc::ConceptLidarIngestor> lidar_;
	std::uint64_t sweeps_ = 0;
	std::string world_frame_;
	bool resolve_world_frame();
	bool ingest_masks(std::int64_t now_ms);     // one NEW masks frame -> the trunk map
	void ingest_lidar();
	std::vector<rc::trunk::Occluder> people() const;    // persons: occluders, and their points are not trunks
	std::vector<rc::trunk::Occluder> foliage() const;   // other plants' foliage (vegetation_concept): occluders
	std::int64_t masks_age_ms() const;

	// ── Estimation and publication ───────────────────────────────────────────────────────────────
	rc::trunk::TrunkMap map_;
	std::unique_ptr<DSR::RT_API> rt_api_;
	std::uint64_t world_node_id_ = 0;
	std::unordered_map<int, std::uint64_t> trunk_nodes_;   // trunk id -> DSR node id
	struct Published { Eigen::Vector2f c; float p_exists; };
	std::unordered_map<int, Published> published_;
	std::int64_t last_update_ms_ = 0;
	void publish(const rc::trunk::TrunkMap::Changes& changes);
	void insert_trunk_node(const rc::trunk::Trunk& t);
	void update_trunk_node(const rc::trunk::Trunk& t, std::uint64_t node_id);
	void delete_stale_trunk_nodes();

	// ── Diagnostics ──────────────────────────────────────────────────────────────────────────────
	std::uint64_t frames_ = 0, looks_ = 0, band_points_ = 0, born_ = 0, removed_ = 0;
	std::int64_t last_status_ms_ = 0;
	void log_status();
	FPSCounter fps_;

signals:
	void presenceReady();
	void presenceLost();
};

#endif
