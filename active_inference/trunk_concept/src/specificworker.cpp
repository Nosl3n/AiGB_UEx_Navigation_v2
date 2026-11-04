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
#include "specificworker.h"

#include "../../common/agent_exit/terminal_exit.h"
#include "../../common/graph_provenance/creation_stamp.h"
#include "../../common/stream_gate/stream_gate.h"
#include "../../common/vegetation/graph_layout.h"
#include "../../common/vegetation/plant_discs.h"
#include "../../common/world_frame/world_frame.h"

#include <QCoreApplication>
#include <QDateTime>
#include <algorithm>
#include <cmath>
#include <format>
#include <print>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check)
    : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if (this->startup_check_flag)
	{
		this->startup_check();
		return;
	}

	load_config();

	// The map is pure Eigen: prove birth, the multimodal gate and the occlusion-aware removal on a synthetic
	// row before trusting it with the field.
	if (std::string report; rc::trunk::TrunkMap::self_test(&report))
		std::println("{}", report);
	else
		qCritical("%s", report.c_str());
	map_.set_params(cfg_.map);

	// Agent-presence state machine: Waiting -> Operating -> Degraded (same shape as every concept agent).
	const int period = configLoader.get<int>("Period.Compute");
	states["Waiting"] = std::make_unique<GRAFCETStep>("Waiting", period,
		std::bind(&SpecificWorker::waiting_loop, this), std::bind(&SpecificWorker::waiting_enter, this));
	states["Operating"] = std::make_unique<GRAFCETStep>("Operating", period,
		std::bind(&SpecificWorker::operating_loop, this), std::bind(&SpecificWorker::operating_enter, this));
	states["Degraded"] = std::make_unique<GRAFCETStep>("Degraded", period,
		std::bind(&SpecificWorker::degraded_loop, this), std::bind(&SpecificWorker::degraded_enter, this));

	states["Compute"]->addTransition(states["Compute"].get(), SIGNAL(entered()), states["Waiting"].get());
	states["Waiting"]->addTransition(this, SIGNAL(presenceReady()), states["Operating"].get());
	states["Operating"]->addTransition(this, SIGNAL(presenceLost()), states["Degraded"].get());
	states["Degraded"]->addTransition(states["Degraded"].get(), SIGNAL(entered()), states["Waiting"].get());

	statemachine.addState(states["Waiting"].get());
	statemachine.addState(states["Operating"].get());
	statemachine.addState(states["Degraded"].get());
	statemachine.setChildMode(QState::ExclusiveStates);
	statemachine.start();
	if (auto error = statemachine.errorString(); error.length() > 0)
	{
		qWarning() << error;
		throw error;
	}
}

SpecificWorker::~SpecificWorker()
{
	request_shutdown();
}

void SpecificWorker::load_config()
{
	const auto opt = [this](const std::string &key, auto &out)
	{
		try { out = configLoader.get<std::remove_reference_t<decltype(out)>>(key); } catch (...) {}
	};
	const auto optf = [&](const std::string &key, float &out)
	{
		double v = out;
		opt(key, v);
		out = static_cast<float>(v);
	};
	auto &m = cfg_.map;
	optf("Trunk.PlantRadiusM", cfg_.plant_radius_m);
	optf("Trunk.TrunkHeightM", cfg_.trunk_height_m);
	optf("Trunk.PersonRadiusM", cfg_.person_radius_m);
	optf("Trunk.BandZMinM", m.band_z_min);
	optf("Trunk.BandZMaxM", m.band_z_max);
	optf("Trunk.ClusterRadiusM", m.cluster_r);
	optf("Trunk.BirthMass", m.birth_mass);
	optf("Trunk.LlrHit", m.llr_hit);
	optf("Trunk.LlrMiss", m.llr_miss);
	optf("Trunk.LookPeriodMs", m.look_period_ms);
	opt("Trunk.LidarConfirmPoints", m.lidar_confirm_pts);
	optf("Trunk.AssocGateM", m.assoc_gate_m);
	optf("Trunk.MinSeparationM", m.min_separation_m);
	optf("Trunk.RemovalProb", m.removal_prob);
	optf("Trunk.ZedMaxRangeM", m.zed_max_range);
	optf("Trunk.ZedMinRangeM", m.zed_min_range);
	optf("Trunk.LidarConfirmDzM", m.lidar_confirm_dz);
	optf("Trunk.ZedHalfFovRad", m.zed_half_fov);
	opt("Ablation.LidarConfirm", m.lidar_confirm);
	opt("Ablation.VisibilityModel", m.visibility_model);
	if (not m.lidar_confirm or not m.visibility_model)
		std::println("[trunk_concept] ABLATION: LidarConfirm={} VisibilityModel={}", m.lidar_confirm, m.visibility_model);
	std::vector<std::string> labels;
	opt("Trunk.TrunkLabels", labels);
	if (not labels.empty()) cfg_.trunk_labels = {labels.begin(), labels.end()};
	labels.clear();
	opt("Trunk.VegetationLabels", labels);
	if (not labels.empty()) cfg_.vegetation_labels = {labels.begin(), labels.end()};
	opt("Trunk.UpdatePeriodMs", cfg_.update_period_ms);
	opt("Trunk.LidarEvery", cfg_.lidar_every);
	opt("Trunk.MasksStallTimeoutMs", cfg_.masks_stall_timeout_ms);

	std::println("[trunk] disc r={:.2f} m | band z [{:.2f}, {:.2f}] m | birth: {:.0f} cell-frames + {} LiDAR returns | "
	             "removal P(exists) < {:.2f}", cfg_.plant_radius_m, m.band_z_min, m.band_z_max, m.birth_mass,
	             m.lidar_confirm_pts, m.removal_prob);
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	if (not G)
	{
		qWarning() << "trunk_concept: DSR graph not available in initialize()";
		return;
	}
	G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_X_att, laser_Y_att, laser_Z_att>();

	rt_api_ = G->get_rt_api();
	inner_eigen_ = G->get_inner_eigen_api();
	masks_ = std::make_unique<rc::MaskIngestor>(G);
	// The helios subscriber comes up lazily inside pump(), once the node and its media descriptor exist.
	lidar_ = std::make_unique<rc::ConceptLidarIngestor>(G, inner_eigen_.get(),
	                                                    [] { return rc::LidarGates{.helios_precision = 1.0f}; });

	presence_protocol_.wire(
		presence_coordinator_, &statemachine, this, configLoader, G, static_cast<std::uint32_t>(agent_id),
		{
			.shutting_down      = [this] { return shutting_down_.load(); },
			// Polled input; the world frame first, or MaskIngestor drops every frame (see vegetation_concept).
			.pump_primary_input = [this] { if (masks_ and resolve_world_frame()) masks_->refresh(); },
			.primary_age_ms     = [this] { return masks_age_ms(); },
			.primary_live       = [this] { return rc::stream::live(masks_age_ms(), cfg_.masks_stall_timeout_ms); },
			.primary_stalled    = [this](std::int64_t *age)
			{
				const auto a = masks_age_ms();
				if (age) *age = a;
				return rc::stream::stalled(a, cfg_.masks_stall_timeout_ms, presence_protocol_.operating_since_ms(),
				                           QDateTime::currentMSecsSinceEpoch());
			},
			.emit_ready         = [this] { emit presenceReady(); },
			.emit_lost          = [this] { emit presenceLost(); },
			.compute            = [this] { compute(); },
			.terminal_shutdown  = [this] { terminal_shutdown(); },
			.on_first_operating = [this] { delete_stale_trunk_nodes(); },
			.on_optional_peer_lost  = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer lost:" << QString::fromStdString(name); },
			.on_optional_peer_ready = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer ready:" << QString::fromStdString(name); },
		});

	QObject::connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit,
	                 this, &SpecificWorker::terminal_shutdown, Qt::UniqueConnection);
}

bool SpecificWorker::resolve_world_frame()
{
	if (world_node_id_ != 0)
		return true;
	const auto frame = rc::world::frame_node(*G);
	if (not frame.has_value())
		return false;
	world_node_id_ = frame->id();
	world_frame_ = frame->name();
	masks_->set_frames("zed", world_frame_);
	std::println("[trunk] world frame '{}': trunks are placed in it", world_frame_);
	return true;
}

void SpecificWorker::compute()
{
	if (not resolve_world_frame())
		return;
	const auto now = QDateTime::currentMSecsSinceEpoch();
	ingest_masks(now);
	ingest_lidar();
	if (now - last_update_ms_ >= cfg_.update_period_ms)
	{
		last_update_ms_ = now;
		publish(map_.update(now));
	}
	log_status();
	fps_.print("[trunk_concept Compute]", 5000);
}

// ── ZED: the 'tree' masks' band points, the camera's pose, and whether this frame was a real look ──────
bool SpecificWorker::ingest_masks(std::int64_t now_ms)
{
	if (not masks_->refresh())
		return false;
	++frames_;
	const auto &pk = masks_->packet();
	std::vector<Eigen::Vector3f> band;
	bool looked = false;
	const auto &p = map_.params();
	for (const auto &s : pk.slices)
	{
		if (not s.is_semantic() or not s.may_fit_geometry())
			continue;
		// Semantic masks only exist on the frames the ADE20K model actually ran (it is decimated). A frame with
		// vegetation in it is a real look: a trunk expected in view and absent from it is a genuine miss.
		if (cfg_.vegetation_labels.contains(s.label))
			looked = true;
		if (not cfg_.trunk_labels.contains(s.label) or s.support_end > pk.support_points.size())
			continue;
		for (std::size_t i = s.support_begin; i < s.support_end; ++i)
			if (const auto &q = pk.support_points[i]; q.z() >= p.band_z_min and q.z() <= p.band_z_max)
				band.push_back(q);
	}
	if (not looked and band.empty())
		return true;
	const auto T = inner_eigen_->get_transformation_matrix(world_frame_, "zed", 0);
	if (not T.has_value())
		return true;
	const auto &M = T.value();
	const Eigen::Vector2f cam(static_cast<float>(M(0, 3)), static_cast<float>(M(1, 3)));
	// The ZED's optical axis is its +Y (x right, y forward/depth, z up — the retina deprojection convention).
	const float heading = static_cast<float>(std::atan2(M(1, 1), M(0, 1)));
	looks_ += looked ? 1 : 0;
	band_points_ += band.size();
	const auto ppl = people();
	auto occ = foliage();
	occ.insert(occ.end(), ppl.begin(), ppl.end());
	map_.add_zed_frame(band, cam, heading, occ, ppl, looked, now_ms);
	return true;
}

void SpecificWorker::ingest_lidar()
{
	if (not lidar_ or not lidar_->pump() or not lidar_->helios_fresh())
		return;
	if ((sweeps_++ % static_cast<std::uint64_t>(std::max(1, cfg_.lidar_every))) != 0)
		return;
	map_.add_lidar_sweep(lidar_->sweep_room(), lidar_->origin_room(), people());
}

// What can hide a trunk from the camera: other plants' FOLIAGE (vegetation_concept) ...
std::vector<rc::trunk::Occluder> SpecificWorker::foliage() const
{
	std::vector<rc::trunk::Occluder> occ;
	for (const auto &d : rc::veg::read_plant_discs(*G, *inner_eigen_, world_frame_, rc::veg::kFoliagePrefix))
		occ.push_back({d.c, d.r});
	return occ;
}

// ... and people, whose legs also cross the trunk band: their points are never a trunk's, for either sensor.
std::vector<rc::trunk::Occluder> SpecificWorker::people() const
{
	std::vector<rc::trunk::Occluder> occ;
	for (const auto &n : G->get_nodes_by_type("person"))
		if (const auto t = inner_eigen_->transform(world_frame_, n.name(), 0); t.has_value() and t->allFinite())
			occ.push_back({Eigen::Vector2f(static_cast<float>(t->x()), static_cast<float>(t->y())), cfg_.person_radius_m});
	return occ;
}

std::int64_t SpecificWorker::masks_age_ms() const
{
	return masks_ ? masks_->ms_since_last_frame() : -1;
}

// ── Graph ─────────────────────────────────────────────────────────────────────────────────────────
void SpecificWorker::publish(const rc::trunk::TrunkMap::Changes &changes)
{
	for (const int id : changes.removed)
		if (const auto it = trunk_nodes_.find(id); it != trunk_nodes_.end())
		{
			G->delete_node(it->second);
			std::println("[trunk] trunk_{} REMOVED: looked at, unoccluded, and not there", id);
			trunk_nodes_.erase(it);
			published_.erase(id);
			++removed_;
		}
	for (const auto &t : map_.trunks())
	{
		if (const auto it = trunk_nodes_.find(t.id); it == trunk_nodes_.end())
			insert_trunk_node(t);
		else
			update_trunk_node(t, it->second);
	}
}

void SpecificWorker::insert_trunk_node(const rc::trunk::Trunk &t)
{
	auto world = G->get_node(world_node_id_);
	if (not world.has_value())
		return;
	const std::string name = std::format("{}{}", rc::veg::kTrunkPrefix, t.id);
	DSR::Node node = DSR::Node::create<trunk_node_type>(name);
	const float diameter = 2.0f * cfg_.plant_radius_m;
	G->add_or_modify_attrib_local<obj_id_att>(node, t.id);
	G->add_or_modify_attrib_local<obj_width_m_att>(node, diameter);
	G->add_or_modify_attrib_local<width_m_att>(node, diameter);
	G->add_or_modify_attrib_local<depth_m_att>(node, diameter);
	G->add_or_modify_attrib_local<height_m_att>(node, cfg_.trunk_height_m);
	G->add_or_modify_attrib_local<obj_exist_prob_att>(node, t.existence.p_exists());
	{
		const float wx = G->get_attrib_by_name<pos_x_att>(world.value()).value_or(200.f);
		const float wy = G->get_attrib_by_name<pos_y_att>(world.value()).value_or(200.f);
		// Born in no plant: the viewer's "unassigned" strip; plant_concept puts it next to its plant.
		const auto [px, py] = rc::veg::layout::unassigned_part(wx, wy, t.id);
		G->add_or_modify_attrib_local<pos_x_att>(node, px);
		G->add_or_modify_attrib_local<pos_y_att>(node, py);
	}
	rc::provenance::stamp_creation(*G, node);
	const auto id = G->insert_node(node);
	if (not id.has_value())
	{
		qWarning() << "trunk_concept: failed to insert" << name.c_str();
		return;
	}
	trunk_nodes_[t.id] = id.value();
	++born_;
	std::println("[trunk] created '{}' at {} ({:.2f}, {:.2f}) — ZED candidate confirmed by {} LiDAR return(s)",
	             name, world_frame_, t.c.x(), t.c.y(), map_.params().lidar_confirm_pts);
	update_trunk_node(t, id.value());
}

void SpecificWorker::update_trunk_node(const rc::trunk::Trunk &t, std::uint64_t node_id)
{
	const float pe = t.existence.p_exists();
	const auto it = published_.find(t.id);
	const bool moved = it == published_.end() or (it->second.c - t.c).norm() >= 0.01f;
	const bool belief = it == published_.end() or std::abs(it->second.p_exists - pe) >= 0.05f;
	if (not moved and not belief)
		return;
	auto world = G->get_node(world_node_id_);
	auto node = G->get_node(node_id);
	if (not world.has_value() or not node.has_value())
		return;
	if (moved)
	{
		rt_api_->insert_or_assign_edge_RT(world.value(), node_id, {t.c.x(), t.c.y(), 0.0f}, {0.0f, 0.0f, 0.0f});
		if (auto edge = G->get_edge(world_node_id_, node_id, "RT"); edge.has_value())
		{
			std::vector<float> cov(36, 0.0f);
			cov[0] = cov[7] = std::max(t.var, 1e-4f);
			cov[14] = 0.01f;
			for (int d = 3; d < 6; ++d) cov[d * 6 + d] = 1.0f;
			G->add_or_modify_attrib_local<rt_covariance_att>(edge.value(), cov);
			G->insert_or_assign_edge(edge.value());
		}
	}
	if (belief)
	{
		G->add_or_modify_attrib_local<obj_exist_prob_att>(node.value(), pe);
		G->update_node(node.value());
	}
	published_[t.id] = {t.c, pe};
}

void SpecificWorker::delete_stale_trunk_nodes()
{
	int n = 0;
	for (const auto &node : G->get_nodes_by_type(std::string(rc::veg::kTrunkType)))
		if (node.name().starts_with(rc::veg::kTrunkPrefix)
		    and std::ranges::none_of(trunk_nodes_, [&](const auto &kv) { return kv.second == node.id(); }))
		{
			G->delete_node(node.id());
			++n;
		}
	if (n > 0)
		std::println("[trunk] removed {} stale trunk node(s) left by a previous run", n);
}

void SpecificWorker::log_status()
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_status_ms_ < 5000)
		return;
	last_status_ms_ = now;
	std::println("[trunk] status: masks frames {} (looks {}), band points {}, helios sweeps {}, candidates {}, "
	             "trunks {} (born {}, removed {})", frames_, looks_, band_points_, sweeps_, map_.pending(),
	             map_.trunks().size(), born_, removed_);
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────────────────────────
void SpecificWorker::request_shutdown()
{
	if (shutting_down_.exchange(true)) return;
	if (G) disconnect(G.get(), nullptr, this, nullptr);
	masks_.reset();
	lidar_.reset();
	if (not owned_nodes_cleaned_)
	{
		owned_nodes_cleaned_ = true;
		presence_coordinator_.cleanup_owned_nodes();
	}
}

void SpecificWorker::terminal_shutdown()
{
	static std::atomic<bool> terminating{false};
	if (terminating.exchange(true)) return;
	rc::agent::terminal_exit([this] { request_shutdown(); }, [this] { if (G) G->reset(); });
}

void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl;
}

void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
	std::string report;
	const bool ok = rc::trunk::TrunkMap::self_test(&report);
	std::println("{}", report);
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return ok ? 0 : 1;
}
