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
	if (std::string report; rc::rows::RowTracker::self_test(&report))
		std::println("{}", report);
	else
		qCritical("%s", report.c_str());
	tracker_.set_params(cfg_.rows);

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
	auto &r = cfg_.rows;
	optf("Rows.NeighbourMaxM", r.neighbour_max_m);
	optf("Rows.RowGapM", r.row_gap_m);
	optf("Rows.AlongGapM", r.along_gap_m);
	opt("Rows.MinMembers", r.min_members);
	optf("Rows.HalfWidthM", r.half_width_m);
	optf("Rows.EndMarginM", r.end_margin_m);
	optf("Rows.RowHeightM", cfg_.row_height_m);
	opt("Rows.UpdatePeriodMs", cfg_.update_period_ms);
	std::println("[rows] >= {} trunks on a line make a row; rows apart > {:.1f} m, split at gaps > {:.1f} m; "
	             "rectangle {:.2f} m beyond the line and the end trunks", r.min_members, r.row_gap_m, r.along_gap_m,
	             r.half_width_m);
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	if (not G)
	{
		qWarning() << "row_metaconcept: DSR graph not available in initialize()";
		return;
	}
	G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_X_att, laser_Y_att, laser_Z_att>();
	rt_api_ = G->get_rt_api();
	inner_eigen_ = G->get_inner_eigen_api();

	// A metaconcept reads only the graph: there is no sensor stream to gate on, so the input is always "fresh".
	presence_protocol_.wire(
		presence_coordinator_, &statemachine, this, configLoader, G, static_cast<std::uint32_t>(agent_id),
		{
			.shutting_down      = [this] { return shutting_down_.load(); },
			.pump_primary_input = [] {},
			.primary_age_ms     = [] { return std::int64_t{0}; },
			.primary_live       = [] { return true; },
			.primary_stalled    = [](std::int64_t *age) { if (age) *age = 0; return false; },
			.emit_ready         = [this] { emit presenceReady(); },
			.emit_lost          = [this] { emit presenceLost(); },
			.compute            = [this] { compute(); },
			.terminal_shutdown  = [this] { terminal_shutdown(); },
			.on_first_operating = [this] { delete_stale_row_nodes(); },
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
	return true;
}

void SpecificWorker::compute()
{
	if (not resolve_world_frame())
		return;
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_update_ms_ < cfg_.update_period_ms)
		return;
	last_update_ms_ = now;
	const auto trunks = read_trunks();
	publish(tracker_.update(trunks));
	log_status(trunks.size());
	fps_.print("[row_metaconcept Compute]", 10000);
}

// The plants plant_concept has placed (one `plant` node per individual, its parts as attributes). A row is a line of
// TRUNKS (the precise part), so the row inference is fed the plants that have one: member id = the plant's obj_id,
// at its world->plant RT (= its trunk axis). Plants with only a canopy (trunk not seen yet) are kept aside in
// canopy_only_: they join a row whose rectangle holds their canopy (publish()).
std::vector<rc::rows::Member> SpecificWorker::read_trunks()
{
	std::vector<rc::rows::Member> out;
	trunk_node_of_.clear();
	trunk_pos_.clear();
	plant_obj_of_.clear();
	canopy_only_.clear();
	for (const auto &n : G->get_nodes_by_type(std::string(rc::veg::kPlantType)))
	{
		if (not n.name().starts_with(rc::veg::kPlantPrefix))
			continue;
		const auto id = G->get_attrib_by_name<obj_id_att>(n);
		const auto t = inner_eigen_->transform(world_frame_, n.name(), 0);
		if (not id.has_value() or not t.has_value() or not t->allFinite())
			continue;
		const Eigen::Vector2f origin(static_cast<float>(t->x()), static_cast<float>(t->y()));
		trunk_node_of_[id.value()] = n.id();
		plant_obj_of_[n.id()] = id.value();
		if (G->get_attrib_by_name<plant_has_trunk_att>(n).value_or(false))
		{
			out.push_back({id.value(), origin});
			trunk_pos_[id.value()] = origin;
		}
		else if (G->get_attrib_by_name<plant_has_canopy_att>(n).value_or(false))
		{
			Eigen::Vector2f off = Eigen::Vector2f::Zero();
			if (const auto o = G->get_attrib_by_name<plant_canopy_offset_att>(n); o.has_value() and o->get().size() >= 2)
				off = {o->get()[0], o->get()[1]};
			canopy_only_.emplace_back(id.value(), origin + off);
			trunk_pos_[id.value()] = origin + off;   // its place along the row, for the viewer's order
		}
	}
	return out;
}

void SpecificWorker::publish(const rc::rows::RowTracker::Changes &changes)
{
	for (const int id : changes.removed)
		if (const auto it = row_nodes_.find(id); it != row_nodes_.end())
		{
			G->delete_node(it->second);    // its group_member edges go with it
			std::println("[rows] row_{} removed (merged into an older row, or no longer {} trunks on a line)", id,
			             cfg_.rows.min_members);
			row_nodes_.erase(it);
			row_edges_.erase(id);
			published_.erase(id);
		}
	auto world = G->get_node(world_node_id_);
	if (not world.has_value())
		return;
	member_ids_.clear();
	for (const auto &r : tracker_.rows())
	{
		auto it = row_nodes_.find(r.id);
		if (it == row_nodes_.end())
		{
			const std::string name = std::format("row_{}", r.id);
			DSR::Node node = DSR::Node::create<metaconcept_node_type>(name);
			G->add_or_modify_attrib_local<obj_id_att>(node, r.id);
			G->add_or_modify_attrib_local<width_m_att>(node, r.length);    // ALONG the row
			G->add_or_modify_attrib_local<depth_m_att>(node, r.width);     // across it
			G->add_or_modify_attrib_local<height_m_att>(node, cfg_.row_height_m);
			const float wx = G->get_attrib_by_name<pos_x_att>(world.value()).value_or(200.f);
			const float wy = G->get_attrib_by_name<pos_y_att>(world.value()).value_or(200.f);
			// Provisional column at the end of the row layer; layout_graph() below puts it in its rank.
			const auto [px, py] = rc::veg::layout::row(wx, wy, static_cast<int>(row_nodes_.size()));
			G->add_or_modify_attrib_local<pos_x_att>(node, px);
			G->add_or_modify_attrib_local<pos_y_att>(node, py);
			rc::provenance::stamp_creation(*G, node);
			const auto id = G->insert_node(node);
			if (not id.has_value())
				continue;
			it = row_nodes_.emplace(r.id, id.value()).first;
			std::println("[rows] created 'row_{}': {} trunks, {:.1f} x {:.1f} m, heading {:.1f} deg", r.id,
			             r.members.size(), r.length, r.width, r.yaw * 57.2958f);
		}
		// The row's plants: those whose trunks make it, plus the canopy-only plants (trunk not seen yet) whose canopy
		// lies inside its rectangle — the row is where that plant stands.
		std::vector<int> members = r.members;
		const Eigen::Vector2f along(std::cos(r.yaw), std::sin(r.yaw)), across(-along.y(), along.x());
		for (const auto &[pid, p] : canopy_only_)
			if (std::abs(along.dot(p - r.c)) <= 0.5f * r.length and std::abs(across.dot(p - r.c)) <= 0.5f * r.width)
				members.push_back(pid);
		member_ids_[r.id] = members;
		sync_members(r.id, it->second, members);
		// Rewrite the rectangle only when it changed: it grows as trunks join; centres settle a few cm.
		if (const auto pr = published_.find(r.id); pr != published_.end()
		    and (pr->second.c - r.c).norm() < 0.02f and std::abs(pr->second.length - r.length) < 0.05f
		    and std::abs(pr->second.width - r.width) < 0.05f and std::abs(pr->second.yaw - r.yaw) < 0.01f)
			continue;
		rt_api_->insert_or_assign_edge_RT(world.value(), it->second, {r.c.x(), r.c.y(), 0.0f}, {0.0f, 0.0f, r.yaw});
		if (auto node = G->get_node(it->second); node.has_value())
		{
			G->add_or_modify_attrib_local<width_m_att>(node.value(), r.length);
			G->add_or_modify_attrib_local<depth_m_att>(node.value(), r.width);
			G->update_node(node.value());
		}
		if (const auto pr = published_.find(r.id); pr != published_.end() and r.length > pr->second.length + 0.05f)
			std::println("[rows] row_{} grew {:.1f} -> {:.1f} m ({} trunks)", r.id, pr->second.length, r.length,
			             r.members.size());
		published_[r.id] = {r.c, r.yaw, r.length, r.width};
	}
	layout_graph(world.value());
}

// The viewer layers (common/vegetation/graph_layout.h): field -> rows (ranked across the rows) -> each row's
// plants in a column under it (ranked along the row, by their trunks); plant_concept keeps each plant's parts
// next to it. A plant that left every row goes back to the unassigned zone. pos_x/pos_y are cosmetic, so a node is rewritten only when its slot changed (no 1 Hz CRDT traffic).
void SpecificWorker::layout_graph(const DSR::Node &world)
{
	const float wx = G->get_attrib_by_name<pos_x_att>(world).value_or(200.f);
	const float wy = G->get_attrib_by_name<pos_y_att>(world).value_or(200.f);
	const auto &rows = tracker_.rows();
	if (rows.empty() and laid_out_.empty())
		return;

	// Rank rows by their offset across the crop direction (that of the first row; rows are parallel).
	const float yaw0 = rows.empty() ? 0.f : rows.front().yaw;
	const Eigen::Vector2f along(std::cos(yaw0), std::sin(yaw0));
	const Eigen::Vector2f across(-along.y(), along.x());
	std::vector<const rc::rows::Row *> order;
	for (const auto &r : rows)
		order.push_back(&r);
	// across points 90 deg left of the row heading; for N-S rows (yaw ~ +90 deg) that is WEST, so rank by -offset.
	std::ranges::sort(order, std::ranges::greater{}, [&](const rc::rows::Row *r) { return across.dot(r->c); });

	std::unordered_map<std::uint64_t, std::pair<float, float>> want;
	for (const auto [k, r] : std::views::enumerate(order))
	{
		if (const auto it = row_nodes_.find(r->id); it != row_nodes_.end())
			want[it->second] = rc::veg::layout::row(wx, wy, static_cast<int>(k));
		std::vector<int> members;
		for (const int m : member_ids_[r->id])
			if (trunk_pos_.contains(m) and trunk_node_of_.contains(m))
				members.push_back(m);
		std::ranges::sort(members, {}, [&](int m) { return along.dot(trunk_pos_.at(m)); });
		for (const auto [j, m] : std::views::enumerate(members))
			want[trunk_node_of_.at(m)] = rc::veg::layout::row_member(wx, wy, static_cast<int>(k), static_cast<int>(j));
	}
	// Plants this agent placed under a row that are in none now: back to the unassigned zone, where plant_concept
	// puts a new plant, and then they are plant_concept's layout again (forgotten here).
	std::set<std::uint64_t> released;
	for (const auto &[node_id, plant_id] : plant_obj_of_)
		if (not want.contains(node_id) and laid_out_.contains(node_id))
		{
			want[node_id] = rc::veg::layout::unassigned_plant(wx, wy, plant_id);
			released.insert(node_id);
		}

	for (const auto &[node_id, xy] : want)
	{
		if (const auto it = laid_out_.find(node_id); it != laid_out_.end() and it->second == xy)
			continue;
		auto node = G->get_node(node_id);
		if (not node.has_value())
			continue;
		G->add_or_modify_attrib_local<pos_x_att>(node.value(), xy.first);
		G->add_or_modify_attrib_local<pos_y_att>(node.value(), xy.second);
		if (G->update_node(node.value()))
			laid_out_[node_id] = xy;
	}
	// Forget released trunks and nodes that no longer exist (removed rows, trunks trunk_concept deleted).
	std::erase_if(laid_out_, [&](const auto &kv) { return released.contains(kv.first) or not want.contains(kv.first); });
}

// row --group_member--> plant, for exactly the row's current member plants.
void SpecificWorker::sync_members(int row_id, std::uint64_t row_node, const std::vector<int> &members)
{
	std::set<std::uint64_t> want;
	for (const int m : members)
		if (const auto it = trunk_node_of_.find(m); it != trunk_node_of_.end())
			want.insert(it->second);
	auto &have = row_edges_[row_id];
	for (const auto to : have)
		if (not want.contains(to))
			G->delete_edge(row_node, to, "group_member");
	for (const auto to : want)
		if (not have.contains(to))
			G->insert_or_assign_edge(DSR::Edge::create<group_member_edge_type>(row_node, to));
	have = std::move(want);
}

void SpecificWorker::delete_stale_row_nodes()
{
	int n = 0;
	for (const auto &node : G->get_nodes_by_type("metaconcept"))
		if (node.name().starts_with("row_")
		    and std::ranges::none_of(row_nodes_, [&](const auto &kv) { return kv.second == node.id(); }))
		{
			G->delete_node(node.id());
			++n;
		}
	if (n > 0)
		std::println("[rows] removed {} stale row node(s) left by a previous run", n);
}

void SpecificWorker::log_status(std::size_t trunks)
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_status_ms_ < 5000)
		return;
	last_status_ms_ = now;
	std::size_t in_rows = 0;
	for (const auto &r : tracker_.rows())
		in_rows += r.members.size();
	std::println("[rows] status: {} trunks -> {} rows ({} trunks in rows, {} not yet in one)", trunks,
	             tracker_.rows().size(), in_rows, trunks - in_rows);
}

void SpecificWorker::request_shutdown()
{
	if (shutting_down_.exchange(true)) return;
	if (G) disconnect(G.get(), nullptr, this, nullptr);
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
	const bool ok = rc::rows::RowTracker::self_test(&report);
	std::println("{}", report);
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return ok ? 0 : 1;
}
