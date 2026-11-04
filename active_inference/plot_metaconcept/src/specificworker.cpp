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
	if (std::string report; rc::plots::PlotTracker::self_test(&report))
		std::println("{}", report);
	else
		qCritical("%s", report.c_str());
	tracker_.set_params(cfg_.plots);

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
	optf("Plots.ParallelTolRad", cfg_.plots.parallel_tol_rad);
	optf("Plots.SpacingFactor", cfg_.plots.spacing_factor);
	opt("Plots.UpdatePeriodMs", cfg_.update_period_ms);
	std::println("[plots] rows parallel within {:.0f} deg, side by side and at most {:.1f} x the planting spacing form a plot",
	             cfg_.plots.parallel_tol_rad * 57.2958f, cfg_.plots.spacing_factor);
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	if (not G)
	{
		qWarning() << "plot_metaconcept: DSR graph not available in initialize()";
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
			.on_first_operating = [this] { delete_stale_plot_nodes(); },
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
	const auto rows = read_rows();
	publish(tracker_.update(rows));
	log_status(rows.size());
	fps_.print("[plot_metaconcept Compute]", 10000);
}

// row_metaconcept's rows: world→row RT (centre, heading as yaw), width_m along, depth_m across.
std::vector<rc::plots::RowGeom> SpecificWorker::read_rows()
{
	std::vector<rc::plots::RowGeom> out;
	row_node_of_.clear();
	for (const auto &n : G->get_nodes_by_type("metaconcept"))
	{
		if (not n.name().starts_with("row_"))
			continue;
		const auto id = G->get_attrib_by_name<obj_id_att>(n);
		const float len = G->get_attrib_by_name<width_m_att>(n).value_or(0.f);
		const float wid = G->get_attrib_by_name<depth_m_att>(n).value_or(0.f);
		const auto M = inner_eigen_->get_transformation_matrix(world_frame_, n.name(), 0);
		if (not id.has_value() or not M.has_value() or not (len > 0.f and wid > 0.f))
			continue;
		const auto &m = M.value();
		out.push_back({id.value(), Eigen::Vector2f(static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3))),
		               static_cast<float>(std::atan2(m(1, 0), m(0, 0))), len, wid});
		row_node_of_[id.value()] = n.id();
	}
	return out;
}

void SpecificWorker::publish(const rc::plots::PlotTracker::Changes &changes)
{
	for (const int id : changes.removed)
		if (const auto it = plot_nodes_.find(id); it != plot_nodes_.end())
		{
			G->delete_node(it->second);    // its group_member edges go with it
			std::println("[plots] plot_{} removed (merged into an older plot, or none of its rows is left)", id);
			plot_nodes_.erase(it);
			plot_edges_.erase(id);
			published_.erase(id);
		}
	auto world = G->get_node(world_node_id_);
	if (not world.has_value())
		return;
	for (const auto &[pid, pl] : tracker_.plots())
	{
		auto it = plot_nodes_.find(pid);
		if (it == plot_nodes_.end())
		{
			const std::string name = std::format("plot_{}", pid);
			DSR::Node node = DSR::Node::create<metaconcept_node_type>(name);
			G->add_or_modify_attrib_local<obj_id_att>(node, pid);
			G->add_or_modify_attrib_local<width_m_att>(node, pl.rect.length);   // ALONG the rows
			G->add_or_modify_attrib_local<depth_m_att>(node, pl.rect.width);    // across them
			const float wx = G->get_attrib_by_name<pos_x_att>(world.value()).value_or(200.f);
			const float wy = G->get_attrib_by_name<pos_y_att>(world.value()).value_or(200.f);
			G->add_or_modify_attrib_local<pos_x_att>(node, wx + rc::veg::layout::kRowX0);
			G->add_or_modify_attrib_local<pos_y_att>(node, wy + rc::veg::layout::kPlotY);
			rc::provenance::stamp_creation(*G, node);
			const auto id = G->insert_node(node);
			if (not id.has_value())
				continue;
			it = plot_nodes_.emplace(pid, id.value()).first;
			std::println("[plots] created 'plot_{}': {} rows, {:.1f} x {:.1f} m (planting spacing {:.2f} m)", pid,
			             pl.rows.size(), pl.rect.length, pl.rect.width, tracker_.spacing());
		}
		sync_members(pid, it->second, pl.rows);
		if (const auto pr = published_.find(pid); pr != published_.end()
		    and (pr->second.c - pl.rect.c).norm() < 0.02f and std::abs(pr->second.length - pl.rect.length) < 0.05f
		    and std::abs(pr->second.width - pl.rect.width) < 0.05f and std::abs(pr->second.yaw - pl.rect.yaw) < 0.01f)
			continue;
		rt_api_->insert_or_assign_edge_RT(world.value(), it->second, {pl.rect.c.x(), pl.rect.c.y(), 0.0f},
		                                  {0.0f, 0.0f, pl.rect.yaw});
		if (auto node = G->get_node(it->second); node.has_value())
		{
			G->add_or_modify_attrib_local<width_m_att>(node.value(), pl.rect.length);
			G->add_or_modify_attrib_local<depth_m_att>(node.value(), pl.rect.width);
			G->update_node(node.value());
		}
		if (const auto pr = published_.find(pid); pr != published_.end() and pl.rect.width > pr->second.width + 0.05f)
			std::println("[plots] plot_{} grew to {} rows ({:.1f} x {:.1f} m)", pid, pl.rows.size(), pl.rect.length,
			             pl.rect.width);
		published_[pid] = pl.rect;
	}
	layout_graph(world.value());
}

// plot --group_member--> row, for exactly the plot's current rows.
void SpecificWorker::sync_members(int plot_id, std::uint64_t plot_node, const std::vector<int> &rows)
{
	std::set<std::uint64_t> want;
	for (const int r : rows)
		if (const auto it = row_node_of_.find(r); it != row_node_of_.end())
			want.insert(it->second);
	auto &have = plot_edges_[plot_id];
	for (const auto to : have)
		if (not want.contains(to))
			G->delete_edge(plot_node, to, "group_member");
	for (const auto to : want)
		if (not have.contains(to))
			G->insert_or_assign_edge(DSR::Edge::create<group_member_edge_type>(plot_node, to));
	have = std::move(want);
}

// Viewer only (common/vegetation/graph_layout.h): each plot on the plot layer, above the middle of its rows (whose
// columns row_metaconcept places).
void SpecificWorker::layout_graph(const DSR::Node &world)
{
	const float wy = G->get_attrib_by_name<pos_y_att>(world).value_or(200.f);
	for (const auto &[pid, pl] : tracker_.plots())
	{
		const auto it = plot_nodes_.find(pid);
		if (it == plot_nodes_.end())
			continue;
		float sx = 0.f;
		int n = 0;
		for (const int r : pl.rows)
			if (const auto rn = row_node_of_.find(r); rn != row_node_of_.end())
				if (const auto node = G->get_node(rn->second); node.has_value())
				{
					sx += G->get_attrib_by_name<pos_x_att>(node.value()).value_or(0.f);
					++n;
				}
		if (n == 0)
			continue;
		const std::pair<float, float> xy{sx / static_cast<float>(n), wy + rc::veg::layout::kPlotY};
		if (const auto lo = laid_out_.find(it->second); lo != laid_out_.end() and lo->second == xy)
			continue;
		if (auto node = G->get_node(it->second); node.has_value())
		{
			G->add_or_modify_attrib_local<pos_x_att>(node.value(), xy.first);
			G->add_or_modify_attrib_local<pos_y_att>(node.value(), xy.second);
			if (G->update_node(node.value()))
				laid_out_[it->second] = xy;
		}
	}
	std::erase_if(laid_out_, [&](const auto &kv) {
		return std::ranges::none_of(plot_nodes_, [&](const auto &pn) { return pn.second == kv.first; }); });
}

void SpecificWorker::delete_stale_plot_nodes()
{
	int n = 0;
	for (const auto &node : G->get_nodes_by_type("metaconcept"))
		if (node.name().starts_with("plot_")
		    and std::ranges::none_of(plot_nodes_, [&](const auto &kv) { return kv.second == node.id(); }))
		{
			G->delete_node(node.id());
			++n;
		}
	if (n > 0)
		std::println("[plots] removed {} stale plot node(s) left by a previous run", n);
}

void SpecificWorker::log_status(std::size_t rows)
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_status_ms_ < 5000)
		return;
	last_status_ms_ = now;
	std::string sizes;
	for (const auto &[pid, pl] : tracker_.plots())
		sizes += std::format("{}plot_{}:{}", sizes.empty() ? "" : " ", pid, pl.rows.size());
	std::println("[plots] status: {} rows -> {} plots ({}), planting spacing {:.2f} m", rows, tracker_.plots().size(),
	             sizes, tracker_.spacing());
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
	const bool ok = rc::plots::PlotTracker::self_test(&report);
	std::println("{}", report);
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return ok ? 0 : 1;
}
