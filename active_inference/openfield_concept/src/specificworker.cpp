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

#include "../../common/world_frame/world_frame.h"

#include "geo_utm.h"

#include "../../common/agent_exit/terminal_exit.h"
#include "../../common/diag_log/rotating_csv.h"
#include "../../common/stream_gate/stream_gate.h"

#include <QCoreApplication>
#include <QDateTime>
#include <algorithm>
#include <charconv>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <locale>
#include <print>

using rc::openfield::wrap_pi;

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

	// The EKF is pure Eigen: prove it on a synthetic drive before trusting it with the robot.
	if (std::string report; rc::openfield::OpenFieldEkf::self_test(&report))
		std::println("{}", report);
	else
		qCritical("%s", report.c_str());
	ekf_.set_params(cfg_.ekf);

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
	auto &e = cfg_.ekf;
	opt("Ekf.AdvSigma0", e.adv_sigma0);
	opt("Ekf.AdvSlip", e.adv_slip);
	opt("Ekf.SideSigma0", e.side_sigma0);
	opt("Ekf.SideSlip", e.side_slip);
	opt("Ekf.GyroSigma0", e.gyro_sigma0);
	opt("Ekf.GyroScale", e.gyro_scale);
	opt("Ekf.GpsSigmaM", e.gps_sigma_m);
	opt("Ekf.YawSigmaRad", e.yaw_sigma_rad);
	std::vector<double> lever;
	opt("Ekf.GpsLever", lever);
	if (lever.size() == 2) e.gps_lever = {lever[0], lever[1]};

	opt("Field.PolygonX", cfg_.poly_x);
	opt("Field.PolygonY", cfg_.poly_y);
	opt("Field.PolygonLat", cfg_.poly_lat);
	opt("Field.PolygonLon", cfg_.poly_lon);
	opt("Field.Source", cfg_.field_source);
	opt("Field.ObjectTypes", cfg_.object_types);
	opt("Field.TerrainTypes", cfg_.terrain_types);
	opt("Field.MarginX", cfg_.margin_x);
	opt("Field.MarginY", cfg_.margin_y);
	opt("Field.StartMargin", cfg_.start_margin);
	double h = cfg_.world_height;
	opt("Field.RoomHeight", h);
	cfg_.world_height = static_cast<float>(h);

	opt("Imu.UseYaw", cfg_.use_imu_yaw);
	opt("Imu.YawUpdatePeriodMs", cfg_.yaw_update_period_ms);
	opt("Imu.StallTimeoutMs", cfg_.imu_stall_timeout_ms);
	opt("Imu.Node", cfg_.imu_node);
	opt("Gps.PollPeriodMs", cfg_.gps_poll_period_ms);
	opt("Diag.CsvPath", cfg_.csv_path);
	opt("Diag.ExperimentCsv", cfg_.exp_csv_path);
	opt("Gps.StatusFile", cfg_.gps_status_file);
	opt("Experiment.ShadowFilters", cfg_.shadow_filters);
	{
		std::string src = "fixed", lik = "gauss";
		opt("Ekf.GpsSigmaSource", src);
		opt("Ekf.GpsLikelihood", lik);
		opt("Ekf.StudentNu", cfg_.gps_model.nu);
		opt("Ekf.LagCompensation", cfg_.gps_model.lag_comp);
		cfg_.gps_model.receiver_sigma = src == "receiver";
		cfg_.gps_model.student = lik == "student";
	}
	if (cfg_.shadow_filters)
		shadows_ = rc::openfield::make_shadows(e, cfg_.gps_model.nu);
	opt("Viewer.Enabled", cfg_.viewer);
	opt("Viewer.World3D", cfg_.world3d);
	{
		auto &x = cfg_.exploration;
		auto &ep = x.explorer;
		const auto optf = [&](const std::string &key, float &out) { double v = out; opt(key, v); out = static_cast<float>(v); };
		opt("Exploration.Enabled", x.enabled);
		opt("Exploration.PeriodMs", x.period_ms);
		int ms = static_cast<int>(x.offer_unclaimed_ms); opt("Exploration.OfferUnclaimedMs", ms); x.offer_unclaimed_ms = ms;
		ms = static_cast<int>(x.lease_ms); opt("Exploration.LeaseMs", ms); x.lease_ms = ms;
		ms = static_cast<int>(x.stall_ms); opt("Exploration.StallMs", ms); x.stall_ms = ms;
		optf("Exploration.StallProgressM", x.stall_progress_m);
		optf("Exploration.RelocateRadiusM", x.relocate_radius_m);
		optf("Exploration.CellM", ep.cell_m);
		optf("Exploration.SensorRangeM", ep.sensor_range_m);
		optf("Exploration.CanopyTransmittance", ep.canopy_transmittance);
		optf("Exploration.DetectRate", ep.detect_rate);
		optf("Exploration.DwellS", ep.dwell_s);
		optf("Exploration.PriorPlantP", ep.prior_plant_p);
		optf("Exploration.TravelCostNatsPerM", ep.travel_cost);
		optf("Exploration.BoundaryClearanceM", ep.boundary_clearance);
		optf("Exploration.RowClearanceM", ep.row_clearance);
		optf("Exploration.PlantClearanceM", ep.plant_clearance);
		optf("Exploration.FailedSuppressS", ep.failed_suppress_s);
		optf("Exploration.FailedRadiusM", ep.failed_radius_m);
		optf("Exploration.PartPrior", ep.part_prior);
		optf("Exploration.PartRangeM", ep.part_range_m);
		optf("Exploration.ConceptWeight", ep.concept_weight);
		optf("Exploration.OpenPlantKeepoutM", ep.open_plant_keepout_m);
		std::println("[openfield] inspection mode {}: cells {} m, sensor range {} m, canopy transmittance {}, travel cost "
		             "{} nats/m", x.enabled ? "ON" : "OFF", ep.cell_m, ep.sensor_range_m, ep.canopy_transmittance,
		             ep.travel_cost);
	}

	std::println("[openfield] EKF: gps sigma {} m, yaw sigma {} rad, lever ({}, {}) m | IMU yaw {} every {} ms",
	             e.gps_sigma_m, e.yaw_sigma_rad, e.gps_lever.x(), e.gps_lever.y(),
	             cfg_.use_imu_yaw ? "ON" : "OFF", cfg_.yaw_update_period_ms);
	std::println("[openfield] GPS fusion: sigma {}, likelihood {}{}, latency compensation {} | receiver status {} | "
	             "shadow filters {}",
	             cfg_.gps_model.receiver_sigma ? "from the receiver" : "fixed", cfg_.gps_model.student ? "Student-t nu=" : "Gaussian",
	             cfg_.gps_model.student ? std::format("{}", cfg_.gps_model.nu) : std::string(),
	             cfg_.gps_model.lag_comp ? "ON" : "OFF", cfg_.gps_status_file.empty() ? "none" : cfg_.gps_status_file,
	             shadows_.empty() ? "OFF" : "A/B/C/D");
}

void SpecificWorker::initialize()
{
	GenericWorker::initialize();
	if (not G)
	{
		qWarning() << "openfield_concept: DSR graph not available in initialize()";
		return;
	}
	// This agent reads nothing heavy from the graph: drop the payload blobs peers publish.
	G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_X_att, laser_Y_att, laser_Z_att>();

	rt_api_ = G->get_rt_api();
	rt_api_->HISTORY_SIZE = 25;   // same ring depth as room_concept: consumers interpolate into it
	scene_graph_ = std::make_unique<rc::openfield::OpenFieldSceneGraph>(G, rt_api_.get());

	std::filesystem::create_directories(std::filesystem::path(cfg_.csv_path).parent_path());
	if (rc::diag::open_rotating(csv_, cfg_.csv_path,
	        "wall_ms,x,y,theta,sx,sy,stheta,gps_fixes,gt_x,gt_y,gt_theta,err_xy,err_theta\n"))
		csv_.imbue(std::locale::classic());
	if (not cfg_.exp_csv_path.empty())
	{
		std::filesystem::create_directories(std::filesystem::path(cfg_.exp_csv_path).parent_path());
		exp_csv_.open(cfg_.exp_csv_path, std::ios::out | std::ios::trunc);
		exp_csv_.imbue(std::locale::classic());
		exp_csv_ << "wall_ms,clock_ms,gt_stamp,gt_x,gt_y,gt_th";
		std::vector<std::string> names{"main"};
		for (const auto &sh : shadows_) names.push_back(sh.name);
		for (const auto &n : names)
			for (const auto *c : {"x", "y", "th", "pxx", "pxy", "pxt", "pyy", "pyt", "ptt"})
				exp_csv_ << ',' << n << '_' << c;
		exp_csv_ << ",fix_seq,fix_valid,fix_type,fix_sigma,fix_nvis,fix_err,fix_mp,fix_meas_ms,fix_deliver_ms"
		            ",main_d2,main_w,B_d2,B_w,B_lag_ms,gps_enabled,fix_x,fix_y\n";
	}

	presence_protocol_.wire(
		presence_coordinator_, &statemachine, this, configLoader, G, static_cast<std::uint32_t>(agent_id),
		{
			.shutting_down      = [this] { return shutting_down_.load(); },
			// The IMU is POLLED (no reader thread), so it must be pumped while Waiting or its age never
			// advances and the agent could never be admitted.
			.pump_primary_input = [this] { pump_imu(); },
			.primary_age_ms     = [this] { return imu_age_ms(); },
			.primary_live       = [this] { return rc::stream::live(imu_age_ms(), cfg_.imu_stall_timeout_ms); },
			.primary_stalled    = [this](std::int64_t *age)
			{
				const auto a = imu_age_ms();
				if (age) *age = a;
				return rc::stream::stalled(a, cfg_.imu_stall_timeout_ms, presence_protocol_.operating_since_ms(),
				                           QDateTime::currentMSecsSinceEpoch());
			},
			.emit_ready         = [this] { emit presenceReady(); },
			.emit_lost          = [this] { emit presenceLost(); },
			.compute            = [this] { compute(); },
			.terminal_shutdown  = [this] { terminal_shutdown(); },
			.on_first_operating = {},
			.on_optional_peer_lost  = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer lost:" << QString::fromStdString(name); },
			.on_optional_peer_ready = [](const std::string &name, std::uint32_t)
			                          { qInfo() << "[Presence] optional peer ready:" << QString::fromStdString(name); },
		});

	QObject::connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit,
	                 this, &SpecificWorker::terminal_shutdown, Qt::UniqueConnection);

	start_ms_ = QDateTime::currentMSecsSinceEpoch();
	if (cfg_.viewer)
	{
		viewer_ = std::make_unique<rc::openfield::OpenFieldViewer>();
		// The switches only change which measurements the EKF fuses; prediction never stops. With the GPS off
		// the position is pure dead reckoning and its covariance grows — which is what the window shows.
		connect(viewer_.get(), &rc::openfield::OpenFieldViewer::gpsToggled, this, [this](bool on)
		{
			gps_enabled_ = on;
			gps_off_since_ms_ = on ? 0 : QDateTime::currentMSecsSinceEpoch();
			std::println("[openfield] GPS {} by the user", on ? "ON" : "OFF — dead reckoning only");
		});
		connect(viewer_.get(), &rc::openfield::OpenFieldViewer::yawToggled, this, [this](bool on)
		{
			yaw_enabled_ = on;
			std::println("[openfield] IMU absolute yaw {} by the user", on ? "ON" : "OFF — gyro integration only");
		});
		viewer_->show();
	}
	exploration_ = std::make_unique<rc::openfield::OpenFieldExploration>(G, cfg_.exploration);
	if (cfg_.world3d)
	{
		world3d_eigen_ = G->get_inner_eigen_api();
		world3d_ = std::make_unique<rc::viewers::GLWorld3DViewer>();
		world3d_->setWindowTitle("openfield_concept — 3D semantic map");
		world3d_->show();
	}
}

// ── One Operating tick ────────────────────────────────────────────────────────────────────────────
// Order: odometry first (the latest body velocity), then the IMU frames (each one propagates the filter
// on its own clock), then the GPS (corrects the propagated state), then publish.
void SpecificWorker::compute()
{
	read_odometry();
	pump_imu();
	poll_gps();

	if (ekf_.initialized())
	{
		if (field_polygon_.empty())
			resolve_field_polygon();
		if (not field_polygon_.empty() and scene_graph_->ensure_world(field_polygon_, cfg_.world_height))
		{
			publish_pose();
			if ((compute_ticks_ % 50) == 0)        // ~1 Hz: the field's plots as its members
				scene_graph_->group_plots();
			const auto &x = ekf_.state();
			exploration_->tick(QDateTime::currentMSecsSinceEpoch(), field_polygon_,
			                   Eigen::Vector2f(static_cast<float>(x(0)), static_cast<float>(x(1))),
			                   scene_graph_->world_id(), rc::world::frame_name(*G, "field"));
		}
	}
	log_status();
	write_experiment_row();
	if (viewer_ and (compute_ticks_ % 5) == 0)   // ~10 Hz at Period.Compute = 20 ms
		push_viewer();
	if (world3d_ and (compute_ticks_ % 25) == 0)  // ~2 Hz: plants and rows change slowly; the robot is smooth enough
		push_world3d();
	++compute_ticks_;
	fps_.print("[openfield_concept Compute]");
}

// ── IMU: propagate on every frame, correct the heading at a bounded rate ──────────────────────────
void SpecificWorker::pump_imu()
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	// ── FOLLOW THE DESCRIPTOR, NOT THE FIRST PRODUCER ────────────────────────────────────────────────────
	// The IMU's producer can change under us: launched before the sensor layer, we subscribe to the plane
	// robot_concept BRIDGES; when imu_dds comes up, robot_concept stops bridging and relays imu_dds's
	// descriptor instead (different QoS), and a reader built for the old one never matches the new writer.
	// Seen 2026-10-01: initialised, then "primary input STALLED" for good. So a stream silent for 1 s drops
	// the subscriber and the throttled path below rebuilds it from whatever the graph advertises NOW.
	if (imu_sub_ and last_imu_recv_ms_ > 0 and now - last_imu_recv_ms_ > 1000 and now - last_imu_try_ms_ > 1000)
	{
		std::println("[openfield] IMU silent for {} ms — re-subscribing from the current descriptor", now - last_imu_recv_ms_);
		imu_sub_.reset();
	}
	if (not imu_sub_)
	{
		// The producer advertises its descriptor during ITS startup; "not yet" is not "never".
		if (now - last_imu_try_ms_ < 1000 or not G) return;
		last_imu_try_ms_ = now;
		imu_sub_ = rc::media::make_imu_subscriber_from_graph(*G, cfg_.imu_node, "imu");
		if (not imu_sub_) return;
		std::println("[openfield] IMU media subscriber up on node '{}'", cfg_.imu_node);
	}

	imu_sub_->poll([this](const rc::media::ImuFrame &f, std::int64_t)
	{
		// Integrate on the clock the RATES are measured in: a simulated gyro reports rad per SIMULATION
		// second, so wall-clock dt would over- or under-count by the sim/wall ratio. 0 = not simulated.
		const auto clock = static_cast<std::int64_t>(f.sim_stamp_ms() > 0 ? f.sim_stamp_ms() : f.stamp_ms());
		if (clock <= last_imu_clock_ms_) return;        // duplicate / reordered sample
		last_imu_recv_ms_ = QDateTime::currentMSecsSinceEpoch();
		if (last_imu_clock_ms_ > 0 and ekf_.initialized())
		{
			const double dt = 1e-3 * static_cast<double>(clock - last_imu_clock_ms_);
			ekf_.predict(dt, odom_adv_, odom_side_, f.gyro_z());
			for (auto &sh : shadows_)
			{
				sh.ekf.predict(dt, odom_adv_, odom_side_, f.gyro_z());
				if (sh.use_yaw and cfg_.use_imu_yaw and clock - sh.last_yaw_clock_ms >= cfg_.yaw_update_period_ms)
				{
					sh.ekf.correct_yaw(f.yaw());
					sh.last_yaw_clock_ms = clock;
				}
				sh.antenna.push(clock, sh.ekf.predicted_antenna());
				sh.record(static_cast<std::int64_t>(f.stamp_ms()));
			}
		}
		if (ekf_.initialized())
			main_antenna_.push(clock, ekf_.predicted_antenna());
		last_imu_clock_ms_ = clock;
		last_imu_stamp_ms_ = static_cast<std::int64_t>(f.stamp_ms());
		last_gyro_z_ = f.gyro_z();
		last_imu_yaw_ = f.yaw();

		// The absolute yaw is a strong measurement; applying it at 120 Hz would treat 120 correlated
		// samples as independent and make the filter far more certain than the sensor is.
		if (cfg_.use_imu_yaw and yaw_enabled_ and ekf_.initialized()
		    and clock - last_yaw_update_clock_ms_ >= cfg_.yaw_update_period_ms)
		{
			ekf_.correct_yaw(f.yaw());
			last_yaw_update_clock_ms_ = clock;
		}
	});
}

std::int64_t SpecificWorker::imu_age_ms() const
{
	return last_imu_recv_ms_ < 0 ? -1 : QDateTime::currentMSecsSinceEpoch() - last_imu_recv_ms_;
}

// ── Wheel odometry: the robot node's robot_current_* attributes (robot_concept, from the base) ────
void SpecificWorker::read_odometry()
{
	const auto rid = scene_graph_ ? scene_graph_->robot_id() : 0;
	if (rid == 0) return;
	const auto node = G->get_node(rid);
	if (not node.has_value()) return;
	const auto ts = G->get_attrib_by_name<robot_current_speed_timestamp_att>(node.value());
	if (not ts.has_value() or static_cast<std::uint64_t>(ts.value()) == last_odom_ts_) return;
	last_odom_ts_ = static_cast<std::uint64_t>(ts.value());
	odom_adv_  = G->get_attrib_by_name<robot_current_advance_speed_att>(node.value()).value_or(0.f);
	odom_side_ = G->get_attrib_by_name<robot_current_side_speed_att>(node.value()).value_or(0.f);
	odom_rot_  = G->get_attrib_by_name<robot_current_angular_speed_att>(node.value()).value_or(0.f);
	// {adv, side, rot}; a negative entry is the producer saying "unknown" — fall back to the model's floor.
	if (const auto v = G->get_attrib_by_name<robot_current_speed_variance_att>(node.value());
	    v.has_value() and v->get().size() >= 3)
		for (int i = 0; i < 3; ++i) odom_var_(i) = v->get()[i] >= 0.f ? v->get()[i] : 0.f;
}

// ── GPS: antenna position in the field frame, from the bridge ─────────────────────────────────────
void SpecificWorker::poll_gps()
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_gps_poll_ms_ < cfg_.gps_poll_period_ms) return;
	last_gps_poll_ms_ = now;

	rc::openfield::GpsFix fix;
	if (not cfg_.gps_status_file.empty())
	{
		// The receiver's own report (bridge [GnssDegradation] StatusFile): fix + sigma + type + measurement time.
		if (not read_status_fix(fix)) return;
		last_fix_ = fix;
		if (not fix.valid) return;                        // the receiver says: no fix now
	}
	else
	{
		float x = 0.f, y = 0.f, z = 0.f;
		bool ok = false;
		try { ok = gps_proxy->getPos(x, y, z); }
		catch (const Ice::Exception &e)
		{
			static std::int64_t last_warn = 0;
			if (now - last_warn > 5000) { last_warn = now; qWarning() << "[openfield] GPS unreachable:" << e.what(); }
			return;
		}
		if (not ok) return;
		fix.xy = {x, y};
		// The receiver refreshes at its own rate (bridge GPS.Period); a repeated value is the SAME fix read
		// again, and folding it in twice would double-count its information.
		if (last_gps_xy_.has_value() and (fix.xy - *last_gps_xy_).norm() == 0.0) return;
		last_fix_ = fix;
	}
	const Eigen::Vector2d xy = fix.xy;
	last_gps_xy_ = xy;
	++gps_fixes_;
	last_gps_new_ms_ = now;
	if (not gps_enabled_) return;                        // switched off in the window: read, never fused

	if (not ekf_.initialized())
	{
		if (not last_imu_yaw_.has_value()) return;       // need a heading too
		ekf_.initialize(xy, *last_imu_yaw_);
		last_yaw_update_clock_ms_ = last_imu_clock_ms_;
		for (auto &sh : shadows_)
		{
			sh.ekf.initialize(xy, *last_imu_yaw_);
			sh.last_yaw_clock_ms = last_imu_clock_ms_;
		}
		std::println("[openfield] initialized from GPS ({:.3f}, {:.3f}) and IMU yaw {:.3f} rad{}", xy.x(), xy.y(),
		             *last_imu_yaw_, shadows_.empty() ? "" : " (and the A/B/C/D shadow filters)");
		return;
	}
	last_gps_innovation_m_ = (xy - ekf_.predicted_antenna()).norm();
	main_diag_ = rc::openfield::fuse_gps(ekf_, cfg_.gps_model, fix, ekf_.params().gps_sigma_m, main_antenna_,
	                                     last_imu_clock_ms_);
	for (auto &sh : shadows_)
		if (sh.use_gps)
			sh.diag = rc::openfield::fuse_gps(sh.ekf, sh.gps, fix, sh.ekf.params().gps_sigma_m, sh.antenna,
			                                  last_imu_clock_ms_);
	++gps_fused_;
}

// One line "seq meas_ms deliver_ms valid x y z sigma fix_type n_vis n_partial n_above mp err" (bridge,
// write_gnss_status). Parsed with from_chars: the C locale here may use a decimal COMMA (CLAUDE.md).
bool SpecificWorker::read_status_fix(rc::openfield::GpsFix &fix)
{
	std::ifstream in(cfg_.gps_status_file);
	std::string line;
	if (not in or not std::getline(in, line)) return false;
	std::vector<double> v;
	const char *p = line.data(), *end = line.data() + line.size();
	while (p < end)
	{
		while (p < end and *p == ' ') ++p;
		if (p >= end) break;
		double d = 0.0;
		const auto r = std::from_chars(p, end, d);
		if (r.ec != std::errc()) return false;
		v.push_back(d);
		p = r.ptr;
	}
	if (v.size() < 14) return false;
	const auto seq = static_cast<std::uint64_t>(v[0]);
	if (seq == last_status_seq_) return false;
	last_status_seq_ = seq;
	fix.seq = seq;
	fix.meas_ms = static_cast<std::int64_t>(v[1]);
	fix.deliver_ms = static_cast<std::int64_t>(v[2]);
	fix.valid = v[3] > 0.5;
	fix.xy = {v[4], v[5]};
	fix.sigma = v[7];
	fix.fix_type = static_cast<int>(v[8]);
	fix.n_vis = static_cast<int>(v[9]);
	fix.mp = v[12];
	fix.err = v[13];
	fix.has_status = true;
	return true;
}

// ── E1 log: every filter at the ground truth's own stamp, 10 Hz ─────────────────────────────────────────────────
void SpecificWorker::write_experiment_row()
{
	if (not exp_csv_.is_open() or not ekf_.initialized() or not scene_graph_) return;
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_exp_row_ms_ < 100) return;
	last_exp_row_ms_ = now;
	const auto rid = scene_graph_->robot_id();
	if (rid == 0) return;
	const auto n = G->get_node(rid);
	if (not n.has_value()) return;
	const auto ax = G->get_attrib_by_name<robot_gt_x_att>(n.value());
	const auto ay = G->get_attrib_by_name<robot_gt_y_att>(n.value());
	const auto aa = G->get_attrib_by_name<robot_gt_angle_att>(n.value());
	const auto ats = G->get_attrib_by_name<robot_gt_timestamp_att>(n.value());
	if (not (ax.has_value() and ay.has_value() and aa.has_value() and ats.has_value())) return;
	const auto t = static_cast<std::int64_t>(ats.value());
	const auto row = [&](const Eigen::Vector3d &x, const Eigen::Matrix3d &P)
	{
		exp_csv_ << ',' << x(0) << ',' << x(1) << ',' << x(2) << ',' << P(0, 0) << ',' << P(0, 1) << ',' << P(0, 2)
		         << ',' << P(1, 1) << ',' << P(1, 2) << ',' << P(2, 2);
	};
	exp_csv_ << now << ',' << last_imu_clock_ms_ << ',' << t << ',' << ax.value() << ',' << ay.value() << ','
	         << wrap_pi(-(aa.value() + M_PI_2));
	const auto main_at = pose_at(t);
	row(main_at.value_or(ekf_.state()), ekf_.covariance());
	for (const auto &sh : shadows_)
	{
		const auto at = sh.at(t);
		if (at.has_value()) row(at->first, at->second);
		else row(sh.ekf.state(), sh.ekf.covariance());
	}
	const auto &f = last_fix_;
	const double b_d2 = shadows_.size() > 1 ? shadows_[1].diag.d2 : 0.0;
	const double b_w = shadows_.size() > 1 ? shadows_[1].diag.w : 1.0;
	const double b_lag = shadows_.size() > 1 ? shadows_[1].diag.lag_ms : 0.0;
	exp_csv_ << ',' << f.seq << ',' << (f.valid ? 1 : 0) << ',' << f.fix_type << ',' << f.sigma << ',' << f.n_vis << ','
	         << f.err << ',' << f.mp << ',' << f.meas_ms << ',' << f.deliver_ms << ',' << main_diag_.d2 << ','
	         << main_diag_.w << ',' << b_d2 << ',' << b_w << ',' << b_lag << ',' << (gps_enabled_ ? 1 : 0) << ',' << f.xy.x() << ','
	         << f.xy.y() << '\n';
}

// ── The field boundary: local metres, or GPS coordinates converted like the bridge does ───────────
bool SpecificWorker::resolve_field_polygon()
{
	std::vector<Eigen::Vector2f> poly;
	if (cfg_.field_source == "webots")
	{
		if (auto p = field_polygon_from_webots(); p.has_value())
		{
			field_polygon_ = std::move(*p);
			return true;
		}
		qWarning() << "[openfield] Field.Source = \"webots\" failed: falling back to the polygon in the config";
		cfg_.field_source = "config";   // once: the fallback is not retried every tick
	}
	if (cfg_.poly_x.size() >= 3 and cfg_.poly_x.size() == cfg_.poly_y.size())
	{
		for (std::size_t i = 0; i < cfg_.poly_x.size(); ++i)
			poly.emplace_back(static_cast<float>(cfg_.poly_x[i]), static_cast<float>(cfg_.poly_y[i]));
		std::println("[openfield] field polygon: {} vertices in local metres (Field.PolygonX/Y)", poly.size());
	}
	else if (cfg_.poly_lat.size() >= 3 and cfg_.poly_lat.size() == cfg_.poly_lon.size())
	{
		// The local origin in UTM = current UTM fix minus the current local fix (same receiver snapshot).
		int zone = 0; std::string band; double e = 0, n = 0;
		float x = 0, y = 0, z = 0;
		try
		{
			if (not gps_proxy->getUTMData(zone, band, e, n) or not gps_proxy->getPos(x, y, z))
				return false;
		}
		catch (const Ice::Exception &) { return false; }
		const double e0 = e - x, n0 = n - y;
		for (std::size_t i = 0; i < cfg_.poly_lat.size(); ++i)
		{
			const auto u = rc::openfield::wgs84_to_utm(cfg_.poly_lat[i], cfg_.poly_lon[i], zone);
			poly.emplace_back(static_cast<float>(u.east - e0), static_cast<float>(u.north - n0));
		}
		std::println("[openfield] field polygon: {} GPS vertices -> local metres (UTM zone {}{})", poly.size(), zone, band);
	}
	else
	{
		// No boundary configured: a 50 m square around where the robot started, and say so loudly —
		// the planner will treat everything outside it as occupied.
		const auto &s = ekf_.state();
		for (const auto &[dx, dy] : {std::pair{-25., -25.}, {25., -25.}, {25., 25.}, {-25., 25.}})
			poly.emplace_back(static_cast<float>(s(0) + dx), static_cast<float>(s(1) + dy));
		qWarning() << "[openfield] no Field.PolygonX/Y nor Field.PolygonLat/Lon in config: using a 50 m square"
		              " around the start pose";
	}
	for (const auto &v : poly) std::println("    ({:.3f}, {:.3f})", v.x(), v.y());
	field_polygon_ = std::move(poly);
	return true;
}

// ── Field.Source = "webots": the working area of the loaded world ─────────────────────────────────
// World frame == our local frame (origin = WorldInfo.gpsReference, x east, y north), so the bridge's
// extents are used as they come. Rectangle = bbox(ObjectTypes) + (MarginX, MarginY), grown to keep
// StartMargin around the start pose, clipped to bbox(TerrainTypes). No objects -> the whole terrain.
std::optional<std::vector<Eigen::Vector2f>> SpecificWorker::field_polygon_from_webots()
{
	if (not webots2robocomp_proxy)
	{
		qWarning() << "[openfield] Field.Source = \"webots\" but no Proxies.Webots2Robocomp";
		return std::nullopt;
	}
	RoboCompWebots2Robocomp::WorldExtents ext;
	try { ext = webots2robocomp_proxy->ice_invocationTimeout(3000)->getWorldExtents(); }
	catch (const Ice::Exception &e)
	{
		qWarning() << "[openfield] getWorldExtents() failed:" << e.what();
		return std::nullopt;
	}
	struct Box { float x0 = 0, y0 = 0, x1 = 0, y1 = 0; bool valid = false;
	             void add(float a0, float b0, float a1, float b1)
	             { if (not valid) { x0 = a0; y0 = b0; x1 = a1; y1 = b1; valid = true; return; }
	               x0 = std::min(x0, a0); y0 = std::min(y0, b0); x1 = std::max(x1, a1); y1 = std::max(y1, b1); } };
	const auto in = [](const std::vector<std::string> &v, const std::string &t) { return std::ranges::find(v, t) != v.end(); };
	Box objects, terrain;
	int n_objects = 0;
	for (const auto &t : ext.types)
	{
		if (in(cfg_.object_types, t.type)) { objects.add(t.xmin, t.ymin, t.xmax, t.ymax); n_objects += t.count; }
		if (in(cfg_.terrain_types, t.type)) terrain.add(t.xmin, t.ymin, t.xmax, t.ymax);
	}
	std::println("[openfield] world {}: {} working-area objects, terrain {}", ext.world, n_objects,
	             terrain.valid ? std::format("x [{:.1f}, {:.1f}] y [{:.1f}, {:.1f}]", terrain.x0, terrain.x1, terrain.y0, terrain.y1)
	                           : std::string("none"));
	Box area;
	if (objects.valid)
	{
		const auto mx = static_cast<float>(cfg_.margin_x), my = static_cast<float>(cfg_.margin_y);
		area.add(objects.x0 - mx, objects.y0 - my, objects.x1 + mx, objects.y1 + my);
		const auto &s = ekf_.state();
		const auto sx = static_cast<float>(s(0)), sy = static_cast<float>(s(1)), sm = static_cast<float>(cfg_.start_margin);
		area.add(sx - sm, sy - sm, sx + sm, sy + sm);
		if (terrain.valid)
		{
			area.x0 = std::max(area.x0, terrain.x0); area.y0 = std::max(area.y0, terrain.y0);
			area.x1 = std::min(area.x1, terrain.x1); area.y1 = std::min(area.y1, terrain.y1);
		}
	}
	else if (terrain.valid)
	{
		qWarning() << "[openfield] no Field.ObjectTypes nodes in the world: the field is the WHOLE terrain";
		area = terrain;
	}
	if (not area.valid or area.x1 - area.x0 < 1.f or area.y1 - area.y0 < 1.f)
	{
		qWarning() << "[openfield] the world gives no usable working area (check Field.ObjectTypes / TerrainTypes)";
		return std::nullopt;
	}
	std::println("[openfield] field polygon from Webots: x [{:.2f}, {:.2f}] y [{:.2f}, {:.2f}] ({:.1f} x {:.1f} m)",
	             area.x0, area.x1, area.y0, area.y1, area.x1 - area.x0, area.y1 - area.y0);
	return std::vector<Eigen::Vector2f>{{area.x0, area.y0}, {area.x1, area.y0}, {area.x1, area.y1}, {area.x0, area.y1}};
}

void SpecificWorker::publish_pose()
{
	rc::openfield::PoseSample s;
	const auto &x = ekf_.state();
	s.x = x(0); s.y = x(1); s.theta = x(2);
	s.cov = ekf_.covariance();
	// Body twist: wheels for translation, the GYRO for rotation (the wheels over-report it on a skid-steer).
	s.adv = odom_adv_; s.side = odom_side_; s.rot = static_cast<float>(last_gyro_z_);
	const auto &p = ekf_.params();
	s.vel_var = {std::max(odom_var_(0), static_cast<float>(p.adv_sigma0 * p.adv_sigma0)),
	             std::max(odom_var_(1), static_cast<float>(p.side_sigma0 * p.side_sigma0)),
	             static_cast<float>(p.gyro_sigma0 * p.gyro_sigma0)};
	s.timestamp_ms = last_imu_stamp_ms_ > 0 ? static_cast<std::uint64_t>(last_imu_stamp_ms_)
	                                        : static_cast<std::uint64_t>(QDateTime::currentMSecsSinceEpoch());
	if (scene_graph_->publish(s)) ++published_;
	if (pose_history_.empty() or pose_history_.back().first < static_cast<std::int64_t>(s.timestamp_ms))
	{
		pose_history_.emplace_back(static_cast<std::int64_t>(s.timestamp_ms), x);
		while (pose_history_.size() > 500) pose_history_.pop_front();   // ~10 s at 50 Hz
	}
}

// ── 1 Hz status line + CSV, graded against the simulator's ground truth when it is on the graph ───
void SpecificWorker::log_status()
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	if (now - last_status_ms_ < 1000) return;
	last_status_ms_ = now;
	if (not ekf_.initialized())
	{
		std::println("[openfield] waiting for a GPS fix and an IMU yaw (imu age {} ms, gps fixes {})",
		             imu_age_ms(), gps_fixes_);
		return;
	}
	const auto &x = ekf_.state();
	const auto &P = ekf_.covariance();
	const double sx = std::sqrt(P(0, 0)), sy = std::sqrt(P(1, 1)), st = std::sqrt(P(2, 2));

	double gx = NAN, gy = NAN, gth = NAN, exy = NAN, eth = NAN;
	std::int64_t gt_lag_ms = -1;
	if (const auto ge = gt_error(); ge.has_value())
	{
		gx = ge->gt(0); gy = ge->gt(1); gth = ge->gt(2); exy = ge->err_xy; eth = ge->err_theta;
		gt_lag_ms = ge->stamped ? 0 : -1;
	}
	std::println("[openfield] pose ({:.3f}, {:.3f}, {:.3f} rad) sigma ({:.3f}, {:.3f} m, {:.4f} rad) | gps fixes {} "
	             "| published {} | GT err {} ",
	             x(0), x(1), x(2), sx, sy, st, gps_fixes_, published_,
	             std::isnan(exy) ? std::string("n/a")
	                             : std::format("{:.3f} m, {:.2f} deg{}", exy, eth * 180.0 / M_PI,
	                                           gt_lag_ms < 0 ? " (newest estimate, GT unstamped)" : ""));
	if (csv_.is_open())
		csv_ << now << ',' << x(0) << ',' << x(1) << ',' << x(2) << ',' << sx << ',' << sy << ',' << st << ','
		     << gps_fixes_ << ',' << gx << ',' << gy << ',' << gth << ',' << exy << ',' << eth << '\n' << std::flush;
}

// Ground truth (simulation only; robot_concept writes it from the supervisor). The bridge's angle is
// atan2(R01, R00) - pi/2 of the robot's rotation, i.e. -(yaw) - pi/2; converted to this filter's theta.
// The estimate is graded AT the ground truth's own timestamp (pose_history_), else against the newest one.
std::optional<SpecificWorker::GtError> SpecificWorker::gt_error() const
{
	if (not ekf_.initialized() or not scene_graph_) return std::nullopt;
	const auto rid = scene_graph_->robot_id();
	if (rid == 0) return std::nullopt;
	const auto n = G->get_node(rid);
	if (not n.has_value()) return std::nullopt;
	const auto ax = G->get_attrib_by_name<robot_gt_x_att>(n.value());
	const auto ay = G->get_attrib_by_name<robot_gt_y_att>(n.value());
	const auto aa = G->get_attrib_by_name<robot_gt_angle_att>(n.value());
	const auto ats = G->get_attrib_by_name<robot_gt_timestamp_att>(n.value());
	if (not (ax.has_value() and ay.has_value() and aa.has_value())) return std::nullopt;
	GtError e;
	e.gt = {ax.value(), ay.value(), wrap_pi(-(aa.value() + M_PI_2))};
	Eigen::Vector3d est = ekf_.state();
	if (ats.has_value())
		if (const auto at = pose_at(static_cast<std::int64_t>(ats.value())); at.has_value())
		{ est = *at; e.stamped = true; }
	e.err_xy = std::hypot(est(0) - e.gt(0), est(1) - e.gt(1));
	e.err_theta = wrap_pi(est(2) - e.gt(2));
	return e;
}

void SpecificWorker::push_viewer()
{
	const auto now = QDateTime::currentMSecsSinceEpoch();
	// ── Test hook (env, off by default): drive the GPS switch and grab the window, for headless checks.
	//    OPENFIELD_TEST_GPS_OFF=<s> / OPENFIELD_TEST_GPS_ON=<s> after start; OPENFIELD_SNAPSHOT=<file.png>
	//    is saved every 5 s.
	{
		static const double off_s = std::getenv("OPENFIELD_TEST_GPS_OFF") ? std::atof(std::getenv("OPENFIELD_TEST_GPS_OFF")) : -1.0;
		static const double on_s  = std::getenv("OPENFIELD_TEST_GPS_ON")  ? std::atof(std::getenv("OPENFIELD_TEST_GPS_ON"))  : -1.0;
		const double t = 1e-3 * static_cast<double>(now - start_ms_);
		static bool off_done = false, on_done = false;
		if (off_s >= 0 and not off_done and t >= off_s) { off_done = true; viewer_->set_gps_checked(false); }
		if (on_s  >= 0 and not on_done  and t >= on_s)  { on_done  = true; viewer_->set_gps_checked(true); }
		static std::int64_t last_snap = 0;
		if (const char *snap = std::getenv("OPENFIELD_SNAPSHOT"); snap and now - last_snap > 5000)
		{ last_snap = now; viewer_->grab().save(QString::fromUtf8(snap)); }
	}
	rc::openfield::ViewerState s;
	s.t_s = 1e-3 * static_cast<double>(now - start_ms_);
	s.initialized = ekf_.initialized();
	s.pose = ekf_.state();
	s.cov = ekf_.covariance();
	s.gps_enabled = gps_enabled_;
	s.gps_fix = last_gps_xy_;
	s.gps_fixes = gps_fixes_;
	s.gps_fused = gps_fused_;
	s.gps_age_s = last_gps_new_ms_ > 0 ? 1e-3 * static_cast<double>(now - last_gps_new_ms_) : -1.0;
	s.gps_innovation_m = last_gps_innovation_m_;
	s.seconds_without_gps = gps_off_since_ms_ > 0 ? 1e-3 * static_cast<double>(now - gps_off_since_ms_) : 0.0;
	s.yaw_enabled = yaw_enabled_;
	s.adv = odom_adv_; s.side = odom_side_; s.wheel_rot = odom_rot_;
	s.gyro_z = last_gyro_z_;
	s.imu_yaw = last_imu_yaw_;
	s.imu_age_ms = static_cast<double>(imu_age_ms());
	if (const auto ge = gt_error(); ge.has_value())
	{ s.gt = ge->gt; s.err_xy = ge->err_xy; s.err_theta = ge->err_theta; }
	s.polygon = field_polygon_;
	viewer_->update_state(s);
}

// The 3D semantic map: rebuilt from the graph (plants, rows, plots, field) and the filter's own pose.
void SpecificWorker::push_world3d()
{
	const auto frame = rc::world::frame_name(*G, "field");
	world3d_->set_scene(rc::openfield::build_world3d(*G, *world3d_eigen_, frame, ekf_.state(), ekf_.initialized(),
	                                                 field_polygon_, exploration_.get()));
}

// Linear interpolation in the published-pose history (heading along the short arc).
std::optional<Eigen::Vector3d> SpecificWorker::pose_at(std::int64_t t) const
{
	if (pose_history_.size() < 2 or t < pose_history_.front().first or t > pose_history_.back().first)
		return std::nullopt;
	const auto hi = std::ranges::lower_bound(pose_history_, t, {}, &std::pair<std::int64_t, Eigen::Vector3d>::first);
	if (hi == pose_history_.begin()) return hi->second;
	const auto lo = std::prev(hi);
	const double a = hi->first == lo->first ? 0.0
	               : static_cast<double>(t - lo->first) / static_cast<double>(hi->first - lo->first);
	Eigen::Vector3d p = lo->second + a * (hi->second - lo->second);
	p(2) = wrap_pi(lo->second(2) + a * wrap_pi(hi->second(2) - lo->second(2)));
	return p;
}

// ── Shutdown: delete what we own ([Owns] in etc/config.toml), leave DDS cleanly, hard-exit ────────
void SpecificWorker::request_shutdown()
{
	if (shutting_down_.exchange(true)) return;
	if (G) disconnect(G.get(), nullptr, this, nullptr);
	viewer_.reset();
	world3d_.reset();
	imu_sub_.reset();
	if (exploration_) exploration_->cleanup();   // afford_openfield
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
	const bool ok = rc::openfield::OpenFieldEkf::self_test(&report);
	std::println("{}", report);
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return ok ? 0 : 1;
}
