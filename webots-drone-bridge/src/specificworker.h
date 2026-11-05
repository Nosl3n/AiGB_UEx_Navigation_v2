/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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

// drone_bridge — Webots extern controller for the inspection drone (webots-agri InspectionDrone.proto).
//
//   * Camera  -> DDS media plane (rc/drone/rgb, ImageFrame RGB8), descriptor served by MediaPlaneDDS.
//   * Pose    -> FullPoseEstimation:
//                  getFullPoseEuler()  = the drone in the CARRIER's body frame (the Husky: +Y fwd, +X right,
//                                        +Z up), the pose robot_concept writes on the body -> dron RT edge;
//                  getFullPoseMatrix() = the CAMERA in the drone's frame (gimbal pitch included), the pose
//                                        robot_concept writes on the dron -> camera RT edge.
//                Both frames follow the robot convention: +Y = forward / optical axis, +X right, +Z up.
//                In simulation they are ground truth from the Supervisor API (source "webots-supervisor").
//   * Joystick -> flight: JoystickAdapter messages whose `id` is "drone" fly it (the Xbox controller sets the id
//                with A = drone, B = robot); every other message is ignored. Stabiliser: flight_controller.h.
//
// Not a DSR agent: it moves pixels and answers queries; robot_concept owns the graph nodes.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "camera_publisher.h"
#include "flight_controller.h"
#include "ardupilot_link.h"
#include "mavlink_pilot.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	~SpecificWorker();

	RoboCompFullPoseEstimation::FullPoseEuler FullPoseEstimation_getFullPoseEuler();
	RoboCompFullPoseEstimation::FullPoseMatrix FullPoseEstimation_getFullPoseMatrix();
	void FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz);

	std::string MediaPlaneDDS_getMediaDescriptor();

	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();

private:
	bool startup_check_flag;

	struct Params
	{
		std::string webots_url   = "ipc://1234/inspection_drone";
		std::string carrier_def  = "husky";
		std::string camera_name  = "camera";
		int         camera_period_ms = 64;
		double      camera_pitch = 1.5707963;       // rad, 0 = forward, pi/2 = down
		// joystick
		std::string mode_id      = "drone";         // TData.id that addresses this drone
		std::string axis_vertical = "drone_vertical", axis_yaw = "drone_yaw",
		            axis_forward = "drone_forward",   axis_side = "drone_side";
		std::string button_cam_front = "drone_cam_front", button_cam_down = "drone_cam_down";
		double max_vxy = 2.0, max_vz = 1.0, max_yaw_rate = 1.0;
		int    command_timeout_ms = 500;
		// test
		std::string test_script;                   // "" = joystick; "hover" = scripted flight for tuning
		// dds
		CameraPublisher::Config dds;
		drone::FlightGains gains;
		// flight backend: "internal" = flight_controller.h; "ardupilot" = ArduCopter SITL over the JSON interface
		std::string flight_backend = "internal";
		int    ap_port      = 9002;                // UDP port SITL sends its servo frames to
		double ap_max_omega = 165.0;               // rad/s at PWM 2000 (thrust linear in PWM: omega = max·sqrt(u))
		int    ap_timeout_ms = 1000;               // no servo frame for this long -> SITL gone, rotors off
		double motor_tau_s   = 0.03;               // rotor speed first-order lag (s); used with externalPropulsion
		drone::PilotConfig pilot;                  // joystick -> ArduPilot (MAVLink), Flight.Backend = "ardupilot"
	} params;

	// shared between the Ice threads and the simulation thread
	struct Shared
	{
		drone::FlightCommand cmd;
		std::chrono::steady_clock::time_point cmd_stamp{};
		double pitch_request = -1.0;               // <0: no request
		RoboCompFullPoseEstimation::FullPoseEuler drone_in_carrier{};
		RoboCompFullPoseEstimation::FullPoseMatrix camera_in_drone{};
		bool pose_valid = false;
		bool selected = false;                     // the joystick's control target is this drone
	};
	std::mutex mtx_;
	Shared shared_;

	CameraPublisher camera_pub_;
	drone::MavlinkPilot pilot_;
	std::thread sim_thread_;
	std::atomic_bool stop_{false};
	std::atomic_bool connected_{false};
	std::atomic_bool finished_{false};
	std::atomic_bool armed_{false};
	std::atomic<double> altitude_{0.0};
	std::atomic<std::uint64_t> frames_{0};

	void sim_loop();
};

#endif
