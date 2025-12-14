/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file navigator_main.cpp
 *
 * Handles mission items, geo fencing and failsafe navigation behavior.
 * Published the position setpoint triplet for the position controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * and many more...
 */

#include "automation.h"

#include <float.h>
#include <sys/stat.h>

#include <dataman_client/DatamanClient.hpp>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/mavlink_log.h>

using namespace time_literals;

namespace navigator
{
Automation *g_automation;
}

Automation::Automation() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence(this),
	_gf_breach_avoidance(this),
	_mission(this),
	_loiter(this),
	_takeoff(this),
#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
	_vtol_takeoff(this),
#endif //CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
	_land(this),
	_precland(this),
	_rtl(this),
	_vla_trajectory_task(this)
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_mission_task;
	_navigation_mode_array[1] = &_loiter;
	_navigation_mode_array[2] = &_rtl;
	_navigation_mode_array[3] = &_takeoff;
	_navigation_mode_array[4] = &_land;
	_navigation_mode_array[5] = &_precland;
	_navigation_mode_array[6] = &_vla_trajectory_task;
#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
	_navigation_mode_array[7] = &_vtol_takeoff;
#endif //CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

	/* iterate through navigation modes and initialize _current_task_item for each */
	for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
		if (_navigation_mode_array[i]) {
			_navigation_mode_array[i]->initialize();
		}
	}

	_handle_back_trans_dec_mss = param_find("VT_B_DEC_MSS");
	_handle_mpc_jerk_auto = param_find("MPC_JERK_AUTO");
	_handle_mpc_acc_hor = param_find("MPC_ACC_HOR");

	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_mission_sub = orb_subscribe(ORB_ID(mission));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	_distance_sensor_mode_change_request_pub.advertise();
	_distance_sensor_mode_change_request_pub.get().timestamp = hrt_absolute_time();
	_distance_sensor_mode_change_request_pub.get().request_on_off = distance_sensor_mode_change_request_s::REQUEST_OFF;
	_distance_sensor_mode_change_request_pub.update();

	reset_triplets();
}

Automation::~Automation()
{
	perf_free(_loop_perf);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_mission_sub);
	orb_unsubscribe(_vehicle_status_sub);
}

void Automation::params_update()
{
	updateParams();

	if (_handle_back_trans_dec_mss != PARAM_INVALID) {
		param_get(_handle_back_trans_dec_mss, &_param_back_trans_dec_mss);
	}

	if (_handle_mpc_jerk_auto != PARAM_INVALID) {
		param_get(_handle_mpc_jerk_auto, &_param_mpc_jerk_auto);
	}

	if (_handle_mpc_acc_hor != PARAM_INVALID) {
		param_get(_handle_mpc_acc_hor, &_param_mpc_acc_hor);
	}

	_mission_task.set_command_timeout(_param_mis_command_tout.get());
#if CONFIG_NAVIGATOR_ADSB
	_adsb_conflict.set_conflict_detection_params(_param_nav_traff_a_hor_ct.get(),
			_param_nav_traff_a_ver.get(),
			_param_nav_traff_collision_time.get(), _param_nav_traff_avoid.get());
#endif // CONFIG_NAVIGATOR_ADSB
}

void Automation::run()
{

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file */
	struct stat buffer;

	if (stat(GEOFENCE_FILENAME, &buffer) == 0) {
		PX4_INFO("Loading geofence from %s", GEOFENCE_FILENAME);
		_geofence.loadFromFile(GEOFENCE_FILENAME);
	}

	params_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3] {};

	/* Setup of loop */
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vehicle_status_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _mission_sub;
	fds[2].events = POLLIN;

	uint32_t geofence_id{0};
	uint32_t safe_points_id{0};

	/* rate-limit position subscription to 20 Hz / 50 ms */
	orb_set_interval(_local_pos_sub, 50);

	while (!should_exit()) {

		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* Let the loop run anyway, don't do `continue` here. */

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(10000);
			continue;
		}

		perf_begin(_loop_perf);

		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vstatus);

		if (fds[2].revents & POLLIN) {
			mission_s mission;
			orb_copy(ORB_ID(mission), _mission_sub, &mission);

			if (mission.geofence_id != geofence_id) {
				geofence_id = mission.geofence_id;
				_geofence.updateFence();
			}

			if (mission.safe_points_id != safe_points_id) {
				safe_points_id = mission.safe_points_id;
				_rtl.updateSafePoints(safe_points_id);
			}
		}

		/* gps updated */
		if (_gps_pos_sub.updated()) {
			_gps_pos_sub.copy(&_gps_pos);
		}

		/* global position updated */
		if (_global_pos_sub.updated()) {
			_global_pos_sub.copy(&_global_pos);
		}

		/* check for parameter updates */
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			params_update();
		}

		_land_detected_sub.update(&_land_detected);
		_position_controller_status_sub.update();
		_home_pos_sub.update(&_home_pos);

		/* Handle automation task requests */
		handle_automation_task();

#if CONFIG_NAVIGATOR_ADSB
		/* Check for traffic */
		check_traffic();
#endif // CONFIG_NAVIGATOR_ADSB

		/* Check geofence violation */
		geofence_breach_check();

		/* iterate through navigation modes and set active/inactive for each */
		for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
			if (_navigation_mode_array[i]) {
				_navigation_mode_array[i]->run(_navigation_mode == _navigation_mode_array[i]);
			}
		}

		/* if nothing is running, set position setpoint triplet invalid once */
		if (_navigation_mode == nullptr && !_pos_sp_triplet_published_invalid_once) {
			_pos_sp_triplet_published_invalid_once = true;
			reset_triplets();
		}

		if (_pos_sp_triplet_updated) {
			publish_position_setpoint_triplet();
		}

		if (_task_result_updated) {
			publish_mission_result();
		}

		publish_automation_status();

		publish_distance_sensor_mode_request();

		_geofence.run();

		perf_end(_loop_perf);
	}
}

void Automation::geofence_breach_check()
{
	// reset the _time_loitering_after_gf_breach time if no longer in LOITER (and 100ms after it was triggered)
	if (_vstatus.operation_mode != vehicle_status_s::OPERATION_MODE_AUTO_LOITER
	    && hrt_elapsed_time(&_time_loitering_after_gf_breach) > 100_ms) {
		_time_loitering_after_gf_breach = 0;
	}

	if ((_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
	    (hrt_elapsed_time(&_last_geofence_check) > GEOFENCE_CHECK_INTERVAL_US)) {

		const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

		geofence_violation_type_u gf_violation_type{};
		float test_point_bearing;
		float test_point_distance;
		float vertical_test_point_distance;

		if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			test_point_bearing = atan2f(_local_pos.vy, _local_pos.vx);
			const float velocity_hor_abs = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
			_gf_breach_avoidance.setHorizontalVelocity(velocity_hor_abs);
			_gf_breach_avoidance.setClimbRate(-_local_pos.vz);
			test_point_distance = _gf_breach_avoidance.computeBrakingDistanceMultirotor();
			vertical_test_point_distance = _gf_breach_avoidance.computeVerticalBrakingDistanceMultirotor();

		} else {
			test_point_distance = 2.0f * get_loiter_radius();
			vertical_test_point_distance = 5.0f;

			if (hrt_absolute_time() - pos_ctrl_status.timestamp < 100000 && PX4_ISFINITE(pos_ctrl_status.nav_bearing)) {
				test_point_bearing = pos_ctrl_status.nav_bearing;

			} else {
				test_point_bearing = atan2f(_local_pos.vy, _local_pos.vx);
			}
		}

		double current_latitude = _global_pos.lat;
		double current_longitude = _global_pos.lon;
		float current_altitude = _global_pos.alt;
		bool position_valid = _global_pos.timestamp > 0;

		if (_geofence.getSource() == Geofence::GF_SOURCE_GPS) {
			current_latitude = _gps_pos.latitude_deg;
			current_longitude = _gps_pos.longitude_deg;
			current_altitude = _gps_pos.altitude_msl_m;
			position_valid = _global_pos.timestamp > 0;
		}

		if (!position_valid) {
			// we don't have a valid position yet, so we can't check for geofence violations
			return;
		}

		_gf_breach_avoidance.setHorizontalTestPointDistance(test_point_distance);
		_gf_breach_avoidance.setVerticalTestPointDistance(vertical_test_point_distance);
		_gf_breach_avoidance.setTestPointBearing(test_point_bearing);
		_gf_breach_avoidance.setCurrentPosition(current_latitude, current_longitude, current_altitude);
		_gf_breach_avoidance.setMaxHorDistHome(_geofence.getMaxHorDistanceHome());

		if (home_global_position_valid()) {
			_gf_breach_avoidance.setHomePosition(_home_pos.lat, _home_pos.lon, _home_pos.alt);
		}

		double test_point_latitude = current_latitude;
		double test_point_longitude = current_longitude;
		float test_point_altitude = current_altitude;

		if (_geofence.getPredict()) {
			matrix::Vector2<double>fence_violation_test_point = _gf_breach_avoidance.getFenceViolationTestPoint();
			test_point_latitude = fence_violation_test_point(0);
			test_point_longitude = fence_violation_test_point(1);
			test_point_altitude = current_altitude + vertical_test_point_distance;
		}

		if (_time_loitering_after_gf_breach > 0) {
			// if we are in the loitering state after breaching a GF, only allow new ones to be set, but not unset
			_geofence_result.geofence_max_dist_triggered |= !_geofence.isCloserThanMaxDistToHome(test_point_latitude,
					test_point_longitude, test_point_altitude);
			_geofence_result.geofence_max_alt_triggered |= !_geofence.isBelowMaxAltitude(test_point_altitude);
			_geofence_result.geofence_custom_fence_triggered |= !_geofence.isInsidePolygonOrCircle(test_point_latitude,
					test_point_longitude, test_point_altitude);

		} else {
			_geofence_result.geofence_max_dist_triggered = !_geofence.isCloserThanMaxDistToHome(test_point_latitude,
					test_point_longitude, test_point_altitude);
			_geofence_result.geofence_max_alt_triggered = !_geofence.isBelowMaxAltitude(test_point_altitude);
			_geofence_result.geofence_custom_fence_triggered = !_geofence.isInsidePolygonOrCircle(test_point_latitude,
					test_point_longitude, test_point_altitude);
		}

		_last_geofence_check = hrt_absolute_time();

		_geofence_result.timestamp = hrt_absolute_time();
		_geofence_result.geofence_action = _geofence.getGeofenceAction();

		if (_geofence_result.geofence_max_dist_triggered || _geofence_result.geofence_max_alt_triggered ||
		    _geofence_result.geofence_custom_fence_triggered) {

			/* Issue a warning about the geofence violation once and only if we are armed */
			if (!_geofence_reposition_sent && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED
			    && _geofence.getGeofenceAction() == geofence_result_s::GF_ACTION_LOITER) {

				// we have predicted a geofence violation and if the action is to loiter then
				// demand a reposition to a location which is inside the geofence

				position_setpoint_triplet_s *rep = get_reposition_triplet();

				matrix::Vector2<double> loiter_center_lat_lon;

				float loiter_altitude_amsl = current_altitude;
				double loiter_latitude = current_latitude;
				double loiter_longitude = current_longitude;

				if (_geofence.getPredict()) {
					if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
						// the computation of the braking distance does not match the actual braking distance. Until we have a better model
						// we set the loiter point to the current position, that will make sure that the vehicle will loiter inside the fence
						loiter_center_lat_lon =  _gf_breach_avoidance.generateLoiterPointForMultirotor(gf_violation_type,
									 &_geofence);
						loiter_latitude = loiter_center_lat_lon(0);
						loiter_longitude = loiter_center_lat_lon(1);

						loiter_altitude_amsl = _gf_breach_avoidance.generateLoiterAltitudeForMulticopter(gf_violation_type);

					} else {

						loiter_center_lat_lon = _gf_breach_avoidance.generateLoiterPointForFixedWing(gf_violation_type, &_geofence);
						loiter_latitude = loiter_center_lat_lon(0);
						loiter_longitude = loiter_center_lat_lon(1);

						loiter_altitude_amsl = _gf_breach_avoidance.generateLoiterAltitudeForFixedWing(gf_violation_type);
					}
				}

				rep->current.timestamp = hrt_absolute_time();
				rep->current.yaw = NAN;
				rep->current.lat = loiter_latitude;
				rep->current.lon = loiter_longitude;
				rep->current.alt = loiter_altitude_amsl;
				rep->current.valid = true;
				rep->current.loiter_radius = get_loiter_radius();
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
				rep->current.cruising_throttle = get_cruising_throttle();
				rep->current.acceptance_radius = get_acceptance_radius();
				rep->current.cruising_speed = get_cruising_speed();

				_geofence_reposition_sent = true;
				_time_loitering_after_gf_breach = hrt_absolute_time();
			}

		} else {

			_geofence_reposition_sent = false;
		}

		_geofence_result_pub.publish(_geofence_result);
	}
}

int Automation::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("navigator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_NAVIGATION,
				      PX4_STACK_ADJUSTED(2200),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Automation *Automation::instantiate(int argc, char *argv[])
{
	Automation *instance = new Automation();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int Automation::print_status()
{
	PX4_INFO("Running");

	_geofence.printStatus();
	return 0;
}

void Automation::publish_position_setpoint_triplet()
{
	_pos_sp_triplet.timestamp = hrt_absolute_time();
	_pos_sp_triplet_pub.publish(_pos_sp_triplet);
	_pos_sp_triplet_updated = false;
}

float Automation::get_default_acceptance_radius()
{
	return _param_nav_acc_rad.get();
}

float Automation::get_altitude_acceptance_radius()
{
	if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		const position_setpoint_s &curr_sp = get_position_setpoint_triplet()->current;
		const position_setpoint_s &next_sp = get_position_setpoint_triplet()->next;

		if ((PX4_ISFINITE(curr_sp.alt_acceptance_radius) && curr_sp.alt_acceptance_radius > FLT_EPSILON)) {
			return curr_sp.alt_acceptance_radius;

		} else if (!force_vtol() && next_sp.type == position_setpoint_s::SETPOINT_TYPE_LAND && next_sp.valid) {
			// Use separate (tighter) altitude acceptance for clean altitude starting point before FW landing
			return _param_nav_fw_altl_rad.get();

		} else {
			return _param_nav_fw_alt_rad.get();
		}

	} else if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		return INFINITY;

	} else {
		float alt_acceptance_radius = _param_nav_mc_alt_rad.get();
		const position_setpoint_s &curr_sp = get_position_setpoint_triplet()->current;

		if (PX4_ISFINITE(curr_sp.alt_acceptance_radius) && curr_sp.alt_acceptance_radius > FLT_EPSILON) {
			alt_acceptance_radius = curr_sp.alt_acceptance_radius;

		}

		return alt_acceptance_radius;
	}
}

void Automation::reset_triplets()
{
	reset_position_setpoint(_pos_sp_triplet.previous);
	reset_position_setpoint(_pos_sp_triplet.current);
	reset_position_setpoint(_pos_sp_triplet.next);

	_pos_sp_triplet_updated = true;
}

void Automation::reset_position_setpoint(position_setpoint_s &sp)
{
	sp = position_setpoint_s{};
	sp.timestamp = hrt_absolute_time();
	sp.lat = static_cast<double>(NAN);
	sp.lon = static_cast<double>(NAN);
	sp.yaw = NAN;
	sp.loiter_radius = get_loiter_radius();
	sp.acceptance_radius = get_default_acceptance_radius();
	sp.cruising_speed = get_cruising_speed();
	sp.cruising_throttle = get_cruising_throttle();
	sp.valid = false;
	sp.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	sp.loiter_direction_counter_clockwise = false;
}

float Automation::get_cruising_throttle()
{
	/* Return the mission-requested cruise speed, or default FW_THR_TRIM value */
	if (_mission_throttle > FLT_EPSILON) {
		return _mission_throttle;

	} else {
		return NAN;
	}
}

float Automation::get_acceptance_radius()
{
	float acceptance_radius = get_default_acceptance_radius(); // the value specified in the parameter NAV_ACC_RAD
	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	// for fixed-wing and rover, return the max of NAV_ACC_RAD and the controller acceptance radius (e.g. navigation switch distance)
	if (_vstatus.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && PX4_ISFINITE(pos_ctrl_status.acceptance_radius) && pos_ctrl_status.timestamp != 0) {

		acceptance_radius = math::max(acceptance_radius, pos_ctrl_status.acceptance_radius);
	}

	return acceptance_radius;
}

bool Automation::get_yaw_to_be_accepted(float mission_item_yaw)
{
	float yaw = mission_item_yaw;

	return PX4_ISFINITE(yaw);
}

void Automation::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}

#if CONFIG_NAVIGATOR_ADSB
void Automation::take_traffic_conflict_action()
{

	vehicle_command_s vehicle_command{};

	switch (_adsb_conflict._conflict_detection_params.traffic_avoidance_mode) {

	case 2: {
			_rtl.set_return_alt_min(true);
			vehicle_command.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
			publish_vehicle_command(vehicle_command);
			break;
		}

	case 3: {
			vehicle_command.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
			publish_vehicle_command(vehicle_command);
			break;

		}

	case 4: {

			vehicle_command.command = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
			publish_vehicle_command(vehicle_command);
			break;

		}
	}
}

void Automation::run_fake_traffic()
{
	_adsb_conflict.run_fake_traffic(get_global_position()->lat, get_global_position()->lon,
					get_global_position()->alt);
}

void Automation::check_traffic()
{
	if (_traffic_sub.updated()) {
		_traffic_sub.copy(&_adsb_conflict._transponder_report);

		uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY | transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

		if ((_adsb_conflict._transponder_report.flags & required_flags) == required_flags) {

			_adsb_conflict.detect_traffic_conflict(get_global_position()->lat, get_global_position()->lon,
							       get_global_position()->alt, _local_pos.vx, _local_pos.vy, _local_pos.vz);

			if (_adsb_conflict.handle_traffic_conflict()) {
				take_traffic_conflict_action();
			}
		}
	}

	_adsb_conflict.remove_expired_conflicts();
}
#endif // CONFIG_NAVIGATOR_ADSB

bool Automation::abort_landing()
{
	// only abort if currently landing and position controller status updated
	bool should_abort = false;

	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

		if (_pos_ctrl_landing_status_sub.updated()) {
			position_controller_landing_status_s landing_status{};

			// landing status from position controller must be newer than navigator's last position setpoint
			if (_pos_ctrl_landing_status_sub.copy(&landing_status)) {
				if (landing_status.timestamp > _pos_sp_triplet.timestamp) {
					should_abort = (landing_status.abort_status > 0);
				}
			}
		}
	}

	return should_abort;
}

bool Automation::force_vtol()
{
	return _vstatus.is_vtol &&
	       (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || _vstatus.in_transition_to_fw)
	       && _param_nav_force_vt.get();
}

int Automation::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "fencefile")) {
		get_instance()->load_fence_from_file(GEOFENCE_FILENAME);
		return 0;

#if CONFIG_NAVIGATOR_ADSB

	} else if (!strcmp(argv[0], "fake_traffic")) {

		get_instance()->run_fake_traffic();

		return 0;
#endif // CONFIG_NAVIGATOR_ADSB
	}

	return print_usage("unknown command");
}

void Automation::publish_mission_result()
{
	_mission_result.timestamp = hrt_absolute_time();

	/* lazily publish the mission result only once available */
	_mission_result_pub.publish(_mission_result);

	/* reset some of the flags */
	_mission_result.item_do_jump_changed = false;
	_mission_result.item_changed_index = 0;
	_mission_result.item_do_jump_remaining = 0;

	_task_result_updated = false;
}

void Automation::handle_automation_task()
{
	automation_task_s task;

	if (_automation_task_sub.update(&task)) {
		TaskBase *new_task = nullptr;
		uint8_t result = automation_task_result_s::RESULT_UNSUPPORTED;

		switch (task.task_type) {
		case automation_task_s::TASK_MISSION:
			new_task = &_mission_task;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

		case automation_task_s::TASK_LOITER:
			new_task = &_loiter;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

		case automation_task_s::TASK_RTL:
			new_task = &_rtl;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

		case automation_task_s::TASK_TAKEOFF:
			new_task = &_takeoff;

			// Set takeoff altitude if provided
			if (PX4_ISFINITE(task.altitude)) {
				_takeoff_triplet.current.alt = task.altitude;
			}

			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

		case automation_task_s::TASK_LAND:
			new_task = &_land;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

		case automation_task_s::TASK_PRECLAND:
			new_task = &_precland;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

		case automation_task_s::TASK_VLA_TRAJECTORY:
			new_task = &_vla_trajectory_task;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

		case automation_task_s::TASK_VTOL_TAKEOFF:
			new_task = &_vtol_takeoff;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;
#endif // CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

		case automation_task_s::TASK_NONE:
			// Deactivate current task
			_navigation_mode = nullptr;
			result = automation_task_result_s::RESULT_ACCEPTED;
			break;

		default:
			PX4_WARN("Unsupported automation task type: %d", task.task_type);
			result = automation_task_result_s::RESULT_UNSUPPORTED;
			break;
		}

		if (new_task != nullptr) {
			_navigation_mode = new_task;
			_pos_sp_triplet_published_invalid_once = false;
		}

		// Publish result
		publish_automation_task_result(task.task_type, result,
					       task.cmd_source_system, task.cmd_source_component, task.cmd_command);
	}
}

void Automation::publish_automation_task_result(uint8_t task_type, uint8_t result,
		uint8_t source_system, uint8_t source_component, uint32_t cmd)
{
	automation_task_result_s task_result{};
	task_result.timestamp = hrt_absolute_time();
	task_result.task_type = task_type;
	task_result.result = result;
	task_result.cmd_source_system = source_system;
	task_result.cmd_source_component = source_component;
	task_result.cmd_command = cmd;
	_automation_task_result_pub.publish(task_result);
}

void Automation::set_mission_failure_heading_timeout()
{
	if (!_mission_result.failure) {
		_mission_result.failure = true;
		set_task_result_updated();
		mavlink_log_critical(&_mavlink_log_pub, "unable to reach heading within timeout\t");
		events::send(events::ID("navigator_mission_failure_heading"), events::Log::Critical,
			     "Mission failure: unable to reach heading within timeout");
	}
}

void Automation::trigger_hagl_failsafe(const uint8_t nav_state)
{
	if ((_automation_status.failure != navigator_status_s::FAILURE_HAGL) || _automation_status.operation_mode != nav_state) {
		_automation_status.failure = navigator_status_s::FAILURE_HAGL;
		_automation_status.operation_mode = nav_state;

		_automation_status_updated = true;
	}
}

void Automation::publish_automation_status()
{
	uint8_t current_operation_mode = _vstatus.operation_mode;

	if (_navigation_mode != nullptr) {
		current_operation_mode = _navigation_mode->getAutomationStateId();
	}

	if (_automation_status.operation_mode != current_operation_mode) {
		_automation_status.operation_mode = current_operation_mode;
		_automation_status.failure = navigator_status_s::FAILURE_NONE;
		_automation_status_updated = true;
	}

	if (_automation_status_updated
	    || (hrt_elapsed_time(&_last_automation_status_publication) > 500_ms)) {
		_automation_status.timestamp = hrt_absolute_time();
		_automation_status_pub.publish(_automation_status);

		_automation_status_updated = false;
		_last_automation_status_publication = hrt_absolute_time();
	}
}

void Automation::publish_vehicle_command(vehicle_command_s &vehicle_command)
{
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.source_system = _vstatus.system_id;
	vehicle_command.source_component = _vstatus.component_id;
	vehicle_command.target_system = _vstatus.system_id;
	vehicle_command.confirmation = false;
	vehicle_command.from_external = false;

	int target_camera_component_id;

	// The camera commands are not processed on the autopilot but will be
	// sent to the mavlink links to other components.
	switch (vehicle_command.command) {
	case NAV_CMD_IMAGE_START_CAPTURE:

		if (static_cast<int>(vehicle_command.param3) == 1) {
			// When sending a single capture we need to include the sequence number, thus camera_trigger needs to handle this command
			vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
			vehicle_command.param1 = 0.f; // Session control hide lens
			vehicle_command.param2 = 0.f; // Zoom absolute position
			vehicle_command.param3 = 0.f; // Zoom step
			vehicle_command.param4 = 0.f; // Focus lock
			vehicle_command.param5 = 1.; // Shoot command
			vehicle_command.param6 = 0.; // Command identity
			vehicle_command.param7 = 0.f; // Shot identifier

		} else {
			// We are only capturing multiple if param3 is 0 or > 1.
			// For multiple pictures the sequence number does not need to be included, thus there is no need to go through camera_trigger
			_is_capturing_images = true;
		}

		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_IMAGE_STOP_CAPTURE:
		_is_capturing_images = false;
		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_SET_CAMERA_MODE:
		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_SET_CAMERA_SOURCE:
		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
		vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	default:
		vehicle_command.target_component = 0;
		break;
	}

	_vehicle_cmd_pub.publish(vehicle_command);
}

void Automation::publish_distance_sensor_mode_request()
{
	// Send request to enable distance sensor when in the landing phase of a mission or RTL
	if (((_navigation_mode == &_rtl) && _rtl.isLanding()) || ((_navigation_mode == &_mission_task) && _mission_task.isLanding())) {

		if (_distance_sensor_mode_change_request_pub.get().request_on_off !=
		    distance_sensor_mode_change_request_s::REQUEST_ON) {

			_distance_sensor_mode_change_request_pub.get().timestamp = hrt_absolute_time();
			_distance_sensor_mode_change_request_pub.get().request_on_off =
				distance_sensor_mode_change_request_s::REQUEST_ON;
			_distance_sensor_mode_change_request_pub.update();
		}

	} else if (_distance_sensor_mode_change_request_pub.get().request_on_off !=
		   distance_sensor_mode_change_request_s::REQUEST_OFF) {

		_distance_sensor_mode_change_request_pub.get().timestamp = hrt_absolute_time();
		_distance_sensor_mode_change_request_pub.get().request_on_off =
			distance_sensor_mode_change_request_s::REQUEST_OFF;
		_distance_sensor_mode_change_request_pub.update();
	}
}

void Automation::publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s command_ack = {};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.from_external = false;

	command_ack.result = result;
	command_ack.result_param1 = 0;
	command_ack.result_param2 = 0;

	_vehicle_cmd_ack_pub.publish(command_ack);
}

void Automation::acquire_gimbal_control()
{
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vehicle_command.param1 = _vstatus.system_id; // Take primary control
	vehicle_command.param2 = _vstatus.component_id;
	vehicle_command.param3 = -1.f; // Leave secondary control unchanged
	vehicle_command.param4 = -1.f;
	publish_vehicle_command(vehicle_command);
}

void Automation::release_gimbal_control()
{
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vehicle_command.param1 = -3.f; // Remove primary control if it was taken
	vehicle_command.param2 = -3.f;
	vehicle_command.param3 = -1.f; // Leave secondary control unchanged
	vehicle_command.param4 = -1.f;
	publish_vehicle_command(vehicle_command);
}


void Automation::stop_capturing_images()
{
	if (_is_capturing_images) {
		vehicle_command_s vehicle_command{};
		vehicle_command.command = NAV_CMD_IMAGE_STOP_CAPTURE;
		vehicle_command.param1 = 0.f;
		publish_vehicle_command(vehicle_command);

		// _is_capturing_images is reset inside publish_vehicle_command.
	}
}

bool Automation::geofence_allows_position(const vehicle_global_position_s &pos)
{
	if ((_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
	    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_WARN)) {

		if (PX4_ISFINITE(pos.lat) && PX4_ISFINITE(pos.lon) && PX4_ISFINITE(pos.alt)) {
			return _geofence.checkPointAgainstAllGeofences(pos.lat, pos.lon, pos.alt);
		}
	}

	return true;
}

void Automation::preproject_stop_point(double &lat, double &lon)
{
	// For multirotors we need to account for the braking distance, otherwise the vehicle will overshoot and go back
	const float course_over_ground = atan2f(_local_pos.vy, _local_pos.vx);

	// predict braking distance

	const float velocity_hor_abs = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);

	const float multirotor_braking_distance = math::trajectory::computeBrakingDistanceFromVelocity(velocity_hor_abs,
			_param_mpc_jerk_auto, _param_mpc_acc_hor, 0.6f * _param_mpc_jerk_auto);

	waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, course_over_ground,
					   multirotor_braking_distance, &lat, &lon);
}

void Automation::mode_completed(uint8_t nav_state, uint8_t result)
{
	mode_completed_s mode_completed{};
	mode_completed.timestamp = hrt_absolute_time();
	mode_completed.result = result;
	mode_completed.operation_mode = nav_state;
	_mode_completed_pub.publish(mode_completed);
}


void Automation::disable_camera_trigger()
{
	// Disable camera trigger
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// Pause trigger
	vehicle_command.param1 = -1.f;
	vehicle_command.param3 = 1.f;
	publish_vehicle_command(vehicle_command);
}

void Automation::set_gimbal_neutral()
{
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
	vehicle_command.param1 = NAN; // Don't set any angles
	vehicle_command.param2 = NAN;
	vehicle_command.param3 = NAN; // Don't set any angular velocities
	vehicle_command.param4 = NAN;
	vehicle_command.param5 = gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_NEUTRAL;
	publish_vehicle_command(vehicle_command);
}

void Automation::sendWarningDescentStoppedDueToTerrain()
{
	mavlink_log_critical(&_mavlink_log_pub, "Terrain collision risk, descent is stopped\t");
	events::send(events::ID("navigator_terrain_collision_risk"), events::Log::Critical,
		     "Terrain collision risk, descent is stopped");
}

int Automation::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module that is responsible for autonomous flight modes. This includes missions (read from dataman),
takeoff and RTL.
It is also responsible for geofence violation checking.

### Implementation
The different internal modes are implemented as separate classes that inherit from a common base class `TaskBase`.
The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("automation", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fencefile", "load a geofence file from SD card, stored at etc/geofence.txt");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fake_traffic", "publishes 24 fake transponder_report_s uORB messages");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * automation app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int automation_main(int argc, char *argv[])
{
	return Automation::main(argc, argv);
}
