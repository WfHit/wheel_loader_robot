/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file mission_base.cpp
 *
 * Mission base mode class that can be used for modes interacting with a mission.
 *
 */

#include <cmath>

#include "mission_base.h"

#include "px4_platform_common/defines.h"

#include "mission_feasibility_checker.h"
#include "../automation.h"

MissionBase::MissionBase(
	Automation *automation, int32_t dataman_cache_size_signed, uint8_t automation_state_id) :
	TaskBlock(automation, automation_state_id),
	ModuleParams(automation),
	_dataman_cache_size_signed(dataman_cache_size_signed)
{
	_dataman_cache.resize(abs(dataman_cache_size_signed));

	// Reset _mission here, and listen on changes on the uorb topic instead of initialize from dataman.
	_current_auto_mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	_current_auto_mission.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	_current_auto_mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	_current_auto_mission.count = 0;
	_current_auto_mission.current_seq = 0;
	_current_auto_mission.land_start_index = -1;
	_current_auto_mission.land_index = -1;
	_current_auto_mission.mission_id = 0;
	_current_auto_mission.geofence_id = 0;
	_current_auto_mission.safe_points_id = 0;

	_mission_pub.advertise();
}

void
MissionBase::updateDatamanCache()
{
	if ((_current_auto_mission.count > 0) && (_current_auto_mission.current_seq != _load_mission_index)) {

		const int32_t start_index = math::constrain(_current_auto_mission.current_seq, INT32_C(0), int32_t(_current_auto_mission.count) - 1);
		const int32_t end_index = math::constrain(start_index + _dataman_cache_size_signed, INT32_C(0),
					  int32_t(_current_auto_mission.count) - 1);

		for (int32_t index = start_index; index != end_index; index += math::signNoZero(_dataman_cache_size_signed)) {

			_dataman_cache.load(static_cast<dm_item_t>(_current_auto_mission.mission_dataman_id), index);
		}

		_load_mission_index = _current_auto_mission.current_seq;
	}

	_dataman_cache.update();
}

void MissionBase::updateMavlinkMission()
{
	if (_mission_sub.updated()) {
		mission_s new_current_auto_mission;
		_mission_sub.update(&new_current_auto_mission);

		const bool mission_items_changed = (new_current_auto_mission.mission_id != _current_auto_mission.mission_id);
		const bool mission_data_changed = checkMissionDataChanged(new_current_auto_mission);

		if (new_current_auto_mission.current_seq < 0) {
			new_current_auto_mission.current_seq = math::constrain(_current_auto_mission.current_seq, INT32_C(0),
						  static_cast<int32_t>(new_current_auto_mission.count) - 1);
		}

		if (new_current_auto_mission.geofence_id != _current_auto_mission.geofence_id) {
			// New geofence data, need to check mission again.
			_mission_checked = false;
		}

		_current_auto_mission = new_current_auto_mission;

		/* Relevant mission items updated externally*/
		if (mission_data_changed) {

			onMissionUpdate(mission_items_changed);
		}

		_is_current_planned_mission_item_valid = isMissionValid();
	}
}

void MissionBase::onMissionUpdate(bool has_mission_items_changed)
{
	if (has_mission_items_changed) {
		_dataman_cache.invalidate();
		_load_mission_index = -1;

		if (canRunMissionFeasibility()) {
			_mission_checked = true;
			check_mission_valid();

		} else {
			_mission_checked = false;
		}
	}

	if (isActive()) {
		_mission_has_been_activated = true;
		_automation->reset_triplets();
		update_mission();
		set_mission_items();

	} else {
		if (has_mission_items_changed) {
			_mission_has_been_activated = false;
		}
	}

	// reset as when we update mission we don't want to proceed at previous index
	_inactivation_index = -1;
}

void
MissionBase::on_inactive()
{
	_land_detected_sub.update();
	_vehicle_status_sub.update();
	_global_pos_sub.update();
	_geofence_status_sub.update();

	parameters_update();

	updateMavlinkMission();

	/* Check the mission */
	if (!_mission_checked && canRunMissionFeasibility()) {
		_mission_checked = true;
		check_mission_valid();
		_is_current_planned_mission_item_valid = isMissionValid();
	}

	if (_vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		_system_disarmed_while_inactive = true;
	}
}

void
MissionBase::on_inactivation()
{
	_automation->disable_camera_trigger();

	_automation->stop_capturing_images();
	_automation->set_gimbal_neutral(); // point forward
	_automation->release_gimbal_control();

	if (_automation->get_precland()->is_activated()) {
		_automation->get_precland()->on_inactivation();
	}

	/* reset so current mission item gets restarted if mission was paused */
	_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

	/* reset so MISSION_ITEM_REACHED isn't published */
	_automation->get_task_result()->seq_reached = -1;

	_mission_type = MissionType::MISSION_TYPE_NONE;

	_inactivation_index = _current_auto_mission.current_seq;
}

void
MissionBase::on_activation()
{
	/* reset the current mission to the start sequence if needed.*/
	checkMissionRestart();

	_mission_has_been_activated = true;
	_system_disarmed_while_inactive = false;

	update_mission();

	// reset the cache and fill it with the items up to the previous item. The cache contains
	// commands that are valid for the whole mission, not just a single waypoint.
	if (_current_auto_mission.current_seq > 0) {
		resetItemCache();
		updateCachedItemsUpToIndex(_current_auto_mission.current_seq - 1);
	}

	int32_t resume_index = _inactivation_index > 0 ? _inactivation_index : 0;

	bool resume_mission_on_previous = false;

	if (_inactivation_index > 0 && cameraWasTriggering()) {
		size_t num_found_items{0U};
		getPreviousPositionItems(_inactivation_index - 1, &resume_index, num_found_items, 1U);

		if (num_found_items == 1U) {
			// The mission we are resuming had camera triggering enabled. In order to not lose any images
			// we restart the mission at the previous position item.
			// We will replay the cached commands once we reach the previous position item and have yaw aligned.
			setMissionIndex(resume_index);

			_align_heading_necessary = true;
			resume_mission_on_previous = true;
		}
	}

	if (!resume_mission_on_previous) {
		// Only replay speed changes immediately if we are not resuming the mission at the previous position item.
		// Otherwise it must be handled in the on_active() method once we reach the previous position item.
		replayCachedSpeedChangeItems();
		_speed_replayed_on_activation = true;

	} else {
		_speed_replayed_on_activation = false;
	}

	checkClimbRequired(_current_auto_mission.current_seq);
	set_mission_items();

	_mission_activation_index = _current_auto_mission.current_seq;
	_inactivation_index = -1; // reset

	// reset cruise speed
	_automation->reset_cruising_speed();
}

void
MissionBase::on_active()
{
	_land_detected_sub.update();
	_vehicle_status_sub.update();
	_global_pos_sub.update();
	_geofence_status_sub.update();

	parameters_update();

	updateMavlinkMission();
	updateDatamanCache();
	updateMissionAltAfterHomeChanged();

	/* Check the mission */
	if (!_mission_checked && canRunMissionFeasibility()) {
		_mission_checked = true;
		check_mission_valid();
		_is_current_planned_mission_item_valid = isMissionValid();
		update_mission();
		set_mission_items();
	}

	// check if heading alignment is necessary, and add it to the current mission item if necessary
	if (_align_heading_necessary && is_current_task_item_reached_or_completed()) {

		// add yaw alignment requirement on the current mission item
		int32_t next_mission_item_index;
		size_t num_found_items{0U};
		getNextPositionItems(_current_auto_mission.current_seq + 1, &next_mission_item_index, num_found_items, 1U);

		if (num_found_items == 1U && !PX4_ISFINITE(_current_task_item.yaw)) {
			task_item_s next_position_current_task_item;
			const dm_item_t mission_dataman_id = static_cast<dm_item_t>(_current_auto_mission.mission_dataman_id);
			bool success = _dataman_cache.loadWait(mission_dataman_id, next_mission_item_index,
							       reinterpret_cast<uint8_t *>(&next_position_current_task_item), sizeof(next_position_current_task_item), MAX_DATAMAN_LOAD_WAIT);

			if (success) {
				_current_task_item.yaw = matrix::wrap_pi(get_bearing_to_next_waypoint(_current_task_item.lat, _current_task_item.lon,
								    next_position_current_task_item.lat, next_position_current_task_item.lon));
				_current_task_item.force_heading = true; // note: doesn't have effect in fixed-wing mode
			}
		}

		task_item_to_position_setpoint(_current_task_item, &_automation->get_position_setpoint_triplet()->current);

		reset_current_task_item_reached();

		_automation->set_position_setpoint_triplet_updated();
		_align_heading_necessary = false;
	}

	// Replay camera mode commands immediately upon mission resume
	if (haveCachedCameraModeItems()) {
		replayCachedCameraModeItems();
	}

	// Replay cached gimbal commands immediately upon mission resume, but only after the vehicle has reached the final target altitude
	if (haveCachedGimbalItems() && _work_item_type != WorkItemType::WORK_ITEM_TYPE_CLIMB) {
		replayCachedGimbalItems();
	}

	// Replay cached trigger commands once the last mission waypoint is re-reached after the mission resume
	if (_current_auto_mission.current_seq > _mission_activation_index) {
		// replay trigger commands
		if (cameraWasTriggering()) {
			replayCachedTriggerItems();
		}
	}

	if (!_speed_replayed_on_activation && _current_auto_mission.current_seq > _mission_activation_index) {
		// replay speed change items if not already done on mission (re-)activation
		replayCachedSpeedChangeItems();
	}

	/* lets check if we reached the current mission item */
	if (_mission_type != MissionType::MISSION_TYPE_NONE && is_current_task_item_reached_or_completed()) {
		/* If we just completed a takeoff which was inserted before the right waypoint,
		   there is no need to report that we reached it because we didn't. */
		if (_work_item_type != WorkItemType::WORK_ITEM_TYPE_CLIMB) {
			set_mission_item_reached();
		}

		if (_current_task_item.autocontinue) {
			/* switch to next waypoint if 'autocontinue' flag set */
			advance_mission();
			set_mission_items();
		}
	}

	/* see if we need to update the current yaw heading */
	if (!_param_mis_mnt_yaw_ctl.get()
	    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
	    && (_automation->get_vroi().mode != vehicle_roi_s::ROI_NONE)
	    && !(_current_task_item.nav_cmd == NAV_CMD_TAKEOFF
		 || _current_task_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
		 || _current_task_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
		 || _current_task_item.nav_cmd == NAV_CMD_LAND
		 || _current_task_item.nav_cmd == NAV_CMD_VTOL_LAND
		 || _work_item_type == WorkItemType::WORK_ITEM_TYPE_ALIGN_HEADING)) {
		// Mount control is disabled If the vehicle is in ROI-mode, the vehicle
		// needs to rotate such that ROI is in the field of view.
		// ROI only makes sense for multicopters.
		heading_sp_update();
	}

	// TODO: Add vtol heading update method if required.
	// Question: Why does vtol ever have to update heading?

	/* check if landing needs to be aborted */
	if ((_current_task_item.nav_cmd == NAV_CMD_LAND)
	    && (_automation->abort_landing())) {

		do_abort_landing();
	}

	if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND) {
		_automation->get_precland()->on_active();

	} else if (_automation->get_precland()->is_activated()) {
		_automation->get_precland()->on_inactivation();
	}

	updateAltToAvoidTerrainCollisionAndRepublishTriplet(_current_task_item);
}

bool
MissionBase::isLanding()
{
	if (hasMissionLandStart() &&
	(_current_auto_mission.current_seq > _current_auto_mission.land_start_index)) {

		static constexpr size_t max_num_next_items{1u};
		int32_t next_task_items_index[max_num_next_items];
		size_t num_found_items;

		getNextPositionItems(_current_auto_mission.land_start_index + 1, next_task_items_index, num_found_items, max_num_next_items);

		// vehicle is currently landing if
		//  mission valid, still flying, and in the landing portion of mission (past land start marker)
		bool on_landing_stage = (num_found_items > 0U) &&  _current_auto_mission.current_seq > next_task_items_index[0U];

		// special case: if the land start index is at a LOITER_TO_ALT WP, then we're in the landing sequence already when the
		// distance to the WP is below the loiter radius + acceptance.
		if ((num_found_items > 0U) && _current_auto_mission.current_seq == next_task_items_index[0U]
		    && _current_task_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
			const float d_current = get_distance_to_next_waypoint(_current_task_item.lat, _current_task_item.lon,
						_automation->get_global_position()->lat, _automation->get_global_position()->lon);

			// consider task_item.loiter_radius invalid if NAN or 0, use default value in this case.
			const float mission_item_loiter_radius_abs = (PX4_ISFINITE(_current_task_item.loiter_radius)
					&& fabsf(_current_task_item.loiter_radius) > FLT_EPSILON) ? fabsf(_current_task_item.loiter_radius) :
					_automation->get_loiter_radius();

			on_landing_stage = d_current <= (_automation->get_acceptance_radius() + mission_item_loiter_radius_abs);
		}

		return _automation->get_task_result()->valid && on_landing_stage;

	} else {
		return false;
	}
}

void MissionBase::update_mission()
{
	if (_current_auto_mission.count == 0u || !_is_current_planned_mission_item_valid || !isMissionValid()) {
		if (_land_detected_sub.get().landed) {
			/* landed, refusing to take off without a mission */
			mavlink_log_critical(_automation->get_mavlink_log_pub(), "No valid mission available, refusing takeoff\t");
			events::send(events::ID("mission_not_valid_refuse"), {events::Log::Error, events::LogInternal::Disabled},
				     "No valid mission available, refusing takeoff");

		} else {
			mavlink_log_critical(_automation->get_mavlink_log_pub(), "No valid mission available, loitering\t");
			events::send(events::ID("mission_not_valid_loiter"), {events::Log::Error, events::LogInternal::Disabled},
				     "No valid mission available, loitering");
		}

		_mission_type = MissionType::MISSION_TYPE_NONE;

	} else {
		if (_mission_type == MissionType::MISSION_TYPE_NONE) {
			mavlink_log_info(_automation->get_mavlink_log_pub(), "Executing Mission\t");
			events::send(events::ID("mission_execute"), events::Log::Info, "Executing Mission");
		}

		_mission_type = MissionType::MISSION_TYPE_MISSION;
	}

	/* Reset vehicle_roi
	 * Missions that do not explicitly configure ROI would not override
	 * an existing ROI setting from previous missions */
	_automation->reset_vroi();

	if (_automation->get_task_result()->valid) {
		/* reset work item if new mission has been accepted */
		_work_item_type = WorkItemType::WORK_ITEM_TYPE_DEFAULT;

		/* reset mission failure if we have an updated valid mission */
		_automation->get_task_result()->failure = false;

		/* reset sequence info as well */
		_automation->get_task_result()->seq_reached = -1;
		_automation->get_task_result()->seq_total = _current_auto_mission.count;
	}

	// we start from the first item so can reset the cache
	if (_current_auto_mission.current_seq == 0) {
		resetItemCache();
	}

	set_mission_result();
}

void
MissionBase::advance_mission()
{
	/* do not advance mission item if we're processing sub mission work items */
	if (_work_item_type != WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		return;
	}

	if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
		_is_current_planned_mission_item_valid = setNextMissionItem();

		if (!_is_current_planned_mission_item_valid) {
			// Mission ended
			if (_land_detected_sub.get().landed) {
				mavlink_log_info(_automation->get_mavlink_log_pub(), "Mission finished, landed\t");

				events::send(events::ID("mission_finished"), events::Log::Info, "Mission finished, landed");

			} else {
				/* https://en.wikipedia.org/wiki/Loiter_(aeronautics) */
				mavlink_log_info(_automation->get_mavlink_log_pub(), "Mission finished, loitering\t");

				events::send(events::ID("mission_finished_loiter"), events::Log::Info, "Mission finished, loitering");
			}

			// Reset jump counter if the mission was completed
			if ((_current_auto_mission.current_seq + 1) == _current_auto_mission.count) {
				resetMissionJumpCounter();
			}
		}
	}
}

void
MissionBase::set_mission_items()
{
	bool set_end_of_mission{false};

	if (_is_current_planned_mission_item_valid && _mission_type == MissionType::MISSION_TYPE_MISSION && isMissionValid()) {
		/* By default set the mission item to the current planned mission item. Depending on request, it can be altered. */
		if (loadCurrentTaskItem()) {
			/* force vtol land */
			if (_automation->force_vtol() && _current_task_item.nav_cmd == NAV_CMD_LAND) {
				_current_task_item.nav_cmd = NAV_CMD_VTOL_LAND;
			}

			setActiveMissionItems();

		} else {
			set_end_of_current_auto_mission = true;
		}

	} else {
		set_end_of_current_auto_mission = true;
	}

	if (set_end_of_current_auto_mission) {
		setEndOfMissionItems();
		_automation->mode_completed(getAutomationStateId());
	}
}

bool MissionBase::loadCurrentTaskItem()
{
	const dm_item_t dm_item = static_cast<dm_item_t>(_current_auto_mission.mission_dataman_id);
	bool success = _dataman_cache.loadWait(dm_item, _current_auto_mission.current_seq, reinterpret_cast<uint8_t *>(&_current_task_item),
					       sizeof(task_item_s), MAX_DATAMAN_LOAD_WAIT);

	if (!success) {
		mavlink_log_critical(_automation->get_mavlink_log_pub(), "Mission item could not be set.\t");
		events::send(events::ID("task_item_set_failed"), events::Log::Error,
			     "Mission item could not be set");
	}

	return success;
}

void MissionBase::setEndOfMissionItems()
{
	position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();

	if (_land_detected_sub.get().landed) {
		_current_task_item.nav_cmd = NAV_CMD_IDLE;

	} else {
		if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
			setLoiterItemFromCurrentPositionSetpoint(&_current_task_item);

		} else {
			setLoiterItemFromCurrentPosition(&_current_task_item);
		}
	}

	/* update position setpoint triplet  */
	pos_sp_triplet->previous.valid = false;
	task_item_to_position_setpoint(_current_task_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	// set mission finished
	_automation->get_task_result()->finished = true;
	_automation->set_task_result_updated();

	publish_automation_current_task_item(); // for logging
	_automation->set_position_setpoint_triplet_updated();

	_mission_type = MissionType::MISSION_TYPE_NONE;
}

void
MissionBase::set_mission_item_reached()
{
	_automation->get_task_result()->seq_reached = _current_auto_mission.current_seq;
	_automation->set_task_result_updated();

	reset_current_task_item_reached();
}

void
MissionBase::set_mission_result()
{
	_automation->get_task_result()->finished = false;
	_automation->get_task_result()->seq_current = _current_auto_mission.current_seq > 0 ? _current_auto_mission.current_seq : 0;

	_automation->set_task_result_updated();
}

bool MissionBase::do_need_move_to_item()
{
	float d_current = get_distance_to_next_waypoint(_current_task_item.lat, _current_task_item.lon,
			  _global_pos_sub.get().lat, _global_pos_sub.get().lon);

	return d_current > _automation->get_acceptance_radius();
}

void MissionBase::handleLanding(WorkItemType &new_work_item_type, task_item_s next_task_items[],
				size_t &num_found_items)
{
	position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();

	bool needs_to_land = !_land_detected_sub.get().landed &&
			     ((_current_task_item.nav_cmd == NAV_CMD_VTOL_LAND)
			      || (_current_task_item.nav_cmd == NAV_CMD_LAND));

	bool needs_vtol_landing = _vehicle_status_sub.get().is_vtol &&
				  (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) &&
				  (_current_task_item.nav_cmd == NAV_CMD_VTOL_LAND) &&
				  !_land_detected_sub.get().landed;

	/* ignore yaw for landing items */
	/* XXX: if specified heading for landing is desired we could add another step before the descent
		* that aligns the vehicle first */
	if (_current_task_item.nav_cmd == NAV_CMD_LAND || _current_task_item.nav_cmd == NAV_CMD_VTOL_LAND) {
		_current_task_item.yaw = NAN;
	}

	/* move to land wp as fixed wing */
	if (needs_vtol_landing) {
		if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {

			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

			/* use current mission item as next position item */
			num_found_items = 1u;
			next_task_items[0u] = _current_task_item;

			float altitude = _global_pos_sub.get().alt;

			if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
				altitude = pos_sp_triplet->current.alt;
			}

			_current_task_item.altitude = altitude;
			_current_task_item.altitude_is_relative = false;
			_current_task_item.nav_cmd = NAV_CMD_WAYPOINT;
			_current_task_item.autocontinue = true;
			_current_task_item.time_inside = 0.0f;
			_current_task_item.vtol_back_transition = true;

			_automation->reset_position_setpoint(pos_sp_triplet->previous);
		}

		/* transition to MC */
		if (_work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) {

			set_vtol_transition_item(&_current_task_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION;
		}

	} else if (needs_to_land) {
		/* move to landing waypoint before descent if necessary */
		if ((_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) &&
		    do_need_move_to_item() &&
		    (_work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT ||
		     _work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION)) {

			new_work_item_type = WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

			/* use current mission item as next position item */
			num_found_items = 1u;
			next_task_items[0u] = _current_task_item;

			/*
				* Ignoring waypoint altitude:
				* Set altitude to the same as we have now to prevent descending too fast into
				* the ground. Actual landing will descend anyway until it touches down.
				* XXX: We might want to change that at some point if it is clear to the user
				* what the altitude means on this waypoint type.
				*/
			float altitude = _global_pos_sub.get().alt;

			if (pos_sp_triplet->current.valid
			    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
				altitude = pos_sp_triplet->current.alt;
			}

			_current_task_item.altitude = altitude;
			_current_task_item.altitude_is_relative = false;
			_current_task_item.nav_cmd = NAV_CMD_WAYPOINT;
			_current_task_item.autocontinue = true;

			// have to reset here because these field were used in set_vtol_transition_item
			_current_task_item.time_inside = 0.f;
			_current_task_item.acceptance_radius = _automation->get_acceptance_radius();

			// make previous setpoint invalid, such that there will be no prev-current line following.
			// if the vehicle drifted off the path during back-transition it should just go straight to the landing point
			_automation->reset_position_setpoint(pos_sp_triplet->previous);

			// set gimbal to neutral position (level with horizon) to reduce change of damage on landing
			_automation->acquire_gimbal_control();
			_automation->set_gimbal_neutral();
			_automation->release_gimbal_control();

		} else {

			if (_current_task_item.land_precision > 0 && _current_task_item.land_precision < 3) {
				new_work_item_type = WorkItemType::WORK_ITEM_TYPE_PRECISION_LAND;

				startPrecLand(_current_task_item.land_precision);
			}
		}
	}
}

bool MissionBase::position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const
{
	return ((p1->valid == p2->valid) &&
		(p1->type == p2->type) &&
		(fabsf(p1->vx - p2->vx) < FLT_EPSILON) &&
		(fabsf(p1->vy - p2->vy) < FLT_EPSILON) &&
		(fabsf(p1->vz - p2->vz) < FLT_EPSILON) &&
		(fabs(p1->lat - p2->lat) < DBL_EPSILON) &&
		(fabs(p1->lon - p2->lon) < DBL_EPSILON) &&
		(fabsf(p1->alt - p2->alt) < FLT_EPSILON) &&
		((fabsf(p1->yaw - p2->yaw) < FLT_EPSILON) || (!PX4_ISFINITE(p1->yaw) && !PX4_ISFINITE(p2->yaw))) &&
		(fabsf(p1->loiter_radius - p2->loiter_radius) < FLT_EPSILON) &&
		(p1->loiter_direction_counter_clockwise == p2->loiter_direction_counter_clockwise) &&
		(fabsf(p1->acceptance_radius - p2->acceptance_radius) < FLT_EPSILON) &&
		(fabsf(p1->cruising_speed - p2->cruising_speed) < FLT_EPSILON) &&
		((fabsf(p1->cruising_throttle - p2->cruising_throttle) < FLT_EPSILON) || (!PX4_ISFINITE(p1->cruising_throttle)
				&& !PX4_ISFINITE(p2->cruising_throttle))));

}

void
MissionBase::report_do_jump_mission_changed(int index, int do_jumps_remaining)
{
	/* inform about the change */
	_automation->get_task_result()->item_do_jump_changed = true;
	_automation->get_task_result()->item_changed_index = index;
	_automation->get_task_result()->item_do_jump_remaining = do_jumps_remaining;

	_automation->set_task_result_updated();
}

void
MissionBase::checkMissionRestart()
{
	if (_system_disarmed_while_inactive && _mission_has_been_activated && (_current_auto_mission.count > 0U)
	    && ((_current_auto_mission.current_seq + 1) == _current_auto_mission.count)) {
		setMissionIndex(0);
		_inactivation_index = -1; // reset
		_is_current_planned_mission_item_valid = isMissionValid();
		resetMissionJumpCounter();
		_automation->reset_cruising_speed();
		_automation->reset_vroi();
		set_mission_result();
	}
}

void
MissionBase::check_mission_valid(bool forced)
{
	// Allow forcing it, since we currently not rechecking if parameters have changed.
	if (forced ||
	    (_automation->get_task_result()->mission_id != _current_auto_mission.mission_id) ||
	    (_automation->get_task_result()->geofence_id != _current_auto_mission.geofence_id) ||
	    (_automation->get_task_result()->home_position_counter != _automation->get_home_position()->update_count)) {

		_automation->get_task_result()->mission_id = _current_auto_mission.mission_id;
		_automation->get_task_result()->geofence_id = _current_auto_mission.geofence_id;
		_automation->get_task_result()->home_position_counter = _automation->get_home_position()->update_count;

		MissionFeasibilityChecker missionFeasibilityChecker(_automation, _dataman_client);
		_automation->get_task_result()->valid = missionFeasibilityChecker.checkMissionFeasible(_current_auto_mission);
		_automation->get_task_result()->seq_total = _current_auto_mission.count;
		_automation->get_task_result()->seq_reached = -1;
		_automation->get_task_result()->failure = false;

		set_mission_result();

		// only warn if the check failed on merit
		if ((!_automation->get_task_result()->valid) && _current_auto_mission.count > 0U) {
			PX4_WARN("mission check failed");
		}

	}
}

void
MissionBase::heading_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet =
		_automation->get_position_setpoint_triplet();

	// Only update if current triplet is valid
	if (pos_sp_triplet->current.valid) {

		double point_from_latlon[2] = { _global_pos_sub.get().lat,
						_global_pos_sub.get().lon
					      };
		double point_to_latlon[2] = { _global_pos_sub.get().lat,
					      _global_pos_sub.get().lon
					    };
		float yaw_offset = 0.0f;

		// Depending on ROI-mode, update heading
		switch (_automation->get_vroi().mode) {
		case vehicle_roi_s::ROI_LOCATION: {
				// ROI is a fixed location. Vehicle needs to point towards that location
				point_to_latlon[0] = _automation->get_vroi().lat;
				point_to_latlon[1] = _automation->get_vroi().lon;
				// No yaw offset required
				yaw_offset = 0.0f;
				break;
			}

		case vehicle_roi_s::ROI_WPNEXT: {
				// ROI is current waypoint. Vehcile needs to point towards current waypoint
				point_to_latlon[0] = pos_sp_triplet->current.lat;
				point_to_latlon[1] = pos_sp_triplet->current.lon;
				// Add the gimbal's yaw offset
				yaw_offset = _automation->get_vroi().yaw_offset;
				break;
			}

		case vehicle_roi_s::ROI_NONE:
		case vehicle_roi_s::ROI_WPINDEX:
		case vehicle_roi_s::ROI_TARGET:
		case vehicle_roi_s::ROI_ENUM_END:
		default: {
				return;
			}
		}

		// Get desired heading and update it.
		// However, only update if distance to desired heading is
		// larger than acceptance radius to prevent excessive yawing
		float d_current = get_distance_to_next_waypoint(point_from_latlon[0],
				  point_from_latlon[1], point_to_latlon[0], point_to_latlon[1]);

		if (d_current > _automation->get_acceptance_radius()) {
			float yaw = matrix::wrap_pi(
					    get_bearing_to_next_waypoint(point_from_latlon[0],
							    point_from_latlon[1], point_to_latlon[0],
							    point_to_latlon[1]) + yaw_offset);

			_current_task_item.yaw = yaw;
			pos_sp_triplet->current.yaw = _current_task_item.yaw;

		} else {
			if (!PX4_ISFINITE(pos_sp_triplet->current.yaw)) {
				_current_task_item.yaw = NAN;
				pos_sp_triplet->current.yaw = _current_task_item.yaw;
			}
		}

		// we set yaw directly so we can run this in parallel to the FOH update
		publish_automation_current_task_item();
		_automation->set_position_setpoint_triplet_updated();
	}
}

void
MissionBase::do_abort_landing()
{
	// Abort FW landing, loiter above landing site in at least MIS_LND_ABRT_ALT
	if (_mission_type == MissionType::MISSION_TYPE_NONE) {
		return;
	}

	if (_current_task_item.nav_cmd != NAV_CMD_LAND) {
		return;
	}

	const float alt_landing = get_absolute_altitude_for_item(_current_task_item);
	const float alt_sp = math::max(alt_landing + _automation->get_landing_abort_min_alt(),
				       _global_pos_sub.get().alt);

	// turn current landing waypoint into an indefinite loiter
	_current_task_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_current_task_item.altitude_is_relative = false;
	_current_task_item.altitude = alt_sp;
	_current_task_item.loiter_radius = _automation->get_loiter_radius();
	_current_task_item.acceptance_radius = _automation->get_acceptance_radius();
	_current_task_item.autocontinue = false;
	_current_task_item.origin = ORIGIN_ONBOARD;

	task_item_to_position_setpoint(_current_task_item, &_automation->get_position_setpoint_triplet()->current);

	// XXX: this is a hack to invalidate the "next" position setpoint for the fixed-wing position controller during
	// the landing abort hold. otherwise, the "next" setpoint would still register as a "LAND" point, and trigger
	// the early landing configuration (flaps and landing airspeed) during the hold.
	_automation->get_position_setpoint_triplet()->next.lat = (double)NAN;
	_automation->get_position_setpoint_triplet()->next.lon = (double)NAN;
	_automation->get_position_setpoint_triplet()->next.alt = NAN;

	publish_automation_current_task_item(); // for logging
	_automation->set_position_setpoint_triplet_updated();

	mavlink_log_info(_automation->get_mavlink_log_pub(), "Holding at %d m above landing waypoint.\t",
			 (int)(alt_sp - alt_landing));
	events::send<float>(events::ID("mission_holding_above_landing"), events::Log::Info,
			    "Holding at {1:.0m_v} above landing waypoint", alt_sp - alt_landing);

	// reset mission index to start of landing
	if (hasMissionLandStart()) {
		_is_current_planned_mission_item_valid = true;
		setMissionIndex(_current_auto_mission.land_start_index);

	} else {
		// move mission index back (landing approach point)
		_is_current_planned_mission_item_valid = (goToPreviousItem(false) == PX4_OK);
	}

	// send reposition cmd to get out of mission
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
	vehicle_command.param1 = -1.f; // Default speed
	vehicle_command.param2 = 1.f; // Modes should switch, not setting this is unsupported
	vehicle_command.param5 = _current_task_item.lat;
	vehicle_command.param6 = _current_task_item.lon;
	vehicle_command.param7 = alt_sp;
	_automation->publish_vehicle_command(vehicle_command);
}

void MissionBase::publish_automation_current_task_item()
{
	navigator_current_task_item_s navigator_current_task_item{};

	navigator_current_task_item.sequence_current = _current_auto_mission.current_seq;
	navigator_current_task_item.nav_cmd = _current_task_item.nav_cmd;
	navigator_current_task_item.latitude = _current_task_item.lat;
	navigator_current_task_item.longitude = _current_task_item.lon;
	navigator_current_task_item.altitude = _current_task_item.altitude;

	navigator_current_task_item.time_inside = get_time_inside(_current_task_item);
	navigator_current_task_item.acceptance_radius = _current_task_item.acceptance_radius;
	navigator_current_task_item.loiter_radius = _current_task_item.loiter_radius;
	navigator_current_task_item.yaw = _current_task_item.yaw;

	navigator_current_task_item.frame = _current_task_item.frame;
	navigator_current_task_item.frame = _current_task_item.origin;

	navigator_current_task_item.loiter_exit_xtrack = _current_task_item.loiter_exit_xtrack;
	navigator_current_task_item.force_heading = _current_task_item.force_heading;
	navigator_current_task_item.altitude_is_relative = _current_task_item.altitude_is_relative;
	navigator_current_task_item.autocontinue = _current_task_item.autocontinue;
	navigator_current_task_item.vtol_back_transition = _current_task_item.vtol_back_transition;

	navigator_current_task_item.timestamp = hrt_absolute_time();

	_automation_mission_item_pub.publish(navigator_current_task_item);
}

bool MissionBase::isMissionValid() const
{
	bool ret_val{false};

	if (((_current_auto_mission.current_seq < _current_auto_mission.count) || (_current_auto_mission.count == 0U && _current_auto_mission.current_seq <= 0)) &&
	    (_current_auto_mission.mission_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ||
	     _current_auto_mission.mission_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) &&
	    (_current_auto_mission.timestamp != 0u) &&
	    (_automation->get_task_result()->valid)) {
		ret_val = true;
	}

	return ret_val;
}

int MissionBase::getNonJumpItem(int32_t &mission_index, task_item_s &mission, bool execute_jump,
				bool write_jumps, bool mission_direction_backward)
{
	if (mission_index >= _current_auto_mission.count || mission_index < 0) {
		return PX4_ERROR;
	}

	const dm_item_t mission_dataman_id = (dm_item_t)_current_auto_mission.mission_dataman_id;
	int32_t new_mission_index{mission_index};
	task_item_s new_current_auto_mission;

	for (uint16_t jump_count = 0u; jump_count < MAX_JUMP_ITERATION; jump_count++) {
		/* read mission item from datamanager */
		bool success = _dataman_cache.loadWait(mission_dataman_id, new_mission_index, reinterpret_cast<uint8_t *>(&new_current_auto_mission),
						       sizeof(task_item_s), MAX_DATAMAN_LOAD_WAIT);

		if (!success) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_automation->get_mavlink_log_pub(), "Waypoint could not be read.\t");
			events::send<uint16_t>(events::ID("mission_failed_to_read_wp"), events::Log::Error,
					       "Waypoint {1} could not be read from storage", new_mission_index);
			return PX4_ERROR;
		}

		if (new_current_auto_mission.nav_cmd == NAV_CMD_DO_JUMP) {
			if (new_current_auto_mission.do_jump_mission_index >= _current_auto_mission.count || new_current_auto_mission.do_jump_mission_index < 0) {
				PX4_ERR("Do Jump mission index is out of bounds.");
				return PX4_ERROR;
			}

			if ((new_current_auto_mission.do_jump_current_count < new_current_auto_mission.do_jump_repeat_count) && execute_jump) {
				if (write_jumps) {
					new_current_auto_mission.do_jump_current_count++;
					success = _dataman_cache.writeWait(mission_dataman_id, new_mission_index, reinterpret_cast<uint8_t *>(&new_current_auto_mission),
									   sizeof(struct task_item_s));

					if (!success) {
						/* not supposed to happen unless the datamanager can't access the dataman */
						mavlink_log_critical(_automation->get_mavlink_log_pub(), "DO JUMP waypoint could not be written.\t");
						events::send(events::ID("mission_failed_to_write_do_jump"), events::Log::Error,
							     "DO JUMP waypoint could not be written");
						// Still continue searching for next non jump item.
					}

					report_do_jump_mission_changed(new_mission_index, new_current_auto_mission.do_jump_repeat_count - new_current_auto_mission.do_jump_current_count);
				}

				new_mission_index = new_current_auto_mission.do_jump_mission_index;

			} else {
				if (mission_direction_backward) {
					new_mission_index--;

				} else {
					new_mission_index++;
				}
			}

		} else {
			break;
		}
	}

	mission_index = new_mission_index;
	mission = new_current_auto_mission;

	return PX4_OK;
}

int MissionBase::goToItem(int32_t index, bool execute_jump, bool mission_direction_backward)
{
	task_item_s task_item;

	if (getNonJumpItem(index, task_item, execute_jump, true, mission_direction_backward) == PX4_OK) {
		setMissionIndex(index);
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void MissionBase::setMissionIndex(int32_t index)
{
	if (index != _current_auto_mission.current_seq) {
		_current_auto_mission.current_seq = index;
		_current_auto_mission.timestamp = hrt_absolute_time();
		_mission_pub.publish(_current_auto_mission);
	}
}

void MissionBase::getPreviousPositionItems(int32_t start_index, int32_t items_index[],
		size_t &num_found_items, uint8_t max_num_items)
{
	num_found_items = 0u;

	int32_t next_mission_index{start_index};

	for (size_t item_idx = 0u; item_idx < max_num_items; item_idx++) {
		if (next_mission_index < 0) {
			break;
		}

		task_item_s next_current_task_item;
		bool found_next_item{false};

		do {
			next_mission_index--;
			found_next_item = getNonJumpItem(next_mission_index, next_current_task_item, true, false, true) == PX4_OK;
		} while (!TaskBlock::item_contains_position(next_current_task_item) && found_next_item);

		if (found_next_item) {
			items_index[item_idx] = next_mission_index;
			num_found_items = item_idx + 1;

		} else {
			break;
		}
	}
}

void MissionBase::getNextPositionItems(int32_t start_index, int32_t items_index[],
				       size_t &num_found_items, uint8_t max_num_items)
{
	// Make sure vector does not contain any preexisting elements.
	num_found_items = 0u;

	int32_t next_mission_index{start_index};

	for (size_t item_idx = 0u; item_idx < max_num_items; item_idx++) {
		if (next_mission_index >= _current_auto_mission.count) {
			break;
		}

		task_item_s next_current_task_item;
		bool found_next_item{false};

		do {
			found_next_item = getNonJumpItem(next_mission_index, next_current_task_item, true, false, false) == PX4_OK;
			next_mission_index++;
		} while (!TaskBlock::item_contains_position(next_current_task_item) && found_next_item);

		if (found_next_item) {
			items_index[item_idx] = math::max(next_mission_index - 1,
							  static_cast<int32_t>(0)); // subtract 1 to get the index of the first position item
			num_found_items = item_idx + 1;

		} else {
			break;
		}
	}
}

int MissionBase::goToNextItem(bool execute_jump)
{
	if (_current_auto_mission.current_seq + 1 >= (_current_auto_mission.count)) {
		return PX4_ERROR;
	}

	return goToItem(_current_auto_mission.current_seq + 1, execute_jump);
}

int MissionBase::goToPreviousItem(bool execute_jump)
{
	if (_current_auto_mission.current_seq <= 0) {
		return PX4_ERROR;
	}

	return goToItem(_current_auto_mission.current_seq - 1, execute_jump, true);
}

int MissionBase::goToPreviousPositionItem(bool execute_jump)
{
	size_t num_found_items{0U};
	int32_t previous_position_item_index;
	getPreviousPositionItems(_current_auto_mission.current_seq, &previous_position_item_index, num_found_items, 1);

	if (num_found_items == 1U) {
		setMissionIndex(previous_position_item_index);
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

int MissionBase::goToNextPositionItem(bool execute_jump)
{
	size_t num_found_items{0U};
	int32_t next_position_item_index;
	getNextPositionItems(_current_auto_mission.current_seq + 1, &next_position_item_index, num_found_items, 1);

	if (num_found_items == 1U) {
		setMissionIndex(next_position_item_index);
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

int MissionBase::setMissionToClosestItem(double lat, double lon, float alt, float home_alt,
		const vehicle_status_s &vehicle_status)
{
	int32_t min_dist_index(-1);
	float min_dist(FLT_MAX), dist_xy(FLT_MAX), dist_z(FLT_MAX);
	const dm_item_t mission_dataman_id = static_cast<dm_item_t>(_current_auto_mission.mission_dataman_id);

	for (int32_t mission_item_index = 0; mission_item_index < _current_auto_mission.count; mission_item_index++) {
		task_item_s mission;

		bool success = _dataman_cache.loadWait(mission_dataman_id, mission_item_index, reinterpret_cast<uint8_t *>(&mission),
						       sizeof(task_item_s), MAX_DATAMAN_LOAD_WAIT);

		if (!success) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_automation->get_mavlink_log_pub(), "Could not set mission closest to position.\t");
			events::send(events::ID("mission_failed_set_closest"), events::Log::Error,
				     "Could not set mission closest to position");
			return PX4_ERROR;
		}

		if (TaskBlock::item_contains_position(mission)) {
			// do not consider land waypoints for a fw
			if (!((mission.nav_cmd == NAV_CMD_LAND) &&
			      (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) &&
			      (!vehicle_status.is_vtol))) {
				float dist = get_distance_to_point_global_wgs84(mission.lat, mission.lon,
						TaskBlock::get_absolute_altitude_for_item(mission, home_alt),
						lat,
						lon,
						alt,
						&dist_xy, &dist_z);

				if (dist < min_dist) {
					min_dist = dist;
					min_dist_index = mission_item_index;
				}
			}
		}
	}

	setMissionIndex(min_dist_index);

	return PX4_OK;
}

void MissionBase::resetMission()
{
	/* we do not need to reset mission if is already.*/
	if (_current_auto_mission.count == 0u) {
		return;
	}

	/* Set a new mission*/
	_current_auto_mission.timestamp = hrt_absolute_time();
	_current_auto_mission.current_seq = 0;
	_current_auto_mission.land_start_index = -1;
	_current_auto_mission.land_index = -1;
	_current_auto_mission.count = 0u;
	_current_auto_mission.mission_id = 0u;
	_current_auto_mission.mission_dataman_id = _current_auto_mission.mission_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ?
				      DM_KEY_WAYPOINTS_OFFBOARD_1 :
				      DM_KEY_WAYPOINTS_OFFBOARD_0;

	bool success = _dataman_client.writeSync(DM_KEY_MISSION_STATE, 0, reinterpret_cast<uint8_t *>(&_current_auto_mission),
			sizeof(mission_s));

	if (success) {
		_mission_pub.publish(_current_auto_mission);

	} else {
		PX4_ERR("Mission Initialization failed.");
	}
}

void MissionBase::resetMissionJumpCounter()
{
	const dm_item_t mission_dataman_id = static_cast<dm_item_t>(_current_auto_mission.mission_dataman_id);

	for (size_t mission_index = 0u; mission_index < _current_auto_mission.count; mission_index++) {
		task_item_s task_item;

		bool success = _dataman_client.readSync(mission_dataman_id, mission_index, reinterpret_cast<uint8_t *>(&task_item),
							sizeof(task_item_s), MAX_DATAMAN_LOAD_WAIT);

		if (!success) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_automation->get_mavlink_log_pub(), "Mission could not reset jump count.\t");
			events::send(events::ID("mission_failed_set_jump_count"), events::Log::Error,
				     "Mission could not reset jump count");
			break;
		}

		if (task_item.nav_cmd == NAV_CMD_DO_JUMP) {
			task_item.do_jump_current_count = 0u;

			bool write_success = _dataman_cache.writeWait(mission_dataman_id, mission_index,
					     reinterpret_cast<uint8_t *>(&task_item),
					     sizeof(struct task_item_s));

			if (!write_success) {
				PX4_ERR("Could not write mission item for jump count reset.");
				break;
			}
		}
	}
}

void MissionBase::cacheItem(const task_item_s &task_item)
{
	switch (task_item.nav_cmd) {
	case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		_last_gimbal_configure_item = task_item;
		break;

	case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		_last_gimbal_control_item = task_item;
		break;

	case NAV_CMD_SET_CAMERA_MODE:
		_last_camera_mode_item = task_item;
		break;

	case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
	case NAV_CMD_DO_TRIGGER_CONTROL:
	case NAV_CMD_IMAGE_START_CAPTURE:
	case NAV_CMD_IMAGE_STOP_CAPTURE:
		_last_camera_trigger_item = task_item;
		break;

	case NAV_CMD_DO_CHANGE_SPEED:
		_last_speed_change_item = task_item;
		break;

	case NAV_CMD_DO_VTOL_TRANSITION:
		// delete speed changes after a VTOL transition
		_last_speed_change_item = {};
		break;

	default:
		break;
	}
}

void MissionBase::replayCachedGimbalItems()
{
	if (_last_gimbal_configure_item.nav_cmd > 0) {
		issue_command(_last_gimbal_configure_item);
		_last_gimbal_configure_item = {}; // delete cached item
	}

	if (_last_gimbal_control_item.nav_cmd > 0) {
		issue_command(_last_gimbal_control_item);
		_last_gimbal_control_item = {}; // delete cached item
	}
}

void MissionBase::replayCachedCameraModeItems()
{
	if (_last_camera_mode_item.nav_cmd > 0) {
		issue_command(_last_camera_mode_item);
		_last_camera_mode_item = {}; // delete cached item
	}
}

void MissionBase::replayCachedTriggerItems()
{
	if (_last_camera_trigger_item.nav_cmd > 0) {
		issue_command(_last_camera_trigger_item);
		_last_camera_trigger_item = {}; // delete cached item
	}
}

void MissionBase::replayCachedSpeedChangeItems()
{
	if (_last_speed_change_item.nav_cmd == NAV_CMD_DO_CHANGE_SPEED) {
		issue_command(_last_speed_change_item);
		_last_speed_change_item = {}; // delete cached item
	}
}

void MissionBase::resetItemCache()
{
	_last_gimbal_configure_item = {};
	_last_gimbal_control_item = {};
	_last_camera_mode_item = {};
	_last_camera_trigger_item = {};
}

bool MissionBase::haveCachedGimbalItems()
{
	return _last_gimbal_configure_item.nav_cmd > 0 ||
	       _last_gimbal_control_item.nav_cmd > 0;
}

bool MissionBase::haveCachedCameraModeItems()
{
	return _last_camera_mode_item.nav_cmd > 0;
}

bool MissionBase::cameraWasTriggering()
{
	return (_last_camera_trigger_item.nav_cmd == NAV_CMD_DO_TRIGGER_CONTROL
		&& (int)(_last_camera_trigger_item.params[0] + 0.5f) == 1) ||
	       (_last_camera_trigger_item.nav_cmd == NAV_CMD_IMAGE_START_CAPTURE) ||
	       (_last_camera_trigger_item.nav_cmd == NAV_CMD_DO_SET_CAM_TRIGG_DIST
		&& _last_camera_trigger_item.params[0] > FLT_EPSILON);
}

void MissionBase::updateCachedItemsUpToIndex(const int end_index)
{
	for (int i = 0; i <= end_index; i++) {
		task_item_s task_item;
		const dm_item_t dm_current = (dm_item_t)_current_auto_mission.mission_dataman_id;
		bool success = _dataman_client.readSync(dm_current, i, reinterpret_cast<uint8_t *>(&task_item),
							sizeof(task_item), 500_ms);

		if (success) {
			cacheItem(task_item);
		}
	}
}

void MissionBase::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

void MissionBase::checkClimbRequired(int32_t mission_item_index)
{
	int32_t next_mission_item_index;
	size_t num_found_items{0U};
	getNextPositionItems(mission_item_index, &next_mission_item_index, num_found_items, 1U);

	if (num_found_items > 0U) {

		const dm_item_t mission_dataman_id = static_cast<dm_item_t>(_current_auto_mission.mission_dataman_id);
		task_item_s mission;
		_mission_init_climb_altitude_amsl = NAN; // default to NAN, overwrite below if applicable

		const bool success = _dataman_cache.loadWait(mission_dataman_id, next_mission_item_index,
				     reinterpret_cast<uint8_t *>(&mission),
				     sizeof(mission), MAX_DATAMAN_LOAD_WAIT);

		const bool is_fw_and_takeoff = mission.nav_cmd == NAV_CMD_TAKEOFF
					       && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

		// for FW when on a Takeoff item do not require climb before mission, as we need to keep course to takeoff item straight
		if (success && !is_fw_and_takeoff) {
			const float altitude_amsl_next_position_item = TaskBlock::get_absolute_altitude_for_item(mission);
			const float error_below_setpoint = altitude_amsl_next_position_item -
							   _automation->get_global_position()->alt;

			if (error_below_setpoint > _automation->get_altitude_acceptance_radius()) {

				_mission_init_climb_altitude_amsl = altitude_amsl_next_position_item;
			}
		}
	}
}

bool MissionBase::checkMissionDataChanged(mission_s new_current_auto_mission)
{
	/* count and land_index are the same if the mission_id did not change. We do not care about changes in geofence or rally counters.*/
	return ((new_current_auto_mission.mission_dataman_id != _current_auto_mission.mission_dataman_id) ||
		(new_current_auto_mission.mission_id != _current_auto_mission.mission_id) ||
		(new_current_auto_mission.current_seq != _current_auto_mission.current_seq));
}

bool MissionBase::canRunMissionFeasibility()
{
	return _automation->home_global_position_valid() && // Need to have a home position checked
	       _automation->get_global_position()->timestamp > 0 && // Need to have a position, for first waypoint check
	       (_geofence_status_sub.get().timestamp > 0) && // Geofence data must be loaded
	       (_geofence_status_sub.get().geofence_id == _current_auto_mission.geofence_id) &&
	       (_geofence_status_sub.get().status == geofence_status_s::GF_STATUS_READY);
}

void MissionBase::updateMissionAltAfterHomeChanged()
{
	if (_automation->get_home_position()->update_count > _home_update_counter) {
		float new_alt = get_absolute_altitude_for_item(_current_task_item);
		float altitude_diff = new_alt - _automation->get_position_setpoint_triplet()->current.alt;

		if (_automation->get_position_setpoint_triplet()->previous.valid
		    && PX4_ISFINITE(_automation->get_position_setpoint_triplet()->previous.alt)) {
			_automation->get_position_setpoint_triplet()->previous.alt = _automation->get_position_setpoint_triplet()->previous.alt +
					altitude_diff;
		}

		_automation->get_position_setpoint_triplet()->current.alt = _automation->get_position_setpoint_triplet()->current.alt +
				altitude_diff;

		if (_automation->get_position_setpoint_triplet()->next.valid
		    && PX4_ISFINITE(_automation->get_position_setpoint_triplet()->next.alt)) {
			_automation->get_position_setpoint_triplet()->next.alt = _automation->get_position_setpoint_triplet()->next.alt +
					altitude_diff;
		}

		_automation->set_position_setpoint_triplet_updated();
		_home_update_counter = _automation->get_home_position()->update_count;
	}
}
