/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
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
 * @file UserModeIntention.hpp
 *
 * User mode intention management for mode_manager.
 *
 * This class handles the user's intended operation mode, including:
 * - Processing mode change requests from system_manager
 * - Validating mode availability for the current vehicle type
 * - Publishing mode change results for command ACKs
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mode_change_request.h>
#include <uORB/topics/mode_change_result.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_type_config.h>

namespace mode_manager
{

/**
 * @brief Source of mode change request
 */
enum class ModeChangeSource : uint8_t {
	User = mode_change_request_s::SOURCE_USER,
	ModeExecutor = mode_change_request_s::SOURCE_MODE_EXECUTOR,
	Failsafe = mode_change_request_s::SOURCE_FAILSAFE,
	Command = mode_change_request_s::SOURCE_COMMAND
};

/**
 * @brief User mode intention handler for mode_manager
 *
 * Manages the user's intended operation mode. Receives mode change requests
 * via uORB and publishes results.
 */
class UserModeIntention : public ModuleParams
{
public:
	UserModeIntention(ModuleParams *parent);
	~UserModeIntention() = default;

	/**
	 * @brief Process incoming mode change requests
	 *
	 * Call this in the main loop to handle new mode change requests.
	 * @return true if mode was changed, false otherwise
	 */
	bool update();

	/**
	 * @brief Get the current user intended mode
	 */
	uint8_t get() const { return _user_intended_mode; }

	/**
	 * @brief Check if there was a mode change since last check
	 */
	bool getHadModeChangeAndClear()
	{
		bool ret = _had_mode_change;
		_had_mode_change = false;
		return ret;
	}

	/**
	 * @brief Check if there was ever a mode change
	 */
	bool everHadModeChange() const { return _ever_had_mode_change; }

	/**
	 * @brief Called when vehicle disarms - restore last safe mode
	 */
	void onDisarm();

	/**
	 * @brief Directly change mode (for internal use, e.g., failsafe)
	 *
	 * @param mode New operation mode
	 * @param source Source of the change
	 * @param force Force the change even if mode is not available
	 * @return true if change was accepted
	 */
	bool change(uint8_t mode, ModeChangeSource source = ModeChangeSource::User, bool force = false);

private:
	/**
	 * @brief Check if a mode is available for the current vehicle type
	 */
	bool isModeAvailable(uint8_t mode) const;

	/**
	 * @brief Publish mode change result for command ACK
	 */
	void publishResult(const mode_change_request_s &request, uint8_t result_mode, uint8_t result);

	// Subscriptions
	uORB::Subscription _mode_change_request_sub{ORB_ID(mode_change_request)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<vehicle_type_config_s> _vehicle_type_config_sub{ORB_ID(vehicle_type_config)};

	// Publications
	uORB::Publication<mode_change_result_s> _mode_change_result_pub{ORB_ID(mode_change_result)};

	// State
	uint8_t _user_intended_mode{vehicle_status_s::OPERATION_MODE_AUTO_LOITER};
	uint8_t _mode_after_disarm{vehicle_status_s::OPERATION_MODE_AUTO_LOITER};
	bool _had_mode_change{false};
	bool _ever_had_mode_change{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::COM_POSCTL_NAVL>) _param_com_posctl_navl
	);
};

} // namespace mode_manager
