/***************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file autovla.h
 *
 * Navigator mode for autonomous VLA (Vision-Language-Action) trajectory following
 * for wheel loader robot. Receives VLA trajectory setpoints and generates
 * position setpoints for the flight mode manager.
 *
 * @author PX4 Development Team
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vla_trajectory_setpoint.h>

class Navigator;

class AutoVLA : public MissionBlock, public ModuleParams
{
public:
	AutoVLA(Navigator *navigator);
	~AutoVLA() = default;

	void initialize() override;
	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:
	/**
	 * Update VLA trajectory setpoint
	 */
	void update_vla_trajectory();

	/**
	 * Generate position setpoint triplet from VLA trajectory
	 */
	void generate_position_setpoint();

	/**
	 * Check if VLA trajectory is valid and recent
	 */
	bool is_vla_trajectory_valid() const;

	// Subscription to VLA trajectory setpoint
	uORB::Subscription _vla_trajectory_sub{ORB_ID(vla_trajectory_setpoint)};

	// VLA trajectory data
	vla_trajectory_setpoint_s _vla_trajectory{};

	// Timing
	hrt_abstime _last_vla_update{0};
	static constexpr hrt_abstime VLA_TIMEOUT{500000}; // 500ms timeout

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_AUTOVLA_ACC>) _param_nav_autovla_acc,
		(ParamFloat<px4::params::NAV_AUTOVLA_VEL>) _param_nav_autovla_vel
	)
};
