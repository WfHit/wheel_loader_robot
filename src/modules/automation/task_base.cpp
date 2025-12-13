/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file task_base.cpp
 *
 * Base class for different modes in navigator
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "task_base.h"
#include "automation.h"

TaskBase::TaskBase(Automation *automation, uint8_t automation_state_id) :
	_automation(automation),
	_automation_state_id(automation_state_id)
{
	/* set initial mission items */
	on_inactivation();
	on_inactive();
}

void
TaskBase::run(bool active)
{
	if (active) {
		if (!_active) {
			on_activation();

		} else {
			/* periodic updates when active */
			on_active();
			updateFailsafeChecks();
		}

	} else {
		/* periodic updates when inactive */
		if (_active) {
			on_inactivation();

		} else {
			on_inactive();
		}
	}

	_active = active;
}

void
TaskBase::on_inactive()
{
}

void
TaskBase::on_inactivation()
{
}

void
TaskBase::on_activation()
{
	/* invalidate position setpoint by default */
	_automation->get_position_setpoint_triplet()->current.valid = false;
}

void
TaskBase::on_active()
{
}
