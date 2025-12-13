/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @@file Modes_generated.cpp
 *
 * Generated file to switch between all required modes
 *
 * @@author Christoph Tobler <christoph@@px4.io>
 */

#include "ModeManager.hpp"
#include "Modes_generated.hpp"

int ModeManager::_initMode(ModeIndex mode_index)
{

	// disable the old mode if there is any
	if (_current_mode.mode) {
		_current_mode.mode->~Mode();
		_current_mode.mode = nullptr;
		_current_mode.index = ModeIndex::None;
	}

	switch (mode_index) {
	case ModeIndex::None:
		// already disabled mode
		break;

@# loop through all requested modes
@[if tasks]@
@[for task in tasks]@
	case ModeIndex::@(task):
		_current_mode.mode = new (&_mode_union.@(task)) Mode@(task)();
		break;

@[end for]@
@[end if]@
	default:
		// invalid mode
		return 1;
	}

	// mode construction succeeded
	_current_mode.index = mode_index;
	return 0;
}
