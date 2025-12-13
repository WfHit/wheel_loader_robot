############################################################################
#
#   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(arming)
add_subdirectory(calibration)
add_subdirectory(failsafe)
add_subdirectory(failure_detector)
add_subdirectory(health_and_arming_checks)
add_subdirectory(mode_util)
add_subdirectory(multicopter_throw_launch)

px4_add_module(
	MODULE modules__system_manager
	MAIN system_manager
	COMPILE_FLAGS
	SRCS
		calibration/accelerometer_calibration.cpp
		calibration/airspeed_calibration.cpp
		calibration/baro_calibration.cpp
		calibration/calibration_routines.cpp
		system_manager.cpp
		system_manager_helper.cpp
		calibration/esc_calibration.cpp
		calibration/factory_calibration_storage.cpp
		calibration/gyro_calibration.cpp
		home_position.cpp
		mode_management.cpp
		calibration/level_calibration.cpp
		calibration/lm_fit.cpp
		calibration/mag_calibration.cpp
		calibration/rc_calibration.cpp
		safety.cpp
		user_mode_intention.cpp
		worker_thread.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		arm_authorization
		circuit_breaker
		failsafe
		failure_detector
		geo
		health_and_arming_checks
		hysteresis
		mode_util
		multicopter_throw_launch
		sensor_calibration
		world_magnetic_model
        atmosphere
	)

px4_add_unit_gtest(SRC calibration/mag_calibration_test.cpp LINKLIBS modules__system_manager)
px4_add_functional_gtest(SRC mode_management_test.cpp LINKLIBS modules__system_manager)
