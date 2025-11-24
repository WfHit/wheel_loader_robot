############################################################################
#
#   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

px4_add_library(health_and_arming_checks
	common.cpp
	health_and_arming_checks.cpp

	checks/accelerometer_check.cpp
	checks/airspeed_check.cpp
	checks/arm_permission_check.cpp
	checks/baro_check.cpp
	checks/battery_check.cpp
	checks/cpu_resource_check.cpp
	checks/distance_sensor_checks.cpp
	checks/optical_flow_check.cpp
	checks/esc_check.cpp
	checks/estimator_check.cpp
	checks/failure_detector_check.cpp
	checks/flight_time_check.cpp
	checks/geofence_check.cpp
	checks/gyro_check.cpp
	checks/home_position_check.cpp
	checks/imu_consistency_check.cpp
	checks/logger_check.cpp
	checks/magnetometer_check.cpp
	checks/manual_control_check.cpp
	checks/mission_check.cpp
	checks/mode_check.cpp
	checks/navigator_check.cpp
	checks/offboard_check.cpp
	checks/open_drone_id_check.cpp
	checks/parachute_check.cpp
	checks/power_check.cpp
	checks/rc_and_data_link_check.cpp
	checks/rc_calibration_check.cpp
	checks/sdcard_check.cpp
	checks/system_check.cpp
	checks/vtol_check.cpp
	checks/wind_check.cpp

	checks/external_checks.cpp
)
add_dependencies(health_and_arming_checks mode_util)

px4_add_functional_gtest(SRC health_and_arming_checks_test.cpp
	LINKLIBS health_and_arming_checks mode_util
	)
