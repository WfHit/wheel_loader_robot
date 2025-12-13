############################################################################
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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


###########################################
# Prepare flight modes
###########################################

# add core flight modes to list
set(modes_all)
list(APPEND modes_all
	Auto
	Descend
	Failsafe
	ManualAcceleration
	ManualAccelerationSlow
	ManualAltitude
	ManualAltitudeSmoothVel
	ManualPosition
	ManualWheelLoader
	Transition
	VLA
)

if(NOT px4_constrained_flash_build)
	list(APPEND modes_all
		AutoFollowTarget
		Orbit
	)
endif()

# set the files to be generated
set(files_to_generate
	Modes_generated.hpp
	Modes_generated.cpp
)

# generate files needed for Flight Tasks
set(python_args
	-t ${modes_all}
	-i ${CMAKE_CURRENT_SOURCE_DIR}/Templates
	-o ${CMAKE_CURRENT_BINARY_DIR}
	-f ${files_to_generate}
)

# generate the files using the python script and template
add_custom_command(
	OUTPUT
		${CMAKE_CURRENT_BINARY_DIR}/Modes_generated.hpp
		${CMAKE_CURRENT_BINARY_DIR}/Modes_generated.cpp
	COMMAND ${PYTHON_EXECUTABLE} generate_modes.py ${python_args}
	COMMENT "Generating Flight Tasks"
	DEPENDS
		Templates/Modes_generated.cpp.em
		Templates/Modes_generated.hpp.em
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)
add_custom_target(flightmodes_generated
	DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Modes_generated.cpp)
add_dependencies(prebuild_targets flightmodes_generated)

add_compile_options(
	-Wno-cast-align
) # TODO: fix and enable

# add subdirectory containing all modes
add_subdirectory(modes)

px4_add_module(
	MODULE modules__mode_manager
	MAIN mode_manager
	COMPILE_FLAGS
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_BINARY_DIR}
	SRCS
		ModeManager.cpp
		ModeManager.hpp

		${CMAKE_CURRENT_BINARY_DIR}/Modes_generated.hpp
		${CMAKE_CURRENT_BINARY_DIR}/Modes_generated.cpp
	DEPENDS
		px4_work_queue
		WeatherVane
		flightmodes_generated
)

# add all flight task dependencies
foreach(task ${modes_all})
	target_link_libraries(modules__mode_manager PUBLIC Mode${task})
endforeach()
