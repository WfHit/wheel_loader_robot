/****************************************************************************
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
 * @file vla_trajectory_task_params.c
 *
 * Parameters for VLA Trajectory Task
 *
 * @author PX4 Development Team
 */

/**
 * Maximum chassis velocity
 *
 * Maximum linear velocity for VLA trajectory chassis smoothing.
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_CHS_VEL_MAX, 2.0f);

/**
 * Maximum chassis acceleration
 *
 * Maximum linear acceleration for VLA trajectory chassis smoothing.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_CHS_ACC_MAX, 1.0f);

/**
 * Maximum chassis jerk
 *
 * Maximum linear jerk for VLA trajectory chassis smoothing.
 *
 * @unit m/s^3
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_CHS_JERK_MAX, 2.0f);

/**
 * Maximum chassis yaw rate
 *
 * Maximum yaw rate for VLA trajectory chassis smoothing.
 *
 * @unit rad/s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_CHS_YAW_MAX, 0.5f);

/**
 * Maximum chassis yaw acceleration
 *
 * Maximum yaw acceleration for VLA trajectory chassis smoothing.
 *
 * @unit rad/s^2
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_CHS_YACC_MAX, 0.5f);

/**
 * Maximum steering angle
 *
 * Maximum articulation angle for the wheel loader center pivot.
 *
 * @unit rad
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_STR_ANG_MAX, 0.6f);

/**
 * Maximum steering rate
 *
 * Maximum articulation rate for VLA trajectory steering smoothing.
 *
 * @unit rad/s
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_STR_RATE_MAX, 0.3f);

/**
 * Minimum bucket height
 *
 * Minimum bucket height above ground for VLA trajectory.
 *
 * @unit m
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_BKT_HGT_MIN, 0.1f);

/**
 * Maximum bucket height
 *
 * Maximum bucket height above ground for VLA trajectory.
 *
 * @unit m
 * @min 1.0
 * @max 6.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_BKT_HGT_MAX, 4.0f);

/**
 * Maximum bucket height velocity
 *
 * Maximum vertical velocity for bucket height smoothing.
 *
 * @unit m/s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_BKT_VEL_MAX, 0.5f);

/**
 * Maximum bucket height acceleration
 *
 * Maximum vertical acceleration for bucket height smoothing.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 3.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_BKT_ACC_MAX, 0.8f);

/**
 * Maximum bucket height jerk
 *
 * Maximum vertical jerk for bucket height smoothing.
 *
 * @unit m/s^3
 * @min 0.5
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_BKT_JERK_MAX, 2.0f);

/**
 * Minimum tilt angle
 *
 * Minimum bucket tilt angle (curl back).
 *
 * @unit rad
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_TILT_ANG_MIN, -0.5f);

/**
 * Maximum tilt angle
 *
 * Maximum bucket tilt angle (dump forward).
 *
 * @unit rad
 * @min 0.0
 * @max 1.5
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_TILT_ANG_MAX, 0.8f);

/**
 * Maximum tilt angular velocity
 *
 * Maximum angular velocity for VLA trajectory tilt smoothing.
 *
 * @unit rad/s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_TILT_VEL_MAX, 0.5f);

/**
 * Maximum tilt angular acceleration
 *
 * Maximum angular acceleration for VLA trajectory tilt smoothing.
 *
 * @unit rad/s^2
 * @min 0.1
 * @max 3.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_TILT_ACC_MAX, 0.8f);

/**
 * Maximum tilt jerk
 *
 * Maximum angular jerk for VLA trajectory tilt smoothing.
 *
 * @unit rad/s^3
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_TILT_JERK_MAX, 1.5f);

/**
 * Synchronization time margin
 *
 * Additional time margin added to synchronized trajectory duration
 * to ensure all DOFs complete motion together with buffer.
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group VLA Trajectory
 */
PARAM_DEFINE_FLOAT(VTP_SYNC_MARGIN, 0.1f);
