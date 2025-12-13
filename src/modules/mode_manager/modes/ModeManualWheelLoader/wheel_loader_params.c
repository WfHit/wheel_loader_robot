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
 * @file wheel_loader_params.c
 *
 * Parameters for wheel loader manual mode
 */

/**
 * Wheel Loader Maximum Velocity
 *
 * Maximum forward/backward velocity in manual mode.
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_MAX_VEL, 2.0f);

/**
 * Wheel Loader Maximum Steering Angle
 *
 * Maximum articulation steering angle.
 *
 * @unit rad
 * @min 0.1
 * @max 1.57
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_MAX_STEER, 0.7f);

/**
 * Wheel Loader Maximum Steering Rate
 *
 * Maximum rate of steering angle change.
 *
 * @unit rad/s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_MAX_STEER_R, 0.5f);

/**
 * Wheel Loader Bucket Height Minimum
 *
 * Minimum bucket height from ground.
 *
 * @unit m
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_BKT_H_MIN, 0.0f);

/**
 * Wheel Loader Bucket Height Maximum
 *
 * Maximum bucket height from ground.
 *
 * @unit m
 * @min 0.5
 * @max 5.0
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_BKT_H_MAX, 3.0f);

/**
 * Wheel Loader Bucket Height Velocity
 *
 * Maximum bucket height change rate in manual mode.
 *
 * @unit m/s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_BKT_H_VEL, 0.5f);

/**
 * Wheel Loader Tilt Angle Minimum
 *
 * Minimum tilt/bucket curl angle (dump position).
 *
 * @unit rad
 * @min -1.57
 * @max 0.0
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_TILT_MIN, -0.5f);

/**
 * Wheel Loader Tilt Angle Maximum
 *
 * Maximum tilt/bucket curl angle (curl position).
 *
 * @unit rad
 * @min 0.0
 * @max 1.57
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_TILT_MAX, 1.0f);

/**
 * Wheel Loader Tilt Velocity
 *
 * Maximum tilt angle change rate in manual mode.
 *
 * @unit rad/s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_TILT_VEL, 1.0f);

/**
 * Wheel Loader Maximum Jerk
 *
 * Maximum jerk for motion smoothing.
 *
 * @unit m/s^3
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_JERK_MAX, 5.0f);

/**
 * Wheel Loader Maximum Acceleration
 *
 * Maximum acceleration for motion smoothing.
 *
 * @unit m/s^2
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Wheel Loader
 */
PARAM_DEFINE_FLOAT(WL_ACCEL_MAX, 2.0f);
