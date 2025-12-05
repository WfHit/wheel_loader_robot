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
 * @file autovla_params.c
 *
 * Parameters for AutoVLA navigator mode
 *
 * @author PX4 Development Team
 */

/*
 * AutoVLA mode parameters, accessible via MAVLink
 */

/**
 * AutoVLA maximum velocity
 *
 * Maximum velocity for VLA trajectory following in AutoVLA mode.
 *
 * @unit m/s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group Navigator
 */
PARAM_DEFINE_FLOAT(NAV_AUTOVLA_VEL, 1.0f);

/**
 * AutoVLA maximum acceleration
 *
 * Maximum acceleration for VLA trajectory following in AutoVLA mode.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 3.0
 * @decimal 1
 * @increment 0.1
 * @group Navigator
 */
PARAM_DEFINE_FLOAT(NAV_AUTOVLA_ACC, 0.5f);
