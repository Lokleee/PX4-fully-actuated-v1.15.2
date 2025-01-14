/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Maximum upwards acceleration in climb rate controlled modes
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX, 4.f);

/**
 * Maximum downwards acceleration in climb rate controlled modes
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_DOWN_MAX, 3.f);

/**
 * Manual yaw rate input filter time constant
 *
 * Not used in Stabilized mode
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_TAU, 0.08f);

/**
 * Altitude reference mode
 *
 * Set to 0 to control height relative to the earth frame origin. This origin may move up and down in
 * flight due to sensor drift.
 * Set to 1 to control height relative to estimated distance to ground. The vehicle will move up and down
 * with terrain height variation. Requires a distance to ground sensor. The height controller will
 * revert to using height above origin if the distance to ground estimate becomes invalid as indicated
 * by the local_position.distance_bottom_valid message being false.
 * Set to 2 to control height relative to ground (requires a distance sensor) when stationary and relative
 * to earth frame origin when moving horizontally.
 * The speed threshold is controlled by the MPC_HOLD_MAX_XY parameter.
 *
 * @min 0
 * @max 2
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @value 2 Terrain hold
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_ALT_MODE, 2);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 *
 * Only used with MPC_POS_MODE 0 or MPC_ALT_MODE 2
 *
 * @unit m/s
 * @min 0
 * @max 3
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 *
 * Only used with MPC_ALT_MODE 1
 *
 * @unit m/s
 * @min 0
 * @max 3
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z, 0.6f);

/**
 * Omni-directional attitude setpoint mode
 *
 * Specifies the type of attitude setpoint sent to the attitude controller.
 * The parameter can be set to a normal attitude setpoint, where the tilt
 * of the vehicle (roll and pitch) are calculated from the desired thrust
 * vector. This should be the behavior for the non-omnidirectional vehicles,
 * such as quadrotors and other multirotors with coplanar rotors.
 * The "constant zero tilt" mode enforces zero roll and pitch and can only be
 * used for omnidirectional vehicles. The min-tilt mode enforces zero tilt
 * up to a maximum set acceleration (thrust) and tilts the vehicle as small
 * as possible if the thrust vector is larger than the maximum. The "constant
 * tilt" and "constant roll/pitch" modes enforce a given tilt or roll/pitch
 * for the vehicle. The estimation modes estimate the optimal tilt or roll/pitch
 * to counteract with the external force (e.g., wind).
 *
 * @min 0
 * @max 6
 * @value 0 tilted attitude
 * @value 1 constant zero tilt
 * @value 2 min-tilt attitude
 * @value 3 constant tilt
 * @value 4 constant roll/pitch
 * @value 5 estimate tilt/roll/pitch
 * @value 6 slow attitude change
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(OMNI_ATT_MODE, 0);

/**
 * Maximum direct-force (horizontal) scaled thrust for omnidirectional vehicles
 *
 * Specifies the maximum horizontal thrust compared to the maximum possible
 * thrust generated by the vehicle for an omnidirectional multirotor. The
 * value of this parameter does not affect the behavior if the attitude mode
 * is not set to one of omni-directional modes (if OMNI_ATT_MODE is 0).
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(OMNI_DFC_MAX_THR, 0.15f);
