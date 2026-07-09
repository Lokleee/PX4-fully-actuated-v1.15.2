/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * Enable simulated GPS sinstance
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enabled
  */
PARAM_DEFINE_INT32(SENS_EN_GPSSIM, 0);

/**
 * simulated GPS number of satellites used
 *
 * @min 0
 * @max  50
 * @group Simulator
 */
PARAM_DEFINE_INT32(SIM_GPS_USED, 10);

/**
 * simulated GPS horizontal position noise standard deviation
 *
 * @unit m
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_XY_STD, 0.2f);

/**
 * simulated GPS vertical position noise standard deviation
 *
 * @unit m
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_Z_STD, 0.5f);

/**
 * simulated GPS north velocity noise standard deviation
 *
 * @unit m/s
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_VN_STD, 0.06f);

/**
 * simulated GPS east velocity noise standard deviation
 *
 * @unit m/s
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_VE_STD, 0.077f);

/**
 * simulated GPS down velocity noise standard deviation
 *
 * @unit m/s
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_VD_STD, 0.158f);

/**
 * simulated GPS reported horizontal position accuracy
 *
 * @unit m
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_EPH, 0.9f);

/**
 * simulated GPS reported vertical position accuracy
 *
 * @unit m
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_EPV, 1.78f);

/**
 * simulated GPS reported speed accuracy
 *
 * @unit m/s
 * @min 0
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_SACC, 0.4f);
