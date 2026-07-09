# Skyvortex Estimator Noise Control

This note explains how to control the estimator noise sources used by PX4 SITL for Skyvortex controller tuning.

## Why this matters

`sensor_gps_sim` publishes `sensor_gps` from the simulator ground-truth position. PX4 EKF fuses this GPS data and then publishes `/fmu/out/vehicle_local_position`.

If GPS noise is enabled, `/fmu/out/vehicle_local_position` can move even while the vehicle is physically still on the ground. That false local-position motion can dominate controller tuning results.

After GPS noise is removed, small residual drift can still come from the EKF fusing IMU dynamic bias, magnetometer data, and measurement covariance assumptions. Do not disable GPS height fusion unless you have confirmed that another height source is being fused successfully.

## Parameters

The simulated GPS noise is controlled by these PX4 parameters:

| Parameter | Meaning | Global default |
|---|---:|---:|
| `SIM_GPS_XY_STD` | Horizontal position noise standard deviation | `0.2 m` |
| `SIM_GPS_Z_STD` | Vertical position noise standard deviation | `0.5 m` |
| `SIM_GPS_VN_STD` | North velocity noise standard deviation | `0.06 m/s` |
| `SIM_GPS_VE_STD` | East velocity noise standard deviation | `0.077 m/s` |
| `SIM_GPS_VD_STD` | Down velocity noise standard deviation | `0.158 m/s` |
| `SIM_GPS_EPH` | Reported horizontal GPS accuracy | `0.9 m` |
| `SIM_GPS_EPV` | Reported vertical GPS accuracy | `1.78 m` |
| `SIM_GPS_SACC` | Reported GPS speed accuracy | `0.4 m/s` |

These global defaults match the old hard-coded values, so other airframes keep the same simulation behavior unless an airframe file or runtime parameter explicitly changes them.

## Skyvortex controller tuning setup

For `6027_gz_skyvortex`, the airframe sets the GPS measurement noise to zero and also lowers the reported GPS accuracy values:

```sh
param set-default SIM_GPS_XY_STD 0.0
param set-default SIM_GPS_Z_STD 0.0
param set-default SIM_GPS_VN_STD 0.0
param set-default SIM_GPS_VE_STD 0.0
param set-default SIM_GPS_VD_STD 0.0
param set-default SIM_GPS_EPH 0.05
param set-default SIM_GPS_EPV 0.05
param set-default SIM_GPS_SACC 0.03
```

Both parts are important. If the GPS values are noise-free but `SIM_GPS_EPH`, `SIM_GPS_EPV`, and `SIM_GPS_SACC` remain at their old meter-level defaults, EKF2 will still treat GPS as a weak height/position source and can drift noticeably on the ground.

Do not set the following as airframe defaults unless `vehicle_visual_odometry` is confirmed to be fused by EKF2:

```sh
param set EKF2_EV_DELAY 0
param set EKF2_EV_CTRL 15
param set EKF2_EV_NOISE_MD 1
param set EKF2_EVP_NOISE 0.01
param set EKF2_EVV_NOISE 0.01
param set EKF2_EVA_NOISE 0.05
```

Keep `EKF2_GPS_CTRL`, `EKF2_HGT_REF`, `EKF2_BARO_CTRL`, and `EKF2_MAG_TYPE` at their normal values until the EV fusion status is verified. Disabling GPS/barometer/magnetometer fusion without a working replacement can cause arming failure and continuous `vehicle_local_position.z` drift.

## Runtime control

Use the PX4 shell or QGC MAVLink Console:

```sh
param set SIM_GPS_XY_STD 0
param set SIM_GPS_Z_STD 0
param set SIM_GPS_VN_STD 0
param set SIM_GPS_VE_STD 0
param set SIM_GPS_VD_STD 0
param set SIM_GPS_EPH 0.05
param set SIM_GPS_EPV 0.05
param set SIM_GPS_SACC 0.03
param save
```

Experimental: to test lower-noise EV fusion at runtime without disabling GPS height fallback:

```sh
param set EKF2_EV_DELAY 0
param set EKF2_EV_CTRL 15
param set EKF2_EV_NOISE_MD 1
param set EKF2_EVP_NOISE 0.01
param set EKF2_EVV_NOISE 0.01
param set EKF2_EVA_NOISE 0.05
param save
```

Restart SITL after changing `EKF2_EV_DELAY`, because it requires a clean EKF startup.

To restore the old noisy GPS behavior:

```sh
param set SIM_GPS_XY_STD 0.2
param set SIM_GPS_Z_STD 0.5
param set SIM_GPS_VN_STD 0.06
param set SIM_GPS_VE_STD 0.077
param set SIM_GPS_VD_STD 0.158
param set SIM_GPS_EPH 0.9
param set SIM_GPS_EPV 1.78
param set SIM_GPS_SACC 0.4
param save
```

If the vehicle cannot arm or `vehicle_local_position.z` keeps drifting, restore normal EKF fusion:

```sh
param set EKF2_EV_NOISE_MD 0
param set EKF2_EVP_NOISE 0.1
param set EKF2_EVV_NOISE 0.1
param set EKF2_EVA_NOISE 0.1
param set EKF2_HGT_REF 1
param set EKF2_GPS_CTRL 7
param set EKF2_BARO_CTRL 1
param set EKF2_MAG_TYPE 0
param save
```

Restart SITL after restoring these values.

If the saved parameter file is still overriding the airframe defaults, stop SITL and move the cached parameter files out of the rootfs:

```sh
cd ~/firmware_pfa_skyvortex/build/px4_sitl_default/rootfs
mkdir -p param_bak
mv parameters.bson parameters_backup.bson param_bak/
```

Then restart SITL so PX4 regenerates parameters from the airframe defaults.

The simulator module reads parameter updates during runtime, but after large changes it is best to restart SITL and wait for EKF to settle before measuring controller performance.

## Checking current values

```sh
param show SIM_GPS_XY_STD
param show SIM_GPS_Z_STD
param show SIM_GPS_VN_STD
param show SIM_GPS_VE_STD
param show SIM_GPS_VD_STD
param show SIM_GPS_EPH
param show SIM_GPS_EPV
param show SIM_GPS_SACC
param show EKF2_EV_CTRL
param show EKF2_EV_NOISE_MD
param show EKF2_EVP_NOISE
param show EKF2_EVV_NOISE
param show EKF2_EVA_NOISE
param show EKF2_HGT_REF
param show EKF2_GPS_CTRL
param show EKF2_BARO_CTRL
param show EKF2_MAG_TYPE
```

Then verify the estimator output while the vehicle is still on the ground:

```sh
ros2 topic hz /fmu/out/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position
```

For controller tuning, `vehicle_visual_odometry` must be publishing and the stationary `x`, `y`, `z`, `vx`, `vy`, and `vz` values in `vehicle_local_position` should be close to steady. Small EKF transients immediately after startup are normal; wait several seconds before taking measurements.

## Existing parameter files

`param set-default` affects the airframe default. If an old SITL parameter file already saved different values, the saved values can override the airframe defaults.

In that case, either set and save the values manually with `param set`, or reset the parameters:

```sh
param reset SIM_GPS_XY_STD
param reset SIM_GPS_Z_STD
param reset SIM_GPS_VN_STD
param reset SIM_GPS_VE_STD
param reset SIM_GPS_VD_STD
param reset SIM_GPS_EPH
param reset SIM_GPS_EPV
param reset SIM_GPS_SACC
param reset EKF2_EV_DELAY
param reset EKF2_EV_CTRL
param reset EKF2_EV_NOISE_MD
param reset EKF2_EVP_NOISE
param reset EKF2_EVV_NOISE
param reset EKF2_EVA_NOISE
param reset EKF2_HGT_REF
param reset EKF2_GPS_CTRL
param reset EKF2_BARO_CTRL
param reset EKF2_MAG_TYPE
param save
```

Restart SITL after resetting if you want a clean estimator initialization.
