# Changelog



## [current dev progress]

#### Hardware

- Added solder jumper so that we don't need to solder some ICs if we don't need ultrasonic and sound
- Moved some solder jumpers to the bottom of the board to keep them accessible even with IMU mounted.

#### Software

##### Breaking Changes

- Moved most of the configuration to an external file
- Removed start and pause mowing from rosparams. This was just a hack to trigger it using rqt_reconfigure.
- Removed area_recorder and corresponding configs
- Added SensorFusion with imu_filter_madgwick, you need to set your mag biases in the global settings file now

##### Features

- Added UI Board integration to LowLevel Firmware
- Added service to start and pause mowing as well as start area recording mode
- Added area recording behavior to main logic, so that we don't need an additional launch file for it
- Added a mower_logic/current_state topic which has the identifier of the current high level state.
- mower_comms now listens for UI button messages and calls the high level control service accordingly.
- Added support for LAT/LNG coordinates instead of relative coordinates. Set **OM_USE_RELATIVE_POSITION** to **false** for this and provide an origin for your local coordinate space using **OM_DATUM_LAT** and **OM_DATUM_LONG** in your local config file.
- Added the mag_calibration node to help magnetometer calibration without manually recording and copying files.

##### Small Fixes

- Changed planner logic, so that it starts with innermost outlines

- We can now configure arbitrarily many outlines
- Motor temperatures can be configured
- Added safety offset parameter to the outline (i.e. shrink area without rerecording the areas)
- Added vesc_msgs to mower_msgs to resolve a dependency issue



