# Real Robot Debugging Changelog

Date: 2026-05-18

This file tracks the changes made while debugging the real Flexiv/OpenSai kendama run.

## Logging and Runner

- Added `logs/real_robot/latest_driver.log` and `logs/real_robot/latest_opensai.log` symlinks so the newest driver and OpenSai logs can be found without manually checking timestamps.
- Updated `scripts/run_real_kendama.sh` to create timestamped driver/OpenSai log files before symlinking to them, avoiding dangling `latest_*` links during startup.
- Fixed the Flexiv driver config path passed by the runner. The driver expects `config_titania.xml` because it resolves files under its own `config_folder/`.
- Reworked OpenSai readiness detection to use Redis key `::sai-interfaces-webui::config_file_name` instead of waiting on a buffered log line.
- Deleted the Redis config-name key before starting OpenSai so the runner waits for the current OpenSai process to republish readiness, rather than accepting a stale value.
- Added cleanup traps in the runner so OpenSai and the Flexiv driver are stopped if the script exits or is interrupted.

## Startup Safety

- Added measured joint-hold seeding before OpenSai startup:
  - Reads `opensai::sensors::Titania::joint_positions`.
  - Writes the joint controller goal position to the measured pose.
  - Writes zero joint goal velocity and acceleration.
  - Forces `opensai::controllers::Titania::active_controller_name` to `joint_controller` while OpenSai is starting.
- Added stale torque clearing before the Flexiv driver starts by setting `opensai::commands::Titania::control_torques` to a zero vector.
- Reused the same torque-clear key inside `seed_joint_hold_goal()` so joint-hold seeding also prevents old Redis torque commands from being replayed.

## Reset Behavior

- Changed the real robot reset source of truth to measured sensor joints via `opensai::sensors::Titania::joint_positions`; simulation still uses the joint task current position.
- Seeded the joint controller with the measured current joint pose before activating joint control, preventing a large initial reset step.
- Added a smooth joint-space reset ramp from the measured start pose to the default lowered pose.
- Increased real reset timeout from 25 seconds to 35 seconds.
- Changed reset completion from L2 joint-error norm to max per-joint error, with a threshold of `0.10` rad.
- Updated reset diagnostics and timeout errors to report max per-joint error.

## Cartesian Transition

- Identified that the Cartesian controller could have stale `current_position` and `current_orientation` values after a joint-controller reset.
- Added URDF-based forward kinematics helpers in `kendama_throw_and_catch.py`:
  - Parses the configured Rizon URDF.
  - Walks the kinematic chain to the `flange` link.
  - Applies URDF origin transforms and revolute/prismatic joint transforms.
  - Computes the flange position and orientation from measured joint positions.
- Changed `enter_cartesian_hold_from_current_pose()` to compute the Cartesian hold pose from measured joints using FK instead of trusting inactive Cartesian task Redis state.
- Verified the FK position against OpenSai Cartesian current position during a live run; they matched to within the logged precision.

## Current Working Hypotheses

- The early large torque saturation was caused by stale Redis torque commands being consumed when the driver entered `RT_JOINT_TORQUE`; the runner now clears that command before driver startup.
- The later joint velocity-limit violations appeared when switching from joint control to Cartesian control; the likely cause was commanding a stale Cartesian hold pose. The FK-based transition should make the Cartesian goal match the robot's actual measured pose at the transition.
- If velocity-limit warnings persist after the FK transition, the next likely fixes are enabling Cartesian velocity saturation and/or lowering Cartesian gains in the real OpenSai controller configuration.
