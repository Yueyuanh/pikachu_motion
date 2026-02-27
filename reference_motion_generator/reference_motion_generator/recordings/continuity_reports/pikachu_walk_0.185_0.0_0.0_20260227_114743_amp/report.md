# Continuity Check Report

- source: `reference_motion_generator/reference_motion_generator/recordings/pikachu_walk_txt_norm/pikachu_walk_0.185_0.0_0.0.txt`
- frames: `101`
- frame_duration: `0.02`
- joint_count: `10` (inferred_no_contacts)

## Boundary Jump (last vs first)
- root_xyz_jump: `[0.37537, 0.05404, -0.00061]`
- root_xyz_jump_norm: `0.37924`
- joint_jump_abs_max: `0.26446`
- joint_jump_abs_mean: `0.10767`

## Velocity Continuity (end vs start)
- root_vel_mismatch: `[-0.0835, 0.087, 0.012]`
- root_vel_mismatch_norm: `0.12118`
- joint_vel_mismatch_abs_max: `4.80700`
- joint_vel_mismatch_abs_mean: `1.57010`

## Smoothness Summary
- root_speed_max: `0.77803`
- root_speed_mean: `0.21261`
- root_speed_delta_abs_max: `0.56006`
- root_speed_delta_abs_mean: `0.03603`

## Charts
- `charts/root_xyz.svg`
- `charts/root_speed.svg`
- `charts/joint_jump_abs.svg`
- `charts/joint_vel_mismatch_abs.svg`