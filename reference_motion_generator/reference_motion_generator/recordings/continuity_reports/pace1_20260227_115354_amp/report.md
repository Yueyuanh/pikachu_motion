# Continuity Check Report

- source: `recordings/dataformat/mocap_motions/pace1.txt`
- frames: `492`
- frame_duration: `0.021`
- joint_count: `18` (inferred_no_contacts)

## Boundary Jump (last vs first)
- root_xyz_jump: `[7.35331, -0.34695, 0.00944]`
- root_xyz_jump_norm: `7.36150`
- joint_jump_abs_max: `0.31879`
- joint_jump_abs_mean: `0.12178`

## Velocity Continuity (end vs start)
- root_vel_mismatch: `[0.04476, -0.00714, -0.07667]`
- root_vel_mismatch_norm: `0.08906`
- joint_vel_mismatch_abs_max: `5.24905`
- joint_vel_mismatch_abs_mean: `1.16624`

## Smoothness Summary
- root_speed_max: `0.91727`
- root_speed_mean: `0.72138`
- root_speed_delta_abs_max: `0.08592`
- root_speed_delta_abs_mean: `0.02087`

## Charts
- `charts/root_xyz.svg`
- `charts/root_speed.svg`
- `charts/joint_jump_abs.svg`
- `charts/joint_vel_mismatch_abs.svg`