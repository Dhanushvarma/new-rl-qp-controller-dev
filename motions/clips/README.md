# clips: short clips that end standing

> **Stage 1 (walking first, 23 Oct):** only the 10 s stand stays in the repository. The 15 and 31 s clips come from a
> random-arm generator (odd wrist poses) and are kept in `hrp5p_motion_tracking_deployment/motions_not_ready/clips/` for a
> later stage; the rates below still hold.

Cut from P6's 60 s trial motions (made by the BaselineWalkingController generator, with random arm gestures, steps
and turns) where the reference stands still. Each clip holds its last frame for 3 s with zero velocities. Distances
are those of the torso from the motion's start.

| File | Length | Source | What the reference does |
|---|---|---|---|
| `motion_stand_seed104_10s.csv` | 10 s | seed104's frame at 0.5 s, held | stands still, arms in the start pose |
| `motion_clip_seed104_15s.csv` | 18.4 s | seed104 0-15.4 s | stands 2 s; raises both arms to about chest height (left first, right by 9 s); turns about 36° left in place with small steps (6-10 s, feet move 7 cm at most); stands with the arms raised. Ends 6 cm from its start |
| `motion_clip_seed084_31s.csv` | 34.6 s | seed084 0-31.6 s | raises the arms; steps back about 0.5 m (6-12 s); turns about 165° left with steps (12-24 s); stands moving the arms. Ends 0.35 m from its start, facing almost backward |
| `motion_clip_seed062_42s.csv` (not in the repo) | 45.7 s | seed062 0-42.7 s | raises the arms; walks about 1.3 m forward while turning about 40° right (6-18 s); steps back about 0.3 m (18-30 s); walks about 1.5 m forward (30-39 s); stands. Ends 2.5 m from its start |

## Failure rates

Method: [../README.md](../README.md#failure-rates). P8 and P9 measured on 23 Oct 2026, P11 on 24 Oct.

| Motion | In the repo | mjlab P11 | mjlab P9 | mjlab P8 | mc_mujoco P11 (est, gt) | mc_mujoco P9 (est, gt) | mc_mujoco P8 (est, gt) | Robot (P11 in slot 3) |
|---|---|---|---|---|---|---|---|---|
| stand 10 s | yes | 0 / 64 | 0 / 64 | 0 / 64 | no fall, no fall | no fall, no fall | no fall, no fall | ready |
| 15 s clip | no (stage 1) | 0 / 64 | 0 / 64 | 0 / 64 | no fall (est only) | no fall, no fall | no fall, no fall | not in stage 1 |
| 31 s clip | no (stage 1) | 0 / 64 | 0 / 64 | 0 / 64 | no fall (est only) | no fall, no fall | no fall, no fall | not in stage 1 |
| 42 s clip | no | 0 / 64 | 3 / 64 | 5 / 64 | not run | no fall (est only) | no fall (est only) | not in stage 1 |

Left out of the repository: the 42 s clip since 23 Oct (falls in mjlab with P9), the 15 and 31 s clips since stage 1;
kept in `hrp5p_motion_tracking_deployment/motions_not_ready/clips/`.

Tracking in mc_mujoco (torso error from the reference, est):

| Motion | P8 | P9 | P11 |
|---|---|---|---|
| stand 10 s | 0.028 m | 0.027 m | 0.022 m |
| 15 s clip | 0.055 m | 0.051 m | 0.070 m |
| 31 s clip | 0.108 m | 0.031 m | 0.082 m |
| 42 s clip | 0.145 m | 0.115 m | not run |
