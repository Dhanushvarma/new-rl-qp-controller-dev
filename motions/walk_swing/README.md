# walk_swing: straight walks with arm swings while walking

Made for deployment with the BaselineWalkingController generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
`topic/deploy-motion-sets`, configs `motions/walk_swing_<d>s.yaml`). Each stands 3 s while both arms move once to a ready
posture, walks straight ahead at 0.15 m/s, stops, and stands for the last 2 s. While the feet step, both arms swing
gently in opposite phase (shoulder pitch 0.10-0.20 rad, elbow half of it, 0.7-0.85 Hz); whenever the feet are still the
hands do not move. Hand speed at most 0.39 m/s; heading within 1.7°, sideways drift within 0.09 m. No turns.

| File | Length | Walk | Arms swing | In the repo |
|---|---|---|---|---|
| `motion_walk_swing_15s.csv` | 15 s | about 0.9 m | 4-10 s | yes |
| `motion_walk_swing_30s.csv` | 30 s | about 3.1 m | 5-21 s | yes |
| `motion_walk_swing_45s.csv` | 45 s | about 5.3 m | 5-37 s | no |
| `motion_walk_swing_60s.csv` | 60 s | about 7.6 m (hands kept 0.2 m above the waist) | 5-52 s | no |

## Failure rates

Method: [../README.md](../README.md#failure-rates). Measured on 23 Oct 2026. mjlab P8 not measured (P9 is in slot 3).

| Motion | mjlab P9 | mc_mujoco P9 (est, gt) | mc_mujoco P8 (est, gt) | Torso error, P9 (est) | Robot (P9 in slot 3) |
|---|---|---|---|---|---|
| 15 s | 0 / 64 | no fall, no fall | no fall, no fall | 0.069 m | ready |
| 30 s | 0 / 64 | no fall, no fall | no fall, no fall | 0.133 m | ready |
| 45 s | 6 / 64 | no fall, no fall | no fall, no fall | 0.114 m | not ready |
| 60 s | 45 / 64 | no fall, no fall | no fall, no fall | 0.346 m | not ready |

Left out of the repository (falls in mjlab): the 45 and 60 s motions, kept in
`hrp5p_motion_tracking_deployment/motions_not_ready/walk_swing/`. As with the plain walks, the falls come with long
continuous walking, which P10 is meant to fix.
