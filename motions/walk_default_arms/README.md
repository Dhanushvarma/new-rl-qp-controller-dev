# walk_default_arms: straight walks, no arm motion of ours

Made for deployment with the BaselineWalkingController generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
`topic/deploy-motion-sets`, configs `motions/walk_default_arms_*.yaml`, option `arms: hold_start`). The wrist targets
never move: both arms keep their default (stance) posture for the whole motion, and only the walking controller acts
(hand speed relative to the torso at most 0.05 m/s). Each stands 3 s, walks straight ahead at 0.15 m/s, and ends standing
still. Heading within 1.6°, sideways drift within 0.09 m. No turns.

Two kinds:
- **continuous:** one walk, then a stop;
- **stops:** 6 s legs of walking with 8 s stops between them, which P9 and P11 manage at every length.

| File | Length | Walk | In the repo |
|---|---|---|---|
| `motion_walk_default_arms_15s.csv` | 15 s | about 0.8 m, continuous | yes |
| `motion_walk_default_arms_30s.csv` | 30 s | about 3.1 m, continuous | yes |
| `motion_walk_default_arms_45s.csv` | 45 s | about 5.3 m, continuous | yes |
| `motion_walk_default_arms_60s.csv` | 60 s | about 7.6 m, continuous | yes |
| `motion_walk_default_arms_stops_30s.csv` | 30 s | about 1.6 m in 2 legs | yes |
| `motion_walk_default_arms_stops_45s.csv` | 45 s | about 2.3 m in 3 legs | yes |
| `motion_walk_default_arms_stops_60s.csv` | 60 s | about 3.1 m in 4 legs | yes |

## Failure rates

Method: [../README.md](../README.md#failure-rates). P9 measured on 23-24 Oct 2026, P11 on 24 Oct.

| Motion | mjlab P11 | mjlab P9 | mc_mujoco P11 (est, gt) | mc_mujoco P9 (est, gt) | Torso error, P11 / P9 (est) | Robot (P11 in slot 3) |
|---|---|---|---|---|---|---|
| 15 s | 0 / 64 | 0 / 64 | no fall, no fall | no fall, no fall | 0.041 / 0.062 m | ready |
| 30 s | 0 / 64 | 1 / 64 | no fall, no fall | no fall, no fall | 0.084 / 0.123 m | ready |
| 45 s | 0 / 64 | 11 / 64 | no fall, no fall | no fall, no fall | 0.148 / 0.158 m | ready |
| 60 s | 0 / 64 | 14 / 64 | no fall, no fall | no fall, no fall | 0.216 / 0.184 m | ready |
| stops 30 s | 0 / 64 | 0 / 64 | no fall, no fall | no fall, no fall | 0.051 / 0.091 m | ready |
| stops 45 s | 0 / 64 | 0 / 64 | no fall, no fall | no fall, no fall | 0.057 / 0.113 m | ready |
| stops 60 s | 0 / 64 | 0 / 64 | no fall, no fall | no fall, no fall | 0.060 / 0.135 m | ready |

The continuous 30, 45 and 60 s walks were left out while P9 was in slot 3 (1, 11 and 14 of 64 fell in mjlab, after
long continuous walking). P11, trained with such walks, tracks all seven without a fall, so they are back in the
repository since 24 Oct. On the robot, start with the 15 s and stop-and-go walks; the continuous ones walk up to 7.6 m.
