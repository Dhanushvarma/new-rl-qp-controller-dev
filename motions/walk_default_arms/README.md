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

Method: [../README.md](../README.md#failure-rates). P9 measured on 23-24 Oct 2026, P11 on 24 Oct, P13 on 25 Oct.

| Motion | mjlab P13 | mjlab P11 / P9 | mc_mujoco P13 (est, gt) | Torso error, P13 / P11 / P9 (est) | Robot (P13 in slot 3) |
|---|---|---|---|---|---|
| 15 s | 0 / 64 | 0 / 0 | no fall, no fall | 0.039 / 0.041 / 0.062 m | ready |
| 30 s | 0 / 64 | 0 / 1 | no fall, no fall | 0.066 / 0.084 / 0.123 m | ready |
| 45 s | 0 / 64 | 0 / 11 | no fall, no fall | 0.082 / 0.148 / 0.158 m | ready |
| 60 s | 0 / 64 | 0 / 14 | no fall, no fall | 0.093 / 0.216 / 0.184 m | ready |
| stops 30 s | 0 / 64 | 0 / 0 | no fall, no fall | 0.059 / 0.051 / 0.091 m | ready |
| stops 45 s | 0 / 64 | 0 / 0 | no fall, no fall | 0.074 / 0.057 / 0.113 m | ready |
| stops 60 s | 0 / 64 | 0 / 0 | no fall, no fall | 0.089 / 0.060 / 0.135 m | ready |

P11 and P9 had no fall in mc_mujoco (est and gt) on all seven either.

**Slot 4 (P15, 26 Oct):** 0 / 64 in mjlab and no fall in mc_mujoco (est, gt) on all seven: ready. Torso error
(est): 15 s 0.031 m; continuous 30 / 45 / 60 s 0.061 / 0.079 / 0.107 m; stops 30 / 45 / 60 s 0.042 / 0.049 / 0.054 m.

**Slot 5 (P17, 27 Oct):** 0 / 64 in mjlab and no fall in mc_mujoco (est, gt) on all seven: ready. Torso error
(est): 15 s 0.018 m; continuous 30 / 45 / 60 s 0.034 / 0.047 / 0.063 m; stops 30 / 45 / 60 s 0.022 / 0.026 / 0.029 m,
the closest of the three slots; the wrists are held up to 0.1 rad further from their orientation than with P15.

**Slot 6 (P21, 31 Oct):** 0 / 64 in mjlab and no fall in mc_mujoco (est, gt) on all seven: ready. Torso error
(est): 15 s 0.027 m; continuous 30 / 45 / 60 s 0.073 / 0.099 / 0.114 m; stops 30 / 45 / 60 s 0.035 / 0.049 / 0.065 m.

The continuous 30, 45 and 60 s walks were left out while P9 was in slot 3 (1, 11 and 14 of 64 fell in mjlab, after
long continuous walking). P11, trained with such walks, tracks all seven without a fall, so they are back in the
repository since 24 Oct; P13 does too, and with the MCWaiko estimate it drifts least on the continuous ones. On the
robot, start with the 15 s and stop-and-go walks; the continuous ones walk up to 7.6 m.
