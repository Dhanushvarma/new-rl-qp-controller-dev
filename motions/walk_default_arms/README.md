# walk_default_arms: straight walks, no arm motion of ours

Made for deployment with the BaselineWalkingController generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
`topic/deploy-motion-sets`, configs `motions/walk_default_arms_*.yaml`, option `arms: hold_start`). The wrist targets
never move: both arms keep their default (stance) posture for the whole motion, and only the walking controller acts
(hand speed relative to the torso at most 0.05 m/s). Each stands 3 s, walks straight ahead at 0.15 m/s, and ends standing
still. Heading within 1.6°, sideways drift within 0.09 m. No turns.

Two kinds:
- **continuous:** one walk, then a stop;
- **stops:** 6 s legs of walking with 8 s stops between them, which P9 manages at every length.

| File | Length | Walk | In the repo |
|---|---|---|---|
| `motion_walk_default_arms_15s.csv` | 15 s | about 0.8 m, continuous | yes |
| `motion_walk_default_arms_30s.csv` | 30 s | about 3.1 m, continuous | no |
| `motion_walk_default_arms_45s.csv` | 45 s | about 5.3 m, continuous | no |
| `motion_walk_default_arms_60s.csv` | 60 s | about 7.6 m, continuous | no (not measured yet) |
| `motion_walk_default_arms_stops_30s.csv` | 30 s | about 1.6 m in 2 legs | yes |
| `motion_walk_default_arms_stops_45s.csv` | 45 s | about 2.3 m in 3 legs | yes |
| `motion_walk_default_arms_stops_60s.csv` | 60 s | about 3.1 m in 4 legs | yes |

## Failure rates

Method: [../README.md](../README.md#failure-rates). Measured on 23 Oct 2026.

| Motion | mjlab P9 | mc_mujoco P9 (est, gt) | Torso error, P9 (est) | Robot (P9 in slot 3) |
|---|---|---|---|---|
| 15 s | 0 / 64 | no fall, no fall | 0.062 m | ready |
| 30 s | 1 / 64 | no fall, no fall | 0.123 m | not ready |
| 45 s | 11 / 64 | no fall, no fall | 0.158 m | not ready |
| 60 s | not measured yet | no fall, no fall | 0.184 m | not ready |
| stops 30 s | 0 / 64 | no fall, no fall | 0.091 m | ready |
| stops 45 s | 0 / 64 | no fall, no fall | 0.113 m | ready |
| stops 60 s | 0 / 64 | no fall, no fall | 0.135 m | ready |

Left out of the repository: the 30 and 45 s continuous walks (1 and 11 of 64 fell in mjlab) and the 60 s one, whose
mjlab check was paused (it cannot pass where the 45 s fails); their CSVs are kept in
`hrp5p_motion_tracking_deployment/motions_not_ready/walk_default_arms/`. As in `walk/`, the falls come with long
continuous walking; the stop-and-go walks avoid it.
