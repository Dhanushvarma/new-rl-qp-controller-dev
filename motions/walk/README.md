# walk: straight walks, arms still

Made for deployment with the BaselineWalkingController generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
`topic/deploy-motion-sets`, configs `motions/walk_<d>s.yaml`, seed 1). Each stands 3 s while both arms move once to a
ready posture in front of the body, walks straight ahead at 0.15 m/s, stops, and stands still for the last 3 s. Heading
stays within 1.6° and sideways drift within 0.13 m; the hands do not move after the first 3 s. No turns.

| File | Length | Walk | In the repo |
|---|---|---|---|
| `motion_walk_15s.csv` | 15 s | about 0.7 m, feet still from about 12 s | yes |
| `motion_walk_30s.csv` | 30 s | about 2.9 m | no |
| `motion_walk_45s.csv` | 45 s | about 5.1 m | no |
| `motion_walk_60s.csv` | 60 s | about 7.4 m | no |

## Failure rates

Method: [../README.md](../README.md#failure-rates). Measured on 23 Oct 2026.

| Motion | mjlab P9 | mjlab P8 | mc_mujoco P9 (est, gt) | mc_mujoco P8 (est, gt) | Torso error, P9 (est) | Robot (P9 in slot 3) |
|---|---|---|---|---|---|---|
| 15 s | 0 / 64 | 0 / 64 | no fall, no fall | no fall, no fall | 0.071 m | ready |
| 30 s | 1 / 64 | 20 / 64 | no fall, no fall | no fall, no fall | 0.201 m | not ready |
| 45 s | 35 / 64 | 49 / 64 | no fall, no fall | no fall, no fall | 0.284 m | not ready |
| 60 s | 56 / 64 | 60 / 64 | no fall, no fall | no fall, no fall | 0.361 m | not ready |

Long straight walks are new to P8 and P9 (their training motions walk in 4 to 8 s segments): in mjlab the falls start
after about 20 s of walking, and the robot falls further behind the reference the longer it walks. P10, trained with
straight walks added, is meant to fix this.

Left out of the repository (falls in mjlab): the 30, 45 and 60 s walks, kept in
`hrp5p_motion_tracking_deployment/motions_not_ready/walk/`.
