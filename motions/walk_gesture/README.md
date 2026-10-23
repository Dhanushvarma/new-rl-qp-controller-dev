# walk_gesture: short straight walks, arm gestures only while standing

> **Stage 1 (walking first, 23 Oct):** this folder's CSVs are out of the repository for now. Their arm poses come from a
> random-arm generator and twist and roll the wrists oddly. They are kept in
> `hrp5p_motion_tracking_deployment/motions_not_ready/walk_gesture/` and come back in a later stage; the rates below still hold.

Made for deployment with the BaselineWalkingController generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
`topic/deploy-motion-sets`, configs `motions/walk_gesture_<d>s.yaml`). Each stands 3 s while both arms move once to a
ready posture, then walks straight ahead at 0.15 m/s in short legs. In the stops, with the feet still, one or both arms
make a slow reach and come back (hand speed at most 0.40 m/s, hands kept 0.2 m below the head); while the feet step the
hands do not move. Heading within 1.7°, sideways drift within 0.09 m. No turns.

| File | Length | Walk | Gestures (feet still) | In the repo |
|---|---|---|---|---|
| `motion_walk_gesture_15s.csv` | 15 s | 2-3 steps in place | left arm reach, 7-11 s | yes |
| `motion_walk_gesture_30s.csv` | 30 s | about 0.8 m, then a few steps | right arm reach, 12.5-17.6 s | yes |
| `motion_walk_gesture_45s.csv` | 45 s | about 1.4 m in two legs | right arm reach 12.5-17.6 s; both arms 31.5-38 s | yes |
| `motion_walk_gesture_60s.csv` | 60 s | about 2.2 m in three legs | right arm reach 12.5-17.6 s; both arms 31.5-38 s | yes |

## Failure rates

Method: [../README.md](../README.md#failure-rates). Measured on 23 Oct 2026. mjlab P8 not measured (P9 is in slot 3).

| Motion | mjlab P9 | mc_mujoco P9 (est, gt) | mc_mujoco P8 (est, gt) | Torso error, P9 (est) | Robot (P9 in slot 3) |
|---|---|---|---|---|---|
| 15 s | 0 / 64 | no fall, no fall | no fall, no fall | 0.029 m | ready |
| 30 s | 0 / 64 | no fall, no fall | no fall, no fall | 0.091 m | ready |
| 45 s | 0 / 64 | no fall, no fall | no fall, no fall | 0.113 m | ready |
| 60 s | 0 / 64 | no fall, no fall | no fall, no fall | 0.132 m | ready |

All four pass: the walking comes in short legs with stops, which P9 handles; it is long continuous walking that it
does not yet (see `walk/` and `walk_swing/`).
