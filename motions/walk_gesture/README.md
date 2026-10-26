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

Method: [../README.md](../README.md#failure-rates). P8 and P9 measured on 23 Oct 2026, P11 on 24 Oct, P13 on 25 Oct.
mjlab P8 not measured.

| Motion | mjlab P13 | mjlab P11 / P9 | mc_mujoco P13 (est) | Torso error, P13 / P11 / P9 (est) | Robot (P13 in slot 3) |
|---|---|---|---|---|---|
| 15 s | 0 / 64 | 0 / 0 | no fall | 0.017 / 0.019 / 0.029 m | not in stage 1 |
| 30 s | 0 / 64 | 0 / 0 | no fall | 0.076 / 0.114 / 0.091 m | not in stage 1 |
| 45 s | 0 / 64 | 0 / 0 | no fall | 0.121 / 0.148 / 0.113 m | not in stage 1 |
| 60 s | 0 / 64 | 2 / 0 | no fall | 0.167 / 0.168 / 0.132 m | not in stage 1 |

mc_mujoco, earlier slot-3 policies: P11 no fall (est); P9 and P8 no fall (est and gt) on all four.

**Slot 4 (P15, 26 Oct):** mjlab 0 / 64 on all four; mc_mujoco no fall (est; torso error 0.019 / 0.082 / 0.105 /
0.115 m).

**Slot 5 (P17, 27 Oct):** mjlab 0 / 1 / 1 / 2 of 64 (15, 30, 45, 60 s); mc_mujoco no fall (est; torso error 0.015 /
0.071 / 0.100 / 0.121 m).

The walking comes in short legs with stops, which they all handle; P11 had 2 falls in 64 on the 60 s motion.
