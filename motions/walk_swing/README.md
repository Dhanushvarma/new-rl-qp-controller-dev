# walk_swing: straight walks with arm swings while walking

> **Stage 1 (walking first, 23 Oct):** this folder's CSVs are out of the repository for now. Their arm poses come from a
> random-arm generator and twist and roll the wrists oddly. They are kept in
> `hrp5p_motion_tracking_deployment/motions_not_ready/walk_swing/` and come back in a later stage; the rates below still hold.

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

Method: [../README.md](../README.md#failure-rates). P8 and P9 measured on 23 Oct 2026, P11 on 24 Oct, P13 on 25 Oct.
mjlab P8 not measured.

| Motion | mjlab P13 | mjlab P11 / P9 | mc_mujoco P13 (est) | Torso error, P13 / P11 / P9 (est) | Robot (P13 in slot 3) |
|---|---|---|---|---|---|
| 15 s | 0 / 64 | 0 / 0 | no fall | 0.030 / 0.044 / 0.069 m | not in stage 1 |
| 30 s | 0 / 64 | 2 / 0 | no fall | 0.109 / 0.120 / 0.133 m | not in stage 1 |
| 45 s | 0 / 64 | 1 / 6 | no fall | 0.108 / 0.107 / 0.114 m | not in stage 1 |
| 60 s | 6 / 64 | 56 / 45 | no fall, no guard trip | 0.130 / 0.181 / 0.346 m | not in stage 1 |

mc_mujoco, earlier slot-3 policies: no fall (P11 est; P9 and P8 est and gt) on all four, but on the 60 s motion the
controller's runaway guard disarms P11 and P9 and holds the posture: a wrist yaw joint passes 6 rad/s (P11: right,
6.17 rad/s at 46.7 s; P9: left, 6.04 and 6.16 rad/s); their torso errors are over the tracked part. Found on 24 Oct in
the run logs; the 23 Oct table had read these runs as plain "no fall". P13 does not trip it.

**Slot 4 (P15, 26 Oct):** mjlab 0 / 0 / 0 / 6 of 64; mc_mujoco no fall and no guard trip (est; torso error 0.029 /
0.077 / 0.090 / 0.263 m).

Left out of the repository (falls in mjlab): the 45 and 60 s motions, kept in
`hrp5p_motion_tracking_deployment/motions_not_ready/walk_swing/`. As with the plain walks, the falls come with long
continuous walking.
