# walk: straight walks, arms still

> **Stage 1 (walking first, 23 Oct):** this folder's CSVs are out of the repository for now. Their arm poses come from a
> random-arm generator and twist and roll the wrists oddly. They are kept in
> `hrp5p_motion_tracking_deployment/motions_not_ready/walk/` and come back in a later stage; the rates below still hold.

Made for deployment with the BaselineWalkingController generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
`topic/deploy-motion-sets`, configs `motions/walk_<d>s.yaml`, seed 1). Each stands 3 s while both arms move once to a
ready posture in front of the body, walks straight ahead at 0.15 m/s, stops, and stands still for the last 3 s. Heading
stays within 1.6° and sideways drift within 0.13 m; the hands do not move after the first 3 s. No turns.

| File | Length | Walk | In the repo |
|---|---|---|---|
| `motion_walk_15s.csv` | 15 s | about 0.7 m, feet still from about 12 s | yes |
| `motion_walk_30s.csv` | 30 s | about 2.9 m, feet still from about 27 s | yes, by decision (see below) |
| `motion_walk_45s.csv` | 45 s | about 5.1 m | no |
| `motion_walk_60s.csv` | 60 s | about 7.4 m | no |

## Failure rates

Method: [../README.md](../README.md#failure-rates). P8 and P9 measured on 23 Oct 2026, P11 on 24 Oct, P13 on 25 Oct.

| Motion | mjlab P13 | mjlab P11 / P9 / P8 | mc_mujoco P13 (est) | Torso error, P13 / P11 / P9 (est) | Robot (P13 in slot 3) |
|---|---|---|---|---|---|
| 15 s | 0 / 64 | 0 / 0 / 0 | no fall | 0.067 / 0.085 / 0.071 m | not in stage 1 |
| 30 s | 17 / 64 | 16 / 1 / 20 | no fall | 0.199 / 0.188 / 0.201 m | not in stage 1 |
| 45 s | 53 / 64 | 56 / 35 / 49 | no fall | 0.364 / 0.236 / 0.284 m | not in stage 1 |
| 60 s | 57 / 64 | 61 / 56 / 60 | no fall | 0.581 / 0.316 / 0.361 m | not in stage 1 |

mc_mujoco, earlier slot-3 policies: P11 no fall (est); P9 and P8 no fall (est and gt) on all four.

Long straight walks with the arms held still in front of the body are hard for all of them (their training motions walk
in 4 to 8 s segments): in mjlab the falls start after about 20 s of walking, mostly as the robot stops at the end of the
walk, and the robot falls further behind the reference the longer it walks. P11 and P13 hold long walks with the arms
in their default pose (`walk_default_arms/`) but fall more than P9 here.

The 30 s walk was in the repository by decision (23 Oct, P9: 1 fall in 64, at 28.5 s) until stage 1.

Left out of the repository (falls in mjlab): the 45 and 60 s walks, kept in
`hrp5p_motion_tracking_deployment/motions_not_ready/walk/`.
