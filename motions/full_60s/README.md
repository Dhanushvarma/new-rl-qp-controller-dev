# full_60s: P6's five 60 s trial motions, whole

> **Stage 1 (walking first, 23 Oct):** this folder's CSVs are out of the repository for now. Their arm poses come from a
> random-arm generator and twist and roll the wrists oddly. They are kept in
> `hrp5p_motion_tracking_deployment/motions_not_ready/full_60s/` and come back in a later stage; the rates below still hold.

Made by the BaselineWalkingController generator with random walking segments (forward, backward, sideways, turns, arcs)
and random arm gestures (swings, reaches, waves). Distances are those of the torso from the motion's start.

| File | What the reference does |
|---|---|
| `motion_bwc_v2_seed010.csv` | steps back and to the left, walks forward about 1.5 m while turning about 90° right, gestures; ends 1.1 m from its start |
| `motion_bwc_v2_seed060.csv` (not in the repo) | turns about 85° right, steps about 1 m sideways, walks about 2.5 m forward and to the right; ends 2.9 m from its start |
| `motion_bwc_v2_seed062.csv` (not in the repo) | walks about 1.3 m forward while turning about 40° right, steps back, walks forward again, raises both arms at the end; ends 2.8 m from its start |
| `motion_bwc_v2_seed084.csv` (not in the repo) | steps back about 0.5 m, turns about 165° left, walks about 1.5 m; ends 1.9 m from its start |
| `motion_bwc_v2_seed104.csv` (not in the repo) | turns about 36° left in place with the arms raised, walks about 1.8 m forward and to the left; ends 2.2 m from its start |

## Failure rates

Method: [../README.md](../README.md#failure-rates). P9 on 23 Oct, P11 on 24 Oct and P13 on 25 Oct: 320 robots spread
over the five motions (51 to 82 each), so the counts are falls out of the robots that ran that motion. P8 on 22 Oct: 128
robots over the five, about 26 each.

| Motion | In the repo | mjlab P13 | mjlab P11 / P9 | mjlab P8 | mc_mujoco P9 (est, gt) | Robot (P13 in slot 3) |
|---|---|---|---|---|---|---|
| seed010 | no (stage 1) | 0 / 63 | 0 / 0 | 2 falls | no fall, no fall (torso error 0.105 / 0.110 m) | not in stage 1 |
| seed060 | no | 1 / 51 | 1 / 2 | 4 falls | not run | not in stage 1 |
| seed062 | no | 0 / 56 | 0 / 2 | 3 falls | not run | not in stage 1 |
| seed084 | no | 0 / 82 | 1 / 2 | 1 fall | not run | not in stage 1 |
| seed104 | no | 0 / 68 | 0 / 1 | 0 falls | not run | not in stage 1 |

mc_mujoco P11 and P13 not run on these.

**Slot 4 (P15, 26 Oct):** mjlab 2 / 63, 0 / 51, 0 / 56, 0 / 82, 1 / 68 (seeds 010, 060, 062, 084, 104); mc_mujoco not
run.

**Slot 5 (P17, 27 Oct):** mjlab 0 / 63, 0 / 51, 0 / 56, 0 / 82, 0 / 68 (seeds 010, 060, 062, 084, 104); mc_mujoco not
run.

**Slot 6 (P21, 31 Oct):** mjlab 0 / 63, 2 / 51, 0 / 56, 0 / 82, 0 / 68 (seeds 010, 060, 062, 084, 104); mc_mujoco not
run.

Left out of the repository (falls in mjlab): seeds 060, 062, 084 and 104, kept in
`hrp5p_motion_tracking_deployment/motions_not_ready/full_60s/`.
