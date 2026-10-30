# walk_natural_swing: straight walks with a gentle arm swing

> **Stage 2 (30 Oct, the user's request):** the 15 s walk and the 30 s stop-and-go walk are in the repository. The 45
> and 60 s stop-and-go versions stay out for now; their CSVs are kept in
> `hrp5p_motion_tracking_deployment/motions_not_ready/walk_natural_swing/`.

Made for deployment with the BaselineWalkingController generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
`topic/footstep-swing`, configs `motions/walk_natural_swing_*.yaml`, options `arms: hold_start` and `footstep_swing`).
They are the [walk_default_arms](../walk_default_arms/README.md) walks (same schedules) with an arm swing timed by the
footsteps, each arm opposite to its leg: shoulder pitch 0.12 rad forward and 0.02 rad back, and the elbow 0.06 rad more
bent going forward. The back swing is small because the upper arm is close to the chest in the stance posture. Each
stands 3 s, walks straight ahead at 0.15 m/s, and ends standing still. Heading within 1.5°, sideways drift within
0.09 m. No turns.

Arm joints, from the CSVs:
- shoulder pitch swings 8° and elbow pitch 6.5°; the other shoulder joints move about 1°;
- the wrists keep their stance angles (within 2°), unlike the random-arm sets, which twist and roll them;
- the elbows stay at least 70° from their limits. Shoulder pitch and wrist roll sit close to their limits already in
  the stance posture (inside the QP's joint-limit damper zone), as in the default-arm walks; the swing adds about 1°
  towards the shoulder pitch limit.

| File | Length | Walk | In the repo |
|---|---|---|---|
| `motion_walk_natural_swing_15s.csv` | 15 s | about 0.8 m, continuous | yes |
| `motion_walk_natural_swing_stops_30s.csv` | 30 s | about 1.6 m in 2 legs | yes |
| `motion_walk_natural_swing_stops_45s.csv` | 45 s | about 2.3 m in 3 legs | no |
| `motion_walk_natural_swing_stops_60s.csv` | 60 s | about 3.1 m in 4 legs | no |

## Failure rates

Method: [../README.md](../README.md#failure-rates). P9 measured on 24 Oct 2026, P13 on 25 Oct, P15 on 26 Oct, P17 on
27 Oct.

| Motion | mjlab P13 | mjlab P15 / P17 / P9 | mc_mujoco P13 (est) | Torso error, P13 / P15 / P17 (est) | Robot (P13 in slot 3) |
|---|---|---|---|---|---|
| 15 s | 0 / 64 | 0 / 0 / 0 | no fall | 0.038 / 0.027 / 0.020 m | ready |
| stops 30 s | 0 / 64 | 0 / 0 / 0 | no fall | 0.057 / 0.035 / 0.028 m | ready |
| stops 45 s | 0 / 64 | 0 / 0 / 0 | no fall | 0.071 / 0.041 / 0.033 m | not in the repo |
| stops 60 s | 0 / 64 | 0 / 0 / 0 | no fall | 0.086 / 0.045 / 0.037 m | not in the repo |

mc_mujoco: P15 and P17 no fall (est) on all four; P9 no fall (est and gt) on all four.

**The arms follow the swing only partly.** The policies swing their arms their own way while walking, even on the
default-arm walks (P15: shoulder and elbow 0.10-0.14 rad peak to peak there). On the 15 s motion, in mjlab from 4 to
11 s (median of 32 robots):
- P15 follows shoulders and elbows with correlation 0.65-0.72 and swings them 0.17-0.19 rad peak to peak (reference
  0.11-0.15 rad);
- P13 follows both shoulders and the right elbow (0.78-0.84), but swings its left arm about twice as much, the left
  elbow (0.37 rad) unrelated to the reference;
- P17 follows with correlation 0.50-0.89.
