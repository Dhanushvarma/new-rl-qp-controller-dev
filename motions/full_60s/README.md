# full_60s: P6's five 60 s trial motions, whole

Made by the BaselineWalkingController generator with random walking segments (forward, backward, sideways, turns, arcs)
and random arm gestures (swings, reaches, waves). Distances are those of the torso from the motion's start.

| File | What the reference does |
|---|---|
| `motion_bwc_v2_seed010.csv` | steps back and to the left, walks forward about 1.5 m while turning about 90° right, gestures; ends 1.1 m from its start |
| `motion_bwc_v2_seed060.csv` | turns about 85° right, steps about 1 m sideways, walks about 2.5 m forward and to the right; ends 2.9 m from its start |
| `motion_bwc_v2_seed062.csv` | walks about 1.3 m forward while turning about 40° right, steps back, walks forward again, raises both arms at the end; ends 2.8 m from its start |
| `motion_bwc_v2_seed084.csv` | steps back about 0.5 m, turns about 165° left, walks about 1.5 m; ends 1.9 m from its start |
| `motion_bwc_v2_seed104.csv` | turns about 36° left in place with the arms raised, walks about 1.8 m forward and to the left; ends 2.2 m from its start |

## Failure rates

Method: [../README.md](../README.md#failure-rates).

| Motion | mjlab P8 | mjlab P9 | mc_mujoco | Robot (P9 in slot 3) |
|---|---|---|---|---|
| seed010 | 2 falls | being measured | not run | not yet |
| seed060 | 4 falls | being measured | not run | not yet |
| seed062 | 3 falls | being measured | not run | not yet |
| seed084 | 1 fall | being measured | not run | not yet |
| seed104 | 0 falls | being measured | not run | not yet |

P8 (22 Oct): 10 of 128 robots fell over the five motions together, about 26 robots per motion. Per-motion runs of 64
robots for P8 and P9 are being measured.
