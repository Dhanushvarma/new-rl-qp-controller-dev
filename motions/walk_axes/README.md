# walk_axes: walking along one axis or turning in place, arms in their default pose

Made for the robot test of 19 Nov 2026 with the BaselineWalkingController generator (private
`Dhanushvarma/hrp5p_mc_rtc_motions`, branch `topic/footstep-swing`, configs `motions/walk_axes_*.yaml`, option
`arms: hold_start`). Each motion:
- stands 3 s;
- walks from 3 s to 18 s at one constant command;
- lands its last step at about 22 s;
- stands still to 24 s.

The arms keep their default pose, and sideways walks keep facing forward. The speeds sit in the middle of what the
policies were trained on. The generator's random training set has forward 0.05-0.25 m/s, backward 0.05-0.15 m/s,
sideways 0.03-0.10 m/s and turning 0.1-0.3 rad/s.

On the robot, the lab's test area is ±3 m in x and ±2 m in y around the start. Start at the centre facing +x, and run
the pairs back to back: forward then backward, turn left then turn right, left then right. Each pair brings the robot
back near its start.

| File | Command | Travel (measured) |
|---|---|---|
| `motion_walk_axes_forward_24s.csv` | 0.15 m/s forward | 2.14 m forward |
| `motion_walk_axes_backward_24s.csv` | 0.10 m/s backward | 1.38 m back |
| `motion_walk_axes_left_24s.csv` | 0.07 m/s to the left | 0.97 m left |
| `motion_walk_axes_right_24s.csv` | 0.07 m/s to the right | 1.03 m right |
| `motion_walk_axes_turn_left_24s.csv` | 0.2 rad/s anti-clockwise | 161° in place (0.03 m) |
| `motion_walk_axes_turn_right_24s.csv` | 0.2 rad/s clockwise | 161° in place (0.03 m) |

## Failure rates

Method: [../README.md](../README.md#failure-rates). Measured on 18-19 Nov 2026. The torso error is that of the
controller in mc_mujoco with the MCWaiko estimate.

| Motion | mjlab falls P15 / P21 / P17 | mc_mujoco P15, P21, P17 (est and gt) | Torso error P15 / P21 / P17 | Robot |
|---|---|---|---|---|
| forward | 0 / 0 / 0 of 64 | no fall | 0.053 / 0.056 / 0.028 m | ready |
| backward | 0 / 0 / 0 of 64 | no fall | 0.122 / 0.162 / 0.175 m | ready |
| sideways left | 0 / 0 / 0 of 64 | no fall | 0.214 / 0.193 / 0.204 m | ready |
| sideways right | 0 / 0 / 0 of 64 | no fall | 0.085 / 0.227 / 0.173 m | ready |
| turn left | 0 / 0 / 0 of 64 | no fall | 0.034 / 0.053 / 0.079 m | ready |
| turn right | 0 / 0 / 0 of 64 | no fall | 0.028 / 0.080 / 0.058 m | ready |

P15 through the 5 s soft hand-over (branch `hrp5p-motion-tracking-soft-handover`): no fall on all six, est and gt.

## Landings

How hard the feet land, through this controller in mc_mujoco with the estimate. For each stance, the highest vertical
foot force, in body weights (about 990 N on the foot sensors), pooled over the six motions. A tap is a foot that leaves
the ground and comes back within 0.15 s.

| Policy | Median | 9 in 10 below | Hardest | Taps |
|---|---|---|---|---|
| P15 (slot 4) | 1.20 | 1.57 | 2.19 (sideways right) | 115 |
| P21 (slot 3) | 1.13 | 1.38 | 1.67 | 64 |
| P17 (slot 5) | 1.24 | 1.57 | 1.95 | 64 |
| BaselineWalkingController (the reference) | 1.07 | 1.08 | 1.08 | 16 |

The sideways walks give the hardest landings for every policy.
