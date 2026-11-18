# walk_axes_long: the walk_axes walks over a longer distance

The walk_axes walks at the same speeds, walking longer: about 3 m forward, 2 m back and 2 m to each side. There are no
turns, because walk_axes already turns 161° in place. Same generator and option as walk_axes (configs
`motions/walk_axes_long_*.yaml`). Each motion stands 3 s, walks at one constant command, lands its last step about 4 s
after the command ends, then stands still to the end.

On the robot, the test area is ±3 m in x and ±2 m in y around the centre. Start each walk so that its path is centred
in the area:
- forward: from 1.5 m behind the centre;
- backward: from 1 m in front of the centre;
- left: from 1 m right of the centre;
- right: from 1 m left of the centre.

| File | Command | Travel (measured) | In the repo |
|---|---|---|---|
| `motion_walk_axes_long_forward_30s.csv` | 0.15 m/s forward | 3.06 m forward | yes |
| `motion_walk_axes_long_backward_31s.csv` | 0.10 m/s backward | 2.07 m back | no: falls in simulation |
| `motion_walk_axes_long_left_40s.csv` | 0.07 m/s to the left | 2.09 m left | no: falls in simulation |
| `motion_walk_axes_long_right_40s.csv` | 0.07 m/s to the right | 2.17 m right | no: falls in simulation |

## Failure rates

Method: [../README.md](../README.md#failure-rates). Measured on 18-19 Nov 2026. The torso error is that of the
controller in mc_mujoco with the MCWaiko estimate.

| Motion | mjlab falls P15 / P21 / P17 | mc_mujoco P15, P21, P17 (est and gt) | Torso error P15 / P21 / P17 | Robot |
|---|---|---|---|---|
| forward | 0 / 0 / 0 of 64 | no fall | 0.059 / 0.073 / 0.034 m | ready |
| backward | 0 / 1 / 0 of 64 | no fall | 0.173 / 0.180 / 0.218 m | not ready |
| sideways left | 3 / 8 / 3 of 64 | no fall | 0.337 / 0.272 / 0.201 m | not ready |
| sideways right | 9 / 4 / 9 of 64 | no fall | 0.131 / 0.289 / 0.281 m | not ready |

P15 through the 5 s soft hand-over (branch `hrp5p-motion-tracking-soft-handover`): no fall on all four, est and gt.

Only the forward walk is ready, so only its CSV is in this folder. The other three stay listed here with their rates.
Their CSVs are kept outside the repository, in `hrp5p_motion_tracking_deployment/motions_not_ready/walk_axes_long/`.
- **Backward:** one of P21's 64 robots fell at 26 s, as the walk comes to a stop. P15 and P17 had none.
- **Sideways:** every policy fell after 22 s or more of continuous sideways walking (at 25-38 s). The policies' training
  has sideways segments of 4-8 s. The single mc_mujoco runs had no fall, but mjlab's 64 robots show the risk. The 15 s
  sideways walks of `walk_axes/` had 0 falls of 64 for all three policies.

## Landings

As in `walk_axes/`: the highest vertical foot force of each stance in body weights, 9 in 10 below / hardest, through
this controller in mc_mujoco with the estimate.

| Motion | P21 (slot 3) | P15 (slot 4) | P17 (slot 5) |
|---|---|---|---|
| forward | 1.27 / 1.28 | 1.54 / 1.56 | 1.55 / 1.59 |
| backward | 1.24 / 1.25 | 1.31 / 1.33 | 1.29 / 1.31 |
| sideways left | 1.58 / 2.62 | 1.83 / 1.91 | 2.20 / 3.05 |
| sideways right | 1.80 / 1.81 | 1.72 / 1.74 | 1.58 / 1.69 |
