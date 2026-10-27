# Changelog over `Leonassim/new-rl-qp-controller` @ `hrp5p`

`Dhanushvarma/new-rl-qp-controller-dev`, branch `hrp5p-motion-tracking`, on top of `hrp5p` at `8e65d5e`. How to use it:
[USAGE.md](USAGE.md).

## Branches

| Branch | Head | Content |
|---|---|---|
| `hrp5p-motion-tracking` | latest | everything below; the branch to build |
| `hrp5p-motion-tracking-safety` | `c160847` | see below |
| `hrp5p-motion-tracking-soft-handover` | latest | `hrp5p-motion-tracking` plus the soft hand-over below |

The safety branch held the four safety commits (`e302aeb` to `c160847`) for review.
- On 23 Oct they were fast-forwarded into `hrp5p-motion-tracking`.
- It has nothing the main branch lacks.

The soft hand-over branch (27 Oct), not yet in `hrp5p-motion-tracking`:
- ARM adds a hand-over after the settle (`tracking.arm_handover_time`, 1 s; 0 = none). The motion is aligned
  and the policy runs on its first frame held still, while the posture target blends from the first frame to
  the policy's filtered target (min-jerk, every tick). Tracking then starts from frame 0, keeping the filter
  and previous action.
- Why: the policy's first command differs from the first frame (P15 in mc_mujoco: up to 0.20 rad, right
  shoulder; 0.13 rad, left knee). Before, the filter passed 40 % of it in one step: a target step of up to
  0.09 rad and a jerk at the tracking start.
- `Tracking_state` 5 = handing over (GUI: `handing over`).

## Motion tracking (new, policy slot 3)

- `c9bb11e`: tracking motion and observation.
  - Reads the motion CSVs written by mjlab's `export_deploy.py`.
  - Builds the 220-d observation of the HRP-5P tracking policies (`obs_format: 9`).
- `a6b677c`: `RLPolicyInterface::customMetadata` reads custom ONNX metadata.
- `99f6c17`: `MotionTracker`.
  - A motions directory with a GUI selector.
  - ARM: a min-jerk ramp to the motion's first frame, a settle, then tracking from frame 0, aligned to the robot.
  - Each command goes through the 4 Hz action filter and the joint-range clamp, then to the PostureTask.
  - `Tracking_*` logs.
- `3e5f5a1`: `tracking.base_source` picks the observed base.
  - `estimate`: the observer pipeline and the gyro.
  - `ground_truth`: simulator only.
  - Both are logged every tick.
- `e199264`, `fd31cfb`: `motions/`, installed to `share/NewRLQPController/motions`.
  - Five 60 s held-out walks.
  - Four short clips that end standing: a 10 s stand, then 18, 35 and 46 s clips.
- `71185f9`: P6 final in slot 3. It stays in the repository as the tests' golden policy. CMake installs the whole
  `policy/` tree.
- `6cb7d2f`: P8 runs in slot 3. It is P6 final fine-tuned through mjlab's model of this posture chain.
- P9 replaces P8 in slot 3: P8 fine-tuned 1000 more iterations at the robot's 5 ms tick only. Over whole 59 s motions
  in mjlab at 5 ms, 4 of 128 robots fall on the held-out motions (P8: 41) and 91 of 512 on the training ones (P8: 148);
  its legs move a little more (15-30 % more jerk in mc_mujoco). P8's file stays in `policy/tracking/P8_17998/`.
- P11 replaces P9 in slot 3: P9 fine-tuned 1000 more iterations with long straight walks and default-arm walks added
  to its motions. In mjlab at 5 ms no robot of 64 falls on any default-arm walk, the 30, 45 and 60 s continuous ones
  included (P9: 1, 11, 14), and in mc_mujoco its torso and wrist errors on them are about half of P9's. It falls more
  than P9 on long walks with the arms held raised and on the held-out random-arm motions (20 of 128, P9: 4), which stage
  1 does not use. P9's file stays in `policy/tracking/P9_18997/`.
- `walk_default_arms/` gets back its continuous 30, 45 and 60 s walks, which P11 tracks without a fall.
- P13 replaces P11 in slot 3: P11 fine-tuned 2000 more iterations on the same motions with adaptive sampling that
  practises more where robots fall. In mjlab at 5 ms no robot of 64 falls on the stand or any default-arm walk (as
  P11); over whole motions 31 of 512 fall on the training motions (P11: 93) and 2 of 128 on the held-out ones (P11:
  20). In mc_mujoco it tracks every stage-1 motion to its end, drifts least with the MCWaiko estimate on the long walks
  (60 s: 0.09 m, P11 0.22) and trips no guard. P11's file stays in `policy/tracking/P11_19996/`.
- Slot 4: a second motion tracking slot, P15, with slot 3's settings (a YAML anchor). P15 is P13 fine-tuned with
  penalties on servo torque requests near the limits and on joint accelerations, its exploration noise lowered. It
  passes every stage-1 motion like P13 and falls much less on long walks with the arms held raised (45 / 60 s: 7 and
  14 of 64, P13 53 and 57).
- Slot 5: a third motion tracking slot, P17, with slot 3's settings. P17 is P15 fine-tuned with both penalties
  doubled: the gentlest (servo torque requests near the limits about 3 times rarer than P15's). It passes every
  stage-1 motion and holds the torso closest to the reference through the controller; it tracks the wrists'
  orientation less closely and, in simulation, recovers from pushes less well and falls more when the command delay
  is longer than the 10 ms of the checks (60 s continuous walk at 20 / 30 ms: 5 and 10 of 64, P15 0 and 1).
- `17633ec`: slot 3's default motion is the 10 s stand.
- Motion sets: the motions directory has one folder per set (`clips/`, `full_60s/`, ...), each with a README of what
  its motions do and their failure rates in simulation. The tracker lists the CSVs of the subfolders by their path
  (`clips/motion_stand_seed104_10s.csv`), which `tracking.motion` takes too.
- Stage 1, walking first: the motions folder keeps only the 10 s stand and the default-arm walks (`walk_default_arms/`,
  generated with no arm targets); the random-arm sets (odd wrist poses) are out of the repository for now, their READMEs
  kept as records.
- Tests: `tests/tracking` compares the observation and the policy output with mjlab golden CSVs (`-DTRACKING_GOLDEN_CSV`).

## Safety

- `e302aeb`: every policy slot loads at startup, so a GUI switch reads nothing from disk.
  - A slot that fails to load is shown in the GUI, and switching to it is refused.
  - The default slot failing to load still stops the start.
- `ed43010`: a failed inference disarms and holds the last target. Before, the zero action sent the robot to `q0`.
- `b9cf3ad`: fall guard. It disarms when the observed torso tilt exceeds `fall_disarm_tilt` (0.5 rad if absent; still
  to be confirmed on the robot). Disarm reasons are logged (`NewRLQPController_disarmReason`) and shown in the GUI
  (`Last disarm`).
- `bb80f36`: the runaway guard names the joint. Logs `NewRLQPController_maxJointVel`, `_maxJointVelIndex` and
  `_armed`.
- `06102c1`: the GUI refuses mode toggles and PostureTask gain inputs while armed.
- `3bfcc71`: while disarmed, `q_rl` follows the held posture target, so the arming tick sees no stale value.

## Estimation

- `63252a6`: MCWaiko uses 6D legged odometry with the lab controller's filter gains. The estimate now follows yaw and
  horizontal position, which the tracking observation needs.

## Logging

- `c160847`: top-level `verbose_log` (default `false`) gates the bulky entries: the observation, constants, joint
  copies and self-collision distances. That is about half the per-tick log. Simulation analyses that read those
  entries need `true`.

## Simulation only (no effect on the robot)

- `37bebba`: `sim_servo_gains` hands mc_mujoco a servo gains file.
- `6e258c5`: slot 3 sends the QP velocity to mc_mujoco's PD (`qp_zero_vel_out: false`). This matches the damping of
  the robot's servo.

## What changes for slots 0 to 2

- The MCWaiko settings above apply to every slot.
- The fall guard is on at 0.5 rad, since these slots have no `fall_disarm_tilt` key.
- A failed inference disarms instead of commanding `q0`.
- Policies preload at startup, and the GUI refuses mode toggles while armed.
- The log has fewer entries unless `verbose_log: true`.
