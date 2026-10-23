# Changelog over `Leonassim/new-rl-qp-controller` @ `hrp5p`

`Dhanushvarma/new-rl-qp-controller-dev`, branch `hrp5p-motion-tracking`, on top of `hrp5p` at `8e65d5e`. How to use it:
[USAGE.md](USAGE.md).

## Branches

| Branch | Head | Content |
|---|---|---|
| `hrp5p-motion-tracking` | latest | everything below; the branch to build |
| `hrp5p-motion-tracking-safety` | `c160847` | see below |

The safety branch held the four safety commits (`e302aeb` to `c160847`) for review.
- On 23 Oct they were fast-forwarded into `hrp5p-motion-tracking`.
- It has nothing the main branch lacks.

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
- `17633ec`: slot 3's default motion is the 10 s stand.
- Motion sets: the motions directory has one folder per set (`clips/`, `full_60s/`, ...), each with a README of what
  its motions do and their failure rates in simulation. The tracker lists the CSVs of the subfolders by their path
  (`clips/motion_stand_seed104_10s.csv`), which `tracking.motion` takes too.
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
