# Usage: HRP-5P motion tracking

Branch `hrp5p-motion-tracking`: `Leonassim/new-rl-qp-controller` @ `hrp5p`, plus whole-body motion tracking in policy
slot 3 and safety guards for every slot. What changed: [CHANGELOG.md](CHANGELOG.md).

## Build and install

```bash
cmake -B build -DCMAKE_INSTALL_PREFIX=<prefix>
cmake --build build -j
cmake --install build
```

- The controller has the same name as the `hrp5p` branch's (`NewRLQPController`). Installing into the same prefix
  replaces that build, so back it up first or install into another prefix.
- Installed files:
  - the controller;
  - its config, `<prefix>/lib/mc_controller/etc/NewRLQPController.yaml`;
  - every ONNX of `policy/`, in `<prefix>/share/NewRLQPController/policy/`;
  - the motions, in `<prefix>/share/NewRLQPController/motions/`.

## Run

- `mc_rtc.yaml`: `MainRobot: HRP5P`, `Enabled: NewRLQPController`.
- The controller starts disarmed on `default_policy_index` (0).
- Policies step every 20 ms at a 2 or 5 ms tick. The robot PC runs at 5 ms.

| Slot | Policy | Kind |
|---|---|---|
| 0 | `push7.onnx` | velocity (joystick) |
| 1 | `angmom.onnx` | velocity (joystick) |
| 2 | `push11.onnx` | velocity (joystick) |
| 3 | `tracking/P21_23992/policy.onnx` | whole-body motion tracking, the softest landings |
| 4 | `tracking/P15_22993/policy.onnx` | whole-body motion tracking, the most robust (slot 3's settings) |
| 5 | `tracking/P17_23992/policy.onnx` | whole-body motion tracking, the gentlest (slot 3's settings) |

## Track a motion (slot 3)

In the GUI, under `NewRLQPController`:

1. `Policy` → `Policy index (disarm to change)`: 3 (P21, the softest landings), 4 (P15, the most robust) or 5 (P17, the
   gentlest).
2. `Tracking` → `Motion (disarm to change)`: pick a motion, listed by set folder (e.g.
   `clips/motion_stand_seed104_10s.csv`). The default is the 10 s stand. `Rescan motions directory` finds files added
   since the start.
3. `ARM policy`:
   - the robot ramps to the motion's first frame and settles for 2 s;
   - it then tracks the motion from where it stands and faces;
   - after the last frame it holds that frame (`Tracking state`: ended).
4. `HOLD (disarm)` at any time: the posture target freezes at the last command.
5. ARM only from the standing pose at the start. ARM always restarts the motion from its first frame: after a HOLD
   mid-motion the feet can be apart, and the ramp to the first frame (feet side by side) would slide a foot. Do not
   ARM again then; bring the robot back to its standing pose first (in mc_mujoco: restart it).

For the first trials (stage 1, walking first), run one motion at a time with a hand on the stop:
1. `clips/motion_stand_seed104_10s.csv` (stands still);
2. `walk_default_arms/motion_walk_default_arms_15s.csv` (walks straight about 0.8 m, arms in their default pose);
3. `walk_default_arms/motion_walk_default_arms_stops_30s.csv` (two short walks with a stop);
4. `walk_default_arms/motion_walk_default_arms_stops_45s.csv`, then `..._stops_60s.csv` (three and four short walks).

[motions/README.md](motions/README.md) lists the motion sets, how their failure rates are measured, and how to add one.

## Safety guards

Each guard HOLDs the policy: the posture target freezes at the last command. The GUI shows the reason under `Last
disarm`; the log records it as `NewRLQPController_disarmReason`.

| Reason (log value) | Trigger | Setting (per slot) |
|---|---|---|
| Operator (1) | `HOLD (disarm)` | |
| Runaway (2) | a measured joint speed above the limit | `runaway_disarm_vel` (rad/s, 0 = off) |
| Tilt (3) | the torso's tilt from vertical, as the policy observes it | `fall_disarm_tilt` (rad, 0 = off, 0.5 if absent) |
| Inference (4) | a failed policy inference | |
| Tracking (5) | a tracking error (non-finite observation, QP bypassed, ...) | |

While the policy is armed, the GUI also refuses:
- policy and motion switches;
- the QP and posture mode toggles;
- the PostureTask gain inputs.

## Settings

Where to change them:
- **Source file:** edit `etc/NewRLQPController.in.yaml`, then reinstall.
- **Installed file:** edit `NewRLQPController.yaml` directly.
- **User override** (`~/.config/mc_rtc/controllers/NewRLQPController.yaml`): it merges maps but replaces lists, so use
  it for top-level keys only. `policy_path` and `policies` are lists.

| Key | Where | Purpose |
|---|---|---|
| `verbose_log` | top level | `false`: lean log. `true`: adds the observation, constants and joint copies (about twice the volume). Use `true` for the first robot trials. |
| `default_policy_index` | top level | slot selected at start |
| `policy_path` | top level | one ONNX per slot; slot 3 is the tracking policy |
| `tracking.motion` | slot 3 | motion selected at start, by its path in the motions folder (e.g. `clips/motion_stand_seed104_10s.csv`) |
| `tracking.motions_dir` | slot 3 | defaults to the installed motions; point it at the repository's `motions/` to skip reinstalling |
| `tracking.base_source` | slot 3 | `estimate` (robot) or `ground_truth` (simulator only) |
| `tracking.arm_ramp_speed`, `tracking.arm_settle_time` | slot 3 | arming ramp speed (rad/s) and settle time (s) |
| `fall_disarm_tilt`, `runaway_disarm_vel` | each slot | the guards above |
| `sim_servo_gains` | top level | simulator only: a servo gains file for mc_mujoco's PD; ignored on the robot |

## Swap the tracking policy

1. Export the policy with mjlab's `scripts/hrp5p/export_deploy.py`. It writes `policy.onnx` and `policy.yaml`.
2. Copy the ONNX to `policy/tracking/<name>/policy.onnx` and point slot 3 of `policy_path` at it.
3. Check slot 3 against the export's `policy.yaml`: `action_scale`, `q0`, `kp`, `kd`, `ref_joint_order` and the
   `tracking` section. P6 final, P8, P9, P11, P13, P15, P17 and P21 share these values.
4. Rebuild and install.

## Logs

| Entry | Content |
|---|---|
| `Tracking_state` | 0 idle, 1 ramping, 2 settling, 3 tracking, 4 ended |
| `Tracking_motion`, `Tracking_motionTime`, `Tracking_frame` | the motion and the position in it |
| `Tracking_anchorPos`, `Tracking_anchorQuat` | the torso as observed |
| `Tracking_refAnchorPos`, `Tracking_refAnchorQuat` | the torso in the reference |
| `Tracking_est*`, `Tracking_gt*` | estimated and ground-truth (simulator) base, every tick |
| `Tracking_action`, `Tracking_target` | the raw policy output and the filtered joint target |
| `NewRLQPController_armed`, `_disarmReason`, `_torsoTilt`, `_maxJointVel`, `_maxJointVelIndex` | guard state |

## Plots

`tools/plot_mc_log.py` turns a run's log into PNG plots, from mc_mujoco or the robot. It needs numpy, matplotlib and
`mc_bin_utils` from the mc_rtc install, not the mc_rtc Python bindings.

```bash
python3 tools/plot_mc_log.py                                   # the latest log
python3 tools/plot_mc_log.py LOG.bin --window 20 35 --only feet,legs_torque
python3 tools/plot_mc_log.py LOG.bin --list                    # the log's entries
```

- Without a path it takes `mc-control-NewRLQPController-latest.bin` from `~/workspace/runs/logs` or `/tmp`.
- The plots go to `plots/<log name>/` beside the logs folder, with a `summary.txt`. The script's docstring lists them:
  the tracking overview, the torso path, joint angles and torques over time, the joint tracking error, foot forces,
  foot impacts, per-joint torque peaks, the IMU, controller timing and the policy action.
- `impact` shows each ankle's vertical velocity and each foot's force around touchdowns, and per landing the velocity
  just before touchdown and the largest force in the first 100 ms. The ankle velocity comes from the base pose and the
  encoders through `HRP5Pmain.urdf`, found in the mc_rtc install (or `--urdf PATH`).
- On the robot `tauIn` holds the motor currents, so the torque plots are in A, not N m.

## Tests

```bash
cmake -B build -DBUILD_TESTING=ON -DTRACKING_GOLDEN_CSV=<golden.csv> -DTRACKING_POLICY_ONNX=<policy.onnx>
cmake --build build -j && ctest --test-dir build
```

- `export_deploy.py` writes the golden CSV next to the policy it was made from. The CSV is about 19 MB and is not in
  the repository.
- The tests rebuild the observation and the action from the golden CSV.
- If an older install is on `LD_LIBRARY_PATH`, put `build/src` first. Otherwise the tests load the installed library.
