# Motions

Reference motions of the tracking slots, one folder per motion set. Each folder has its CSVs and a README with what
each motion does and how often the policies fell on it in simulation.
- The GUI (`NewRLQPController/Tracking`) lists every CSV here by its path, for example
  `clips/motion_stand_seed104_10s.csv`, and selects one while disarmed.
- `tracking.motion` takes the same path.
- `motions_dir` in a slot's `tracking` section can point at this folder to use new files without reinstalling.

**Stage 1, walking first (23 Oct).** The repository holds only the 10 s stand and the default-arm walks. The other sets
move the arms through random poses that twist and roll the wrists oddly; their CSVs are kept in
`hrp5p_motion_tracking_deployment/motions_not_ready/` and come back in later stages (arm swing, then gestures). Their
READMEs stay here as the record of their rates.

**Stage 2, arm swing (30 Oct).** One walk with arm motion joins them: `walk_natural_swing/motion_walk_natural_swing_15s.csv`,
the 15 s default-arm walk with a gentle arm swing at the shoulders and elbows; the wrists keep their stance angles.

| Folder | Set | In the repository now |
|---|---|---|
| [walk_default_arms/](walk_default_arms/README.md) | straight walks at 0.15 m/s, arms kept in their default pose (no arm targets of ours), continuous or stop and go | all seven: continuous 15, 30, 45, 60 s; stop-and-go 30, 45, 60 s |
| [walk_natural_swing/](walk_natural_swing/README.md) | the default-arm walks with an arm swing timed by the footsteps, wrists kept at their stance angles | the 15 s walk |
| [clips/](clips/README.md) | cut from the 60 s trial motions, each ending standing: a 10 s stand, an 18 s and a 35 s clip | the 10 s stand |
| [walk/](walk/README.md) | straight walks at 0.15 m/s, arms still in a raised "ready" pose | none (stage 1) |
| [walk_swing/](walk_swing/README.md) | straight walks at 0.15 m/s with arm swings while walking | none (stage 1) |
| [walk_gesture/](walk_gesture/README.md) | short straight walks with arm reaches only while standing | none (stage 1) |
| [full_60s/](full_60s/README.md) | P6's five 60 s trial motions, whole | none (stage 1) |

## Failure rates

Each folder's README gives, per motion and policy:
- **mjlab:** how many of 64 simulated robots fell over the whole motion.
  - The simulation models this controller: the PostureTask, the kinematic QP and the robot's servos.
  - It runs at the robot's 5 ms tick with a 10 ms command delay.
  - A fall is the torso dropping or tilting past the training's limits. The end-effector and path limits are off, so
    a robot that only drifts from the path is not counted.
- **mc_mujoco:** this controller in mc_mujoco, at the 5 ms tick with the robot's servo gains. One run with the MCWaiko
  estimate (est) and one with the simulator's true base (gt).

**Robot column.** "ready" means the policy in slot 3 had 0 falls in both. Only ready motions of the current stage are in
the repository; the others stay listed in their folder's README with their rates, and their CSVs are kept outside it (in
`hrp5p_motion_tracking_deployment/motions_not_ready/`), so the GUI never offers one.

**Policies.**
- Slot 3 runs **P13** (`policy/tracking/P13_21994`, 25 Oct): P11 fine-tuned with adaptive sampling that practises
  more where robots fall.
- Slot 4 runs **P15** (`policy/tracking/P15_22993`, 26 Oct), the second choice with slot 3's settings: P13 fine-tuned
  with penalties on servo torque requests near the limits and on joint accelerations. Each folder's README gives its
  rates under "Slot 4"; it is ready on every motion in the repository too.
- Slot 5 runs **P17** (`policy/tracking/P17_23992`, 27 Oct), the gentlest, with slot 3's settings: P15 fine-tuned
  with both penalties doubled. Each folder's README gives its rates under "Slot 5"; it is ready on every motion in the
  repository too, though in simulation it recovers from pushes less well than P15 and falls more when the command
  delay is longer than the 10 ms of the checks (60 s continuous walk at 20 / 30 ms: 5 and 10 of 64, P15 0 and 1).
- **P11** (`policy/tracking/P11_19996`, P9 plus long straight walks and default-arm walks), **P9**
  (`policy/tracking/P9_18997`, P8 at the robot's 5 ms tick) and **P8** (`policy/tracking/P8_17998`) ran in slot 3
  before; their rates are kept for comparison.

## Format

One row per 20 ms frame:
- `t`;
- then `q_<joint>` and `qd_<joint>` for the robot's 53 joints, in mjlab's order;
- then the anchor (`Chest_Link2`) position and quaternion (w, x, y, z) in the motion's world frame.

## Add a motion or a set

1. Make the mjlab motion (.npz) at 50 fps. It should start standing in the robot's start pose and end standing still.
   - To cut a clip from a longer motion, use `hrp5p_motion_tracking_deployment/tools/make_clip.py`.
   - For new walking motions, use the generator (private `Dhanushvarma/hrp5p_mc_rtc_motions`, branch
     `topic/deploy-motion-sets`).
2. Write its CSV with `export_deploy.py`'s `_write_motion_csv` (mjlab `scripts/hrp5p`), anchor `Chest_Link2`.
3. Put it in its set's folder, or in a new folder with a README.
4. Measure its failure rates as above and record them in the folder's README before anyone runs it on the robot.

After a reinstall into a prefix that held the old layout, delete the CSVs left at the top of
`share/NewRLQPController/motions`, or the GUI lists them twice.
