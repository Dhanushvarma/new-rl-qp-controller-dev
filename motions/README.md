# Motions

Reference motions of the tracking slots, one folder per motion set. Each folder has its CSVs and a README with what
each motion does and how often the policies fell on it in simulation.
- The GUI (`NewRLQPController/Tracking`) lists every CSV here by its path, for example
  `clips/motion_stand_seed104_10s.csv`, and selects one while disarmed.
- `tracking.motion` takes the same path.
- `motions_dir` in a slot's `tracking` section can point at this folder to use new files without reinstalling.

| Folder | Set |
|---|---|
| [clips/](clips/README.md) | cut from the 60 s trial motions, each ending standing: a 10 s stand, an 18 s and a 35 s clip |
| [full_60s/](full_60s/README.md) | P6's five 60 s trial motions, whole |
| [walk/](walk/README.md) | straight walks at 0.15 m/s, arms still |
| [walk_default_arms/](walk_default_arms/README.md) | straight walks at 0.15 m/s, arms kept in their default pose (no arm targets of ours), continuous or stop and go |
| [walk_swing/](walk_swing/README.md) | straight walks at 0.15 m/s with gentle arm swings while walking |
| [walk_gesture/](walk_gesture/README.md) | short straight walks with slow arm reaches only while standing |

## Failure rates

Each folder's README gives, per motion and policy:
- **mjlab:** how many of 64 simulated robots fell over the whole motion.
  - The simulation models this controller: the PostureTask, the kinematic QP and the robot's servos.
  - It runs at the robot's 5 ms tick with a 10 ms command delay.
  - A fall is the torso dropping or tilting past the training's limits. The end-effector and path limits are off, so
    a robot that only drifts from the path is not counted.
- **mc_mujoco:** this controller in mc_mujoco, at the 5 ms tick with the robot's servo gains. One run with the MCWaiko
  estimate (est) and one with the simulator's true base (gt).

**Robot column.** "ready" means the policy in slot 3 had 0 falls in both. Only ready motions are in the repository
(one exception by decision, `walk/motion_walk_30s.csv`: 1 fall in 64, see its README);
the others stay listed in their folder's README with their rates, and their CSVs are kept outside it (in
`hrp5p_motion_tracking_deployment/motions_not_ready/`), so the GUI never offers one.

**Policies.**
- Slot 3 runs **P9** (`policy/tracking/P9_18997`): P8 fine-tuned at the robot's 5 ms tick only.
- **P8** (`policy/tracking/P8_17998`) ran in slot 3 before; its rates are kept for comparison.

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
