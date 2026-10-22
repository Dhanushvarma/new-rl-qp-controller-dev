# Motions

Reference motions of the tracking slots, one CSV each. The GUI (`NewRLQPController/Tracking`) selects any of them
while disarmed; `motions_dir` in a slot's `tracking` section can point at this folder to use new files without
reinstalling.

Format: one row per 20 ms frame: `t`, then `q_<joint>` and `qd_<joint>` for the robot's 53 joints in mjlab's order,
then the anchor (`Chest_Link2`) position and quaternion (w, x, y, z) in the motion's world frame.

| File | Length | Content |
|---|---|---|
| `motion_stand_seed104_10s.csv` | 10 s | standing: seed104's frame at 0.5 s, held |
| `motion_clip_seed104_15s.csv` | 18.4 s | seed104 from 0 to 15.4 s (0.84 m of walking, at most 0.22 m/s), then 3 s standing |
| `motion_clip_seed084_31s.csv` | 34.6 s | seed084 from 0 to 31.6 s (2.69 m), then 3 s standing |
| `motion_clip_seed062_42s.csv` | 45.7 s | seed062 from 0 to 42.7 s (5.66 m), then 3 s standing |
| `motion_bwc_v2_seed{010,060,062,084,104}.csv` | 60 s | P6's held-out trial motions, whole |

The clips end where the reference stands still (base and feet at rest) and hold that frame with zero velocities,
as the tracker does after a motion's last frame. To add one: cut the mjlab motion (.npz) with
`hrp5p_motion_tracking_deployment/tools/make_clip.py`, then write its CSV with `export_deploy.py`'s
`_write_motion_csv` (mjlab `scripts/hrp5p`), anchor `Chest_Link2`.
