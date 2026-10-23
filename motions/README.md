# Motions

Reference motions of the tracking slots, one CSV each. The GUI (`NewRLQPController/Tracking`) selects any of them
while disarmed; `motions_dir` in a slot's `tracking` section can point at this folder to use new files without
reinstalling.

Format: one row per 20 ms frame: `t`, then `q_<joint>` and `qd_<joint>` for the robot's 53 joints in mjlab's order,
then the anchor (`Chest_Link2`) position and quaternion (w, x, y, z) in the motion's world frame.

| File | Length | Source | What the reference does |
|---|---|---|---|
| `motion_stand_seed104_10s.csv` | 10 s | seed104's frame at 0.5 s, held | stands still, arms in the start pose |
| `motion_clip_seed104_15s.csv` | 18.4 s | seed104 from 0 to 15.4 s, then 3 s held | stands 2 s, raises both arms to about chest height (left first, right by 9 s), turns about 36° left in place with small steps (6 to 10 s, feet move 7 cm at most), stands with the arms raised; ends 6 cm from its start |
| `motion_clip_seed084_31s.csv` | 34.6 s | seed084 from 0 to 31.6 s, then 3 s held | raises the arms, steps back about 0.5 m (6 to 12 s), turns about 165° left with steps (12 to 24 s), stands moving the arms; ends 0.35 m from its start, facing almost backward |
| `motion_clip_seed062_42s.csv` | 45.7 s | seed062 from 0 to 42.7 s, then 3 s held | raises the arms, walks about 1.3 m forward while turning about 40° right (6 to 18 s), steps back about 0.3 m (18 to 30 s), walks about 1.5 m forward (30 to 39 s), stands; ends 2.5 m from its start |
| `motion_bwc_v2_seed{010,060,062,084,104}.csv` | 60 s | P6's held-out trial motions, whole | arm gestures, steps in every direction and turns (up to about 165°); each ends 1 to 3 m from its start |

Distances are those of the base from the motion's start. Its path length is larger, mostly from sway (0.84 m for the
15 s clip).

The clips end where the reference stands still (base and feet at rest) and hold that frame with zero velocities,
as the tracker does after a motion's last frame. To add one: cut the mjlab motion (.npz) with
`hrp5p_motion_tracking_deployment/tools/make_clip.py`, then write its CSV with `export_deploy.py`'s
`_write_motion_csv` (mjlab `scripts/hrp5p`), anchor `Chest_Link2`.
