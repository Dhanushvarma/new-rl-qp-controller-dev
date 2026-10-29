#!/usr/bin/env python3
"""Plots of one NewRLQPController run from its mc_rtc log (an mc_mujoco run or the robot).

Run it where mc_rtc is installed (mc_bin_utils on the PATH):
  python3 tools/plot_mc_log.py                       # the latest NewRLQPController log
  python3 tools/plot_mc_log.py LOG.bin --window 20 35 --only feet,legs
  python3 tools/plot_mc_log.py LOG.bin --list        # the log's entries

The latest log is mc-control-NewRLQPController-latest.bin in ~/workspace/runs/logs or /tmp (mc_rtc's default
LogDirectory). PNG files and a short summary.txt go to plots/<log name>/ beside the logs folder (or --out DIR). Needs
numpy, matplotlib and mc_bin_utils, no mc_rtc Python bindings: mc_bin_utils copies the entries used here into a
temporary .flat file, read as mc_log_ui reads it. A plot whose entries the log does not have is skipped.

The time axis is the log's time from the controller start. By default the plots cover the tracking (Tracking_state 3)
with 2 s on each side; dotted lines mark the tracking start and end, red lines a disarm (HOLD or a guard).

Plots:
  overview     tracking state, motion time, torso tilt, anchor height
  base         the anchor's path seen from above, its distance to the reference, yaw
  legs         each leg joint: policy target, command (qOut), encoder (qIn)
  upper_body   the same for the waist, neck and arms
  legs_torque, upper_body_torque
               each joint's torque over time (tauIn: N m in mc_mujoco; on the robot it holds motor currents in A)
  joint_error  per joint, policy target minus encoder while tracking (RMS and largest)
  feet         each foot's vertical force in body weights, and a close-up of a few steps
  torques      per joint, the largest and 95th percentile torque while tracking; the controller's largest
               torque-to-limit ratio over time
  imu          the IMU's linear acceleration and angular velocity
  timing       the controller's time per tick against the tick
  action       the policy's action over time, and how much it changes per policy step
"""

import argparse
import ctypes
import os
import shutil
import subprocess
import sys
import tempfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# mc_hrp5_p's refJointOrder (src/mc_hrp5_p.cpp): the order of qIn, qOut and tauIn
ROBOT_JOINTS = (
    "RCY",
    "RCR",
    "RCP",
    "RKP",
    "RAP",
    "RAR",
    "LCY",
    "LCR",
    "LCP",
    "LKP",
    "LAP",
    "LAR",
    "WP",
    "WR",
    "WY",
    "HY",
    "HP",
    "RSC",
    "RSP",
    "RSR",
    "RSY",
    "REP",
    "RWRY",
    "RWRR",
    "RWRP",
    "RHDY",
    "RTMP",
    "RTPIP",
    "RTDIP",
    "RIMP",
    "RIPIP",
    "RIDIP",
    "RMMP",
    "RMPIP",
    "RMDIP",
    "LSC",
    "LSP",
    "LSR",
    "LSY",
    "LEP",
    "LWRY",
    "LWRR",
    "LWRP",
    "LHDY",
    "LTMP",
    "LTPIP",
    "LTDIP",
    "LIMP",
    "LIPIP",
    "LIDIP",
    "LMMP",
    "LMPIP",
    "LMDIP",
)
# The policy's joints (the controller's ref_joint_order): the order of Tracking_target and Tracking_action
POLICY_JOINTS = (
    "RCY",
    "RCR",
    "RCP",
    "RKP",
    "RAP",
    "RAR",
    "LCY",
    "LCR",
    "LCP",
    "LKP",
    "LAP",
    "LAR",
    "WP",
    "WR",
    "WY",
    "HY",
    "HP",
    "RSC",
    "RSP",
    "RSR",
    "RSY",
    "REP",
    "RWRY",
    "RWRR",
    "RWRP",
    "LSC",
    "LSP",
    "LSR",
    "LSY",
    "LEP",
    "LWRY",
    "LWRR",
    "LWRP",
)
LEG_LAYOUT = (("RCY", "RCR", "RCP", "RKP", "RAP", "RAR"), ("LCY", "LCR", "LCP", "LKP", "LAP", "LAR"))
UPPER_LAYOUT = (
    ("WP", "WR", "WY", "HY", "HP"),
    ("RSC", "RSP", "RSR", "RSY", "REP", "RWRY", "RWRR", "RWRP"),
    ("LSC", "LSP", "LSR", "LSY", "LEP", "LWRY", "LWRR", "LWRP"),
)
# Tracking_state (MotionTracker.cpp), in the order a run goes through them; 5 exists with the soft hand-over
STATES = {0: "idle", 1: "ramping", 2: "settling", 5: "handing over", 3: "tracking", 4: "ended"}
DISARM = ("none", "operator HOLD", "runaway guard", "fall guard (tilt)", "inference failed", "tracking error")
ENTRIES = (
    "t",
    "Tracking_state",
    "Tracking_motion",
    "Tracking_policyIndex",
    "Tracking_motionTime",
    "Tracking_target",
    "Tracking_action",
    "Tracking_refAnchorPos",
    "Tracking_refAnchorQuat",
    "Tracking_gtAnchorPos",
    "Tracking_gtAnchorQuat",
    "Tracking_estAnchorPos",
    "Tracking_estAnchorQuat",
    "NewRLQPController_armed",
    "NewRLQPController_disarmReason",
    "NewRLQPController_torsoTilt",
    "NewRLQPController_rawTorqueRatioMax",
    "NewRLQPController_leftFootContact",
    "NewRLQPController_rightFootContact",
    "LeftFootForceSensor",
    "RightFootForceSensor",
    "Accelerometer_linearAcceleration",
    "Accelerometer_angularVelocity",
    "qIn",
    "qOut",
    "tauIn",
    "perf_GlobalRun",
    "perf_ControllerRun",
    "perf_SolverBuildAndSolve",
)
LATEST = "mc-control-NewRLQPController-latest.bin"
LOG_DIRS = (os.path.expanduser("~/workspace/runs/logs"), "/tmp")

# A colour-blind safe categorical order (blue, orange, aqua), neutral inks for references and text
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, AXIS, GRID, SURFACE, RED = "#0b0b0b", "#52514e", "#c3c2b7", "#e1e0d9", "#fcfcfb", "#d03b3b"
CONTACT_ON, CONTACT_OFF = 30.0, 15.0  # N, the controller's touchdown rule (impact_force_threshold and half of it)
JOINT_KEY = (
    "R/L right/left, C hip (crotch), K knee, A ankle, W waist, H neck, S shoulder, E elbow, WR wrist; "
    "Y/R/P yaw/roll/pitch"
)
TITLES = {
    "overview": "Overview: tracking state, motion time, torso tilt and height",
    "base": "Torso (anchor) position and yaw against the reference",
    "legs": "Leg joint angles [deg]",
    "upper_body": "Upper body joint angles [deg]",
    "joint_error": "Joint tracking error [deg]: policy target minus encoder",
    "feet": "Foot contact forces",
    "legs_torque": "Leg joint torques [N m] (tauIn)",
    "upper_body_torque": "Upper body joint torques [N m] (tauIn)",
    "torques": "Joint torques [N m]: per joint, over the whole tracking (tauIn)",
    "imu": "Torso IMU",
    "timing": "Controller computation time",
    "action": "Policy action",
}


def style():
    plt.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
            "axes.edgecolor": AXIS,
            "axes.labelcolor": INK2,
            "axes.titlecolor": INK,
            "text.color": INK,
            "xtick.color": INK2,
            "ytick.color": INK2,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "lines.linewidth": 1.2,
            "font.size": 9,
            "axes.titlesize": 9,
            "axes.titlelocation": "left",
            "legend.frameon": False,
            "legend.fontsize": 8,
        }
    )


# ---------------------------------------------------------------------------------------------------------- reading


def entries(log):
    """The log's entry names and its timestep (from mc_bin_utils show)."""
    out = subprocess.run(["mc_bin_utils", "show", log], capture_output=True, text=True, check=True).stdout
    names, dt = [], None
    for line in out.splitlines():
        if line.startswith("- "):
            names.append(line[2:])
        elif line.startswith("Timestep:") and dt is None:
            dt = float(line.split()[1].rstrip("s"))
    return names, dt


def read_flat(path):
    """mc_log_ui's .flat reader (mc_log_ui.py, read_flat): numeric entries as arrays, the others as lists."""

    def size(fd):
        return ctypes.c_size_t.from_buffer_copy(fd.read(ctypes.sizeof(ctypes.c_size_t))).value

    data = {}
    with open(path, "rb") as fd:
        for _ in range(size(fd)):
            numeric = ctypes.c_bool.from_buffer_copy(fd.read(ctypes.sizeof(ctypes.c_bool))).value
            key = fd.read(size(fd)).decode("ascii")
            n = size(fd)
            if numeric:
                data[key] = np.frombuffer(fd.read(n * ctypes.sizeof(ctypes.c_double)), np.double)
            else:
                data[key] = [fd.read(size(fd)).decode("ascii", "replace") for _ in range(n)]
    return data


def load(log):
    names, dt = entries(log)
    have = {n.split(" (")[0] for n in names}
    wanted = [e for e in ENTRIES if e in have]
    with tempfile.TemporaryDirectory() as tmp:
        flat = os.path.join(tmp, "log.flat")
        subprocess.run(
            ["mc_bin_utils", "convert", log, flat, "--format", "flat", "--entries", *wanted],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return read_flat(flat), dt


def vec(d, name):
    """A vector entry (VectorXd, VectorDouble) as [T, n], or None."""
    cols = []
    while f"{name}_{len(cols)}" in d:
        cols.append(d[f"{name}_{len(cols)}"])
    return np.stack(cols, axis=1) if cols else None


def comp(d, name, keys="xyz"):
    """Components of a Vector3d (xyz), Quaterniond (wxyz) or ForceVecd entry as [T, k], or None."""
    if not all(f"{name}_{k}" in d for k in keys):
        return None
    return np.stack([d[f"{name}_{k}"] for k in keys], axis=1)


def yaw_deg(q):
    w, x, y, z = q.T
    return np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def reorder(arr, order, names):
    if arr is None or arr.shape[1] != len(order):
        return None
    return arr[:, [order.index(n) for n in names]]


# ---------------------------------------------------------------------------------------------------------- the run


class Run:
    def __init__(self, d, dt, args, out):
        self.d, self.dt, self.out, self.args = d, dt, out, args
        self.t = d["t"]
        self.state = d.get("Tracking_state")
        track = np.flatnonzero(self.state == 3) if self.state is not None else np.array([], int)
        self.span = (self.t[track[0]], self.t[track[-1]]) if track.size else None
        self.during = "while tracking" if self.span else "in the plotted window"
        if args.window:
            t0, t1 = args.window
        elif self.span:
            t0, t1 = self.span[0] - 2.0, self.span[1] + 2.0
        else:
            t0, t1 = self.t[0], self.t[-1]
        self.sel = (self.t >= t0) & (self.t <= t1)
        self.tw = self.t[self.sel]
        self.track = self.sel & (self.state == 3) if self.span else self.sel.copy()
        if not self.track.any():
            self.track = self.sel.copy()
        self.events = []  # (time, reason) of each disarm
        armed = d.get("NewRLQPController_armed")
        if armed is not None:
            reason = d.get("NewRLQPController_disarmReason")
            for i in np.flatnonzero((armed[:-1] > 0.5) & (armed[1:] < 0.5)) + 1:
                r = int(reason[i]) if reason is not None else 0
                self.events.append((self.t[i], DISARM[r] if 0 <= r < len(DISARM) else f"reason {r}"))
        self.summary = []
        # The reference and the policy target hold placeholders until the motion is aligned (after the settle)
        self.before = np.isin(self.state, (0, 1, 2)) if self.state is not None else np.zeros(len(self.t), bool)

    def masked(self, arr):
        """A copy of a [T, ...] array with NaN before the tracking starts."""
        if arr is None:
            return None
        out = np.array(arr, dtype=float)
        out[self.before] = np.nan
        return out

    def marks(self, ax, label=False):
        if self.span:
            for s in self.span:
                if self.tw[0] <= s <= self.tw[-1]:
                    ax.axvline(s, color=INK2, ls=":", lw=0.8)
        for t, why in self.events:
            if self.tw[0] <= t <= self.tw[-1]:
                ax.axvline(t, color=RED, lw=1.0)
                if label:
                    ax.annotate(
                        why,
                        (t, 1.0),
                        xycoords=("data", "axes fraction"),
                        xytext=(3, -10),
                        textcoords="offset points",
                        color=RED,
                        fontsize=8,
                    )

    def save(self, fig, name):
        fig.suptitle(TITLES[name], x=0.01, ha="left", fontsize=12, fontweight="bold")
        path = os.path.join(self.out, name + ".png")
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print("  " + path)

    def note(self, line):
        self.summary.append(line)


def need(*arrays):
    return all(a is not None for a in arrays)


# ---------------------------------------------------------------------------------------------------------- plots


def plot_overview(run):
    d, s = run.d, run.sel
    rows = []
    if run.state is not None:
        rows.append("state")
    if "Tracking_motionTime" in d:
        rows.append("motion")
    if "NewRLQPController_torsoTilt" in d:
        rows.append("tilt")
    ref, gt, est = (comp(d, f"Tracking_{k}AnchorPos") for k in ("ref", "gt", "est"))
    ref = run.masked(ref)
    if ref is not None:
        rows.append("height")
    if not rows:
        return False
    fig, axes = plt.subplots(len(rows), 1, figsize=(11, 1.9 * len(rows) + 0.6), sharex=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    for ax, row in zip(axes, rows):
        if row == "state":
            rank = {v: i for i, v in enumerate(STATES)}
            ax.step(run.tw, [rank.get(int(v), np.nan) for v in run.state[s]], where="post", color=BLUE)
            ax.set_yticks(range(len(STATES)))
            ax.set_yticklabels(STATES.values())
            ax.set_title("Tracking state")
        elif row == "motion":
            ax.plot(run.tw, d["Tracking_motionTime"][s], color=BLUE)
            ax.set_ylabel("[s]")
            ax.set_title("Motion time (the reference frame being tracked)")
        elif row == "tilt":
            tilt = np.degrees(d["NewRLQPController_torsoTilt"])
            ax.plot(run.tw, tilt[s], color=BLUE)
            ax.set_ylabel("[deg]")
            ax.set_title("Torso tilt from vertical (the fall guard's signal)")
            run.note(
                f"torso tilt {run.during}: median {np.median(tilt[run.track]):.1f} deg, "
                f"largest {np.max(tilt[run.track]):.1f} deg"
            )
        elif row == "height":
            ax.plot(run.tw, ref[s, 2], color=INK2, ls="--", label="reference")
            if gt is not None:
                ax.plot(run.tw, gt[s, 2], color=BLUE, label="ground truth")
            if est is not None:
                ax.plot(run.tw, est[s, 2], color=ORANGE, label="estimate")
            ax.set_ylabel("[m]")
            ax.set_title("Anchor (torso) height")
            ax.legend(loc="lower right", ncol=3)
        run.marks(ax, label=ax is axes[0])
    axes[-1].set_xlabel("time [s]")
    run.save(fig, "overview")
    return True


def plot_base(run):
    d, s, k = run.d, run.sel, run.track
    if run.span is None:
        return "no tracking in this log"
    ref = run.masked(comp(d, "Tracking_refAnchorPos"))
    if ref is None:
        return False
    sources = [("reference", "ref", INK2, "--"), ("ground truth", "gt", BLUE, "-"), ("estimate", "est", ORANGE, "-")]
    fig = plt.figure(figsize=(12, 6), constrained_layout=True)
    gs = fig.add_gridspec(2, 2, width_ratios=(1, 1.5))
    ax_xy, ax_err = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1])
    ax_yaw = fig.add_subplot(gs[1, 1], sharex=ax_err)
    for label, key, color, ls in sources:
        p = comp(d, f"Tracking_{key}AnchorPos")
        q = comp(d, f"Tracking_{key}AnchorQuat", "wxyz")
        if p is None:
            continue
        if key == "ref":
            p, q = ref, run.masked(q)
        ax_xy.plot(p[k, 0], p[k, 1], color=color, ls=ls, label=label)
        ax_xy.plot(p[k, 0][0], p[k, 1][0], "o", color=color, ms=5)
        if key != "ref":
            err = np.linalg.norm(p[:, :2] - ref[:, :2], axis=1)
            ax_err.plot(run.tw, err[s], color=color, label=label)
            run.note(
                f"anchor distance to the reference, {label}: {err[k][-1]:.3f} m at the end of tracking, "
                f"largest {err[k].max():.3f} m"
            )
        if q is not None:
            ok = ~np.isnan(q[:, 0])
            yaw = np.full(len(q), np.nan)
            yaw[ok] = yaw_deg(q[ok])
            ax_yaw.plot(run.tw, yaw[s], color=color, ls=ls, label=label)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Anchor path while tracking, seen from above (dots: start)")
    ax_xy.legend(loc="best")
    ax_err.set_ylabel("[m]")
    ax_err.set_title("Horizontal distance from the reference anchor")
    ax_err.legend(loc="upper left")
    ax_yaw.set_ylabel("[deg]")
    ax_yaw.set_xlabel("time [s]")
    ax_yaw.set_title("Anchor yaw")
    for ax in (ax_err, ax_yaw):
        run.marks(ax, label=ax is ax_err)
    run.save(fig, "base")
    return True


def joints(run, names):
    d = run.d
    return (
        run.masked(reorder(vec(d, "Tracking_target"), POLICY_JOINTS, names)),
        reorder(vec(d, "qOut"), ROBOT_JOINTS, names),
        reorder(vec(d, "qIn"), ROBOT_JOINTS, names),
    )


def joint_grid(run, layout, name, torque=False):
    """One panel per joint: the angles (encoder, command, policy target) or the torque, over time."""
    names = [n for col in layout for n in col]
    if torque:
        tau = reorder(vec(run.d, "tauIn"), ROBOT_JOINTS, names)
        if tau is None:
            return False
    else:
        target, cmd, enc = joints(run, names)
        if enc is None:
            return False
    s, rows = run.sel, max(len(col) for col in layout)
    fig, axes = plt.subplots(
        rows,
        len(layout),
        figsize=(4.3 * len(layout), 1.45 * rows + 0.9),
        sharex=True,
        squeeze=False,
        constrained_layout=True,
    )
    for c, col in enumerate(layout):
        for r in range(rows):
            ax = axes[r][c]
            if r >= len(col):
                ax.axis("off")
                continue
            j = names.index(col[r])
            if torque:
                ax.plot(run.tw, tau[s, j], color=BLUE, lw=1.0)
            else:
                ax.plot(
                    run.tw, np.degrees(enc[s, j]), color=ORANGE, lw=2.4, alpha=0.75, label="encoder (qIn, measured)"
                )
                if cmd is not None:
                    ax.plot(
                        run.tw, np.degrees(cmd[s, j]), color=BLUE, lw=1.0, label="command (qOut, sent to the servos)"
                    )
                if target is not None:
                    ax.plot(run.tw, np.degrees(target[s, j]), color=INK, ls="--", lw=0.9, label="policy target")
            ax.set_title(col[r])
            run.marks(ax)
        axes[min(len(col), rows) - 1][c].tick_params(labelbottom=True)
    if not torque:
        handles, labels = axes[0][0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper right", ncol=3)
    fig.supxlabel("time [s]", fontsize=9, color=INK2)
    run.save(fig, name)
    return True


def plot_legs(run):
    return joint_grid(run, LEG_LAYOUT, "legs")


def plot_upper_body(run):
    return joint_grid(run, UPPER_LAYOUT, "upper_body")


def plot_legs_torque(run):
    return joint_grid(run, LEG_LAYOUT, "legs_torque", torque=True)


def plot_upper_body_torque(run):
    return joint_grid(run, UPPER_LAYOUT, "upper_body_torque", torque=True)


def bars(ax, values, labels, ylabel, title):
    x = np.arange(len(values))
    ax.bar(x, values, width=0.6, color=BLUE)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="x", visible=False)
    for i in np.argsort(values)[-3:]:  # label the three largest
        ax.annotate(
            f"{values[i]:.1f}",
            (x[i], values[i]),
            xytext=(0, 2),
            textcoords="offset points",
            ha="center",
            fontsize=7,
            color=INK2,
        )


def plot_joint_error(run):
    if run.span is None:
        return "no tracking in this log"
    target, _, enc = joints(run, POLICY_JOINTS)
    if not need(target, enc):
        return False
    e = np.degrees(target[run.track] - enc[run.track])
    rms, peak = np.sqrt(np.nanmean(e**2, axis=0)), np.nanmax(np.abs(e), axis=0)
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), constrained_layout=True)
    bars(axes[0], rms, POLICY_JOINTS, "[deg]", "Policy target minus encoder while tracking: RMS")
    bars(axes[1], peak, POLICY_JOINTS, "[deg]", "Largest")
    axes[1].set_xlabel(JOINT_KEY, fontsize=7)
    worst = ", ".join(f"{POLICY_JOINTS[i]} {rms[i]:.1f}" for i in np.argsort(rms)[::-1][:3])
    run.note(f"policy target minus encoder, RMS while tracking: worst {worst} deg")
    run.save(fig, "joint_error")
    return True


def contacts(fz, flag):
    """Contact intervals [start, end) as index pairs: the controller's flags if logged, else its rule on Fz."""
    if flag is not None:
        on = flag > 0.5
    else:
        on = np.zeros(len(fz), bool)
        state = False
        for i, f in enumerate(fz):
            state = f > CONTACT_ON if not state else f >= CONTACT_OFF
            on[i] = state
    edges = np.diff(on.astype(int), prepend=0, append=0)
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def plot_feet(run):
    d = run.d
    L, R = d.get("LeftFootForceSensor_fz"), d.get("RightFootForceSensor_fz")
    if not need(L, R):
        return False
    total = L + R
    stand = (run.t >= run.span[0] - 1.0) & (run.t < run.span[0]) if run.span else run.t < run.t[0] + 1.0
    bw = float(np.median(total[stand])) if stand.any() else float(np.median(total[run.sel]))
    peaks = {}
    for side, fz, flag in (
        ("left", L, d.get("NewRLQPController_leftFootContact")),
        ("right", R, d.get("NewRLQPController_rightFootContact")),
    ):
        # Landings while tracking: contacts that start then (a stance from before the tracking is not one)
        p = [fz[a:b].max() / bw for a, b in contacts(fz, flag) if run.track[a] and b - a > 1]
        peaks[side] = np.array(p)
    if run.args.zoom:
        z0, z1 = run.args.zoom
    elif run.span:
        mid = 0.5 * (run.span[0] + run.span[1])
        z0, z1 = mid - 2.0, mid + 2.0
    else:
        z0, z1 = run.tw[0], min(run.tw[0] + 4.0, run.tw[-1])
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), constrained_layout=True)
    for ax, sel, title in (
        (axes[0], run.sel, "Vertical foot force (the foot F/T sensors)"),
        (axes[1], (run.t >= z0) & (run.t <= z1), f"Close-up, {z0:.1f} to {z1:.1f} s"),
    ):
        ax.axhline(1.0, color=INK2, ls="--", lw=0.8)
        ax.plot(run.t[sel], L[sel] / bw, color=BLUE, label="left foot")
        ax.plot(run.t[sel], R[sel] / bw, color=ORANGE, label="right foot")
        ax.set_ylabel("[body weights]")
        ax.set_title(title)
        ax.legend(loc="upper right", ncol=2)
    run.marks(axes[0], label=True)
    axes[1].set_xlabel("time [s]")
    txt = (
        "; ".join(
            f"{k} foot {len(v)} landings, peak median {np.median(v):.2f} BW, largest {v.max():.2f} BW"
            for k, v in peaks.items()
            if len(v)
        )
        or f"no landings {run.during}"
    )
    axes[0].annotate(
        f"1 body weight = both feet's force while standing before the tracking. {txt}",
        (0, -0.16),
        xycoords="axes fraction",
        fontsize=8,
        color=INK2,
    )
    run.note(f"foot force {run.during} (body weight {bw:.0f} N): {txt}")
    run.save(fig, "feet")
    return True


def plot_torques(run):
    d = run.d
    tau = reorder(vec(d, "tauIn"), ROBOT_JOINTS, POLICY_JOINTS)
    if tau is None:
        return False
    a = np.abs(tau[run.track])
    ratio = d.get("NewRLQPController_rawTorqueRatioMax")
    if ratio is not None and not np.any(ratio[run.sel]):
        ratio = None  # not computed by this controller mode
    fig, axes = plt.subplots(
        3 if ratio is not None else 2, 1, figsize=(11, 9 if ratio is not None else 6), constrained_layout=True
    )
    bars(axes[0], a.max(axis=0), POLICY_JOINTS, "[N m]", f"Largest |torque| {run.during} (the worst moment)")
    bars(
        axes[1],
        np.percentile(a, 95, axis=0),
        POLICY_JOINTS,
        "[N m]",
        "95th percentile of |torque|: the joint stays below this 95 % of the time (its sustained load)",
    )
    axes[1].set_xlabel(JOINT_KEY, fontsize=7)
    if ratio is not None:
        axes[2].plot(run.tw, ratio[run.sel], color=BLUE)
        axes[2].set_ylabel("[fraction of the limit]")
        axes[2].set_xlabel("time [s]")
        axes[2].set_title("The controller's largest joint torque request over its limit (rawTorqueRatioMax)")
        run.marks(axes[2], label=True)
        run.note(f"largest torque request over its limit {run.during}: {ratio[run.track].max():.2f}")
    run.save(fig, "torques")
    return True


def plot_imu(run):
    d = run.d
    acc, gyr = comp(d, "Accelerometer_linearAcceleration"), comp(d, "Accelerometer_angularVelocity")
    if acc is None and gyr is None:
        return False
    panels = [
        (v, title, unit)
        for v, title, unit in ((acc, "IMU linear acceleration", "[m/s²]"), (gyr, "IMU angular velocity", "[rad/s]"))
        if v is not None
    ]
    fig, axes = plt.subplots(
        len(panels), 1, figsize=(11, 2.8 * len(panels) + 0.4), sharex=True, constrained_layout=True
    )
    axes = np.atleast_1d(axes)
    for ax, (v, title, unit) in zip(axes, panels):
        for i, (axis, color) in enumerate(zip("xyz", (BLUE, ORANGE, AQUA))):
            ax.plot(run.tw, v[run.sel, i], color=color, lw=0.9, label=axis)
        ax.set_ylabel(unit)
        ax.set_title(title)
        ax.legend(loc="upper right", ncol=3)
        run.marks(ax, label=ax is axes[0])
    axes[-1].set_xlabel("time [s]")
    run.save(fig, "imu")
    return True


def plot_timing(run):
    d = run.d
    g = d.get("perf_GlobalRun")
    if g is None:
        return False
    tick = 1000.0 * (run.dt or 0.005)
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), constrained_layout=True)
    for key, label, color in (
        ("perf_GlobalRun", "whole tick (GlobalRun)", BLUE),
        ("perf_ControllerRun", "controller (ControllerRun)", ORANGE),
        ("perf_SolverBuildAndSolve", "QP build and solve", AQUA),
    ):
        if key in d:
            axes[0].plot(run.tw, d[key][run.sel], color=color, lw=0.8, label=label)
    axes[0].axhline(tick, color=INK2, ls="--", lw=0.8)
    axes[0].annotate(
        f"tick {tick:.0f} ms",
        (0, tick),
        xycoords=("axes fraction", "data"),
        xytext=(3, 3),
        textcoords="offset points",
        fontsize=8,
        color=INK2,
    )
    axes[0].set_ylabel("[ms]")
    axes[0].set_xlabel("time [s]")
    axes[0].set_title("Controller time per tick")
    axes[0].legend(loc="upper right", ncol=3)
    run.marks(axes[0], label=True)
    gw = g[run.sel]
    axes[1].hist(gw, bins=80, color=BLUE)
    axes[1].axvline(tick, color=INK2, ls="--", lw=0.8)
    axes[1].set_xlabel("whole tick [ms]")
    axes[1].set_ylabel("ticks")
    over = 100.0 * np.mean(gw > tick)
    axes[1].set_title(
        f"Median {np.median(gw):.2f} ms, 99th percentile {np.percentile(gw, 99):.2f} ms, "
        f"{over:.2f} % of the ticks over the tick"
    )
    run.note(
        f"controller time per tick: median {np.median(gw):.2f} ms, p99 {np.percentile(gw, 99):.2f} ms, "
        f"{over:.2f} % over {tick:.0f} ms"
    )
    run.save(fig, "timing")
    return True


def plot_action(run):
    a = vec(run.d, "Tracking_action")
    if a is None or a.shape[1] != len(POLICY_JOINTS):
        return False
    aw, tw = a[run.sel], run.tw
    lim = float(np.percentile(np.abs(aw), 99)) or 1.0
    fig, axes = plt.subplots(
        2, 1, figsize=(11, 8), sharex=True, gridspec_kw={"height_ratios": (2.2, 1)}, constrained_layout=True
    )
    im = axes[0].imshow(
        aw.T,
        aspect="auto",
        cmap="RdBu_r",
        vmin=-lim,
        vmax=lim,
        interpolation="nearest",
        extent=(tw[0], tw[-1], len(POLICY_JOINTS) - 0.5, -0.5),
    )
    axes[0].set_yticks(range(len(POLICY_JOINTS)))
    axes[0].set_yticklabels(POLICY_JOINTS, fontsize=6)
    axes[0].grid(False)
    axes[0].set_title("Policy action per joint (target = default pose + action scale × action)")
    with plt.rc_context({"axes.grid": False}):
        fig.colorbar(im, ax=axes[0], label="action", pad=0.01)
    change = np.any(np.diff(a, axis=0) != 0, axis=1)
    idx = np.flatnonzero(change) + 1
    idx = idx[run.sel[idx]]
    rate = np.linalg.norm(a[idx] - a[idx - 1], axis=1)
    axes[1].plot(run.t[idx], rate, color=BLUE, lw=0.9)
    axes[1].set_ylabel("|Δ action|")
    axes[1].set_xlabel("time [s]")
    axes[1].set_title("Change of the action per policy step (all joints)")
    run.marks(axes[1])
    run.save(fig, "action")
    return True


PLOTS = {
    "overview": plot_overview,
    "base": plot_base,
    "legs": plot_legs,
    "upper_body": plot_upper_body,
    "legs_torque": plot_legs_torque,
    "upper_body_torque": plot_upper_body_torque,
    "joint_error": plot_joint_error,
    "feet": plot_feet,
    "torques": plot_torques,
    "imu": plot_imu,
    "timing": plot_timing,
    "action": plot_action,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("log", nargs="?", help=f"mc_rtc .bin log (default: {LATEST} in {' or '.join(LOG_DIRS)})")
    parser.add_argument("--out", help="output folder (default: plots/<log name> beside the logs folder)")
    parser.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"), help="time window [s]")
    parser.add_argument("--zoom", nargs=2, type=float, metavar=("T0", "T1"), help="feet close-up window [s]")
    parser.add_argument("--only", help="comma-separated plots: " + ",".join(PLOTS))
    parser.add_argument("--list", action="store_true", help="print the log's entries and exit")
    args = parser.parse_args()
    if shutil.which("mc_bin_utils") is None:
        sys.exit("mc_bin_utils not found: open a new terminal in the container (it loads ~/workspace/install)")
    if args.log is None:
        found = [os.path.join(d, LATEST) for d in LOG_DIRS if os.path.exists(os.path.join(d, LATEST))]
        if not found:
            sys.exit(f"no {LATEST} in {' or '.join(LOG_DIRS)}: give the log's path")
        args.log = found[0]
    log = os.path.realpath(os.path.expanduser(args.log))
    if args.list:
        print("\n".join(entries(log)[0]))
        return
    only = args.only.split(",") if args.only else list(PLOTS)
    unknown = [p for p in only if p not in PLOTS]
    if unknown:
        sys.exit(f"unknown plots {unknown}; choose from {list(PLOTS)}")
    folder = os.path.dirname(log)
    if os.path.basename(folder) == "logs":  # e.g. runs/logs/x.bin -> runs/plots/x/
        folder = os.path.dirname(folder)
    out = args.out or os.path.join(folder, "plots", os.path.splitext(os.path.basename(log))[0])
    os.makedirs(out, exist_ok=True)
    print(f"reading {log}")
    d, dt = load(log)
    style()
    run = Run(d, dt, args, out)
    motion = d.get("Tracking_motion")
    first = int(np.flatnonzero(run.track)[0])
    run.note(f"log {os.path.basename(log)}, {run.t[-1]:.1f} s, tick {1000 * (dt or 0):.0f} ms")
    if motion is not None:
        policy = d.get("Tracking_policyIndex")
        run.note(f"motion {motion[first]}" + (f", policy index {int(policy[first])}" if policy is not None else ""))
    run.note(f"tracking {run.span[0]:.2f} to {run.span[1]:.2f} s" if run.span else "no tracking in this log")
    for t, why in run.events:
        run.note(f"disarm at {t:.2f} s: {why}")
    for name in only:
        done = PLOTS[name](run)
        if done is not True:
            print(f"  {name}: skipped ({done or 'entries not in this log'})")
    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write("\n".join(run.summary) + "\n")
    print("\n".join(run.summary))


if __name__ == "__main__":
    main()
