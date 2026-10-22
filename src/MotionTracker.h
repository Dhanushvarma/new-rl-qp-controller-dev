#pragma once

#include "DisarmReason.h"
#include "TrackingMotion.h"
#include "TrackingObservation.h"

#include <Eigen/Core>

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

struct NewRLQPController;

/** Whole-body motion tracking (mjlab BeyondMimic-style policies) for the policy slots with a `tracking` section.
 *
 * A tracking slot has `obs_format: 9`, `velocity_action: false` and runs under the QP. What it does:
 *
 * - Motions: CSVs of mjlab's scripts/hrp5p/export_deploy.py (one row per policy step), listed from the motions
 *   directory (`tracking.motions_dir`, default the installed `share/NewRLQPController/motions`). `tracking.motion`
 *   names the one selected at load; the GUI selects another while disarmed. Motions load in a background thread
 *   and are checked against the policy (frame period, joints, observation size, the ONNX joint order).
 * - ARM: a min-jerk ramp from the held posture target to the motion's first-frame joint angles (`arm_ramp_speed`
 *   rad/s mean, at least 1 s), with the other actuated joints going to q0; then a hold of `arm_settle_time` s; then
 *   tracking from frame 0, the motion aligned to the robot anchor's horizontal position and heading
 *   (TrackingMotion::alignTo), with fresh filter and previous-action state.
 * - Tracking: every policy step (every policy_step_size / timeStep ticks) the observation of frame k
 *   (obs_format 9, buildTrackingObservation) goes to the policy; the command q0 + scale * action of the policy joints
 *   is low-pass filtered as mjlab's FilteredJointPositionAction does (target += alpha (command - target), alpha = 1 -
 *   exp(-2 pi cutoff policy_step_size), seeded with the measured joint angles at the first step), clamped to the
 *   joint range and written to q_rl, which the state hands to the PostureTask. The observation's previous action is
 *   the raw network output (zero at the first step). After the last frame the reference holds it (END, logged once).
 * - HOLD (any disarm: GUI, runaway or fall guard, a failed inference, or an error here): nothing writes the posture
 *   target any more, so it stays at the last command; the next ARM starts over from the ramp.
 *
 * Observation sources: the encoders for the joints, and for the floating base (anchor pose, base linear and angular
 * velocity) the source `tracking.base_source` selects, read in one place (observedBase()):
 * - estimate (default): the real robot's floating base as the observer pipeline sets it (MCWaiko) for the anchor pose
 *   and the base linear velocity, the IMU gyro (`tracking.imu_sensor`) for the angular velocity (MCWaiko's
 *   floating-base angular velocity being identically zero);
 * - ground_truth: the simulator's free joint (FloatingBase body sensor), simulation only.
 * Both are read and logged every tick whatever the source (Tracking_est*, Tracking_gt*). Another source (e.g. motion
 * capture) is a BaseSource value, a reader filling a BaseState and a case in observedBase().
 */
struct MotionTracker
{
  enum class State
  {
    Idle = 0,
    Ramping = 1,
    Settling = 2,
    Tracking = 3,
    Ended = 4
  };

  /** Where the observed floating-base state comes from (`tracking.base_source`) */
  enum class BaseSource
  {
    Estimate = 0,
    GroundTruth = 1
  };

  /** The floating-base state a tracking policy observes, from one source */
  struct BaseState
  {
    Eigen::Vector3d anchorPos = Eigen::Vector3d::Zero(); ///< Anchor position, world frame
    Eigen::Matrix3d anchorRot = Eigen::Matrix3d::Identity(); ///< Anchor orientation, anchor to world
    Eigen::Vector3d linVel = Eigen::Vector3d::Zero(); ///< Base origin velocity, base frame
    Eigen::Vector3d angVel = Eigen::Vector3d::Zero(); ///< Base angular velocity, base frame
  };

  explicit MotionTracker(NewRLQPController & ctl) : ctl_(ctl) {}
  ~MotionTracker();

  MotionTracker(const MotionTracker &) = delete;
  MotionTracker & operator=(const MotionTracker &) = delete;

  /** Read the current policy slot, after configRL(). Enables tracking if the slot has a `tracking` section (throws on
   * an invalid one) and starts loading the slot's motion. */
  void configure();

  /** Is the current policy slot a tracking slot? */
  bool enabled() const noexcept
  {
    return enabled_;
  }

  State state() const noexcept
  {
    return state_;
  }

  /** Why arming must be refused, empty if it can go (always empty for a slot that does not track) */
  std::string armBlocker() const;

  /** The arming edge: starts the ramp. False (and an error) if arming must be refused. */
  bool start();

  /** One armed controller tick: ramp, settle or track; writes q_rl. Disarms the controller on an error. */
  void run();

  /** The disarming edge: back to idle; the posture target stays where it is. */
  void stop();

  /** The tracking observation of the current frame (obs_format 9) */
  Eigen::VectorXd observation();

  /** Tilt from vertical [rad] of the anchor body in the observed base state (tracking.base_source, as the policy
   * observes it), as of the last update(); NaN if that source has no reading */
  double observedTilt() const;

  /** Every controller tick, before the FSM: takes finished motion loads, refreshes the GUI. */
  void update();

  /** Register the Tracking_* log entries */
  void addLog();

  /** Select a motion of the motions directory by file name (refused while armed); it loads in the background */
  void select(const std::string & name);

  static const char * stateName(State s);

  static const char * baseSourceName(BaseSource s);

private:
  struct LoadJob
  {
    uint64_t id = 0;
    std::string name;
    std::string path;
    std::unique_ptr<TrackingMotion> motion;
    std::string error;
    std::atomic<bool> done{false};
  };
  struct Loader
  {
    std::thread thread;
    std::shared_ptr<LoadJob> job;
  };

  /** Disarm the controller after an error, with the reason */
  void fail(const std::string & why, DisarmReason reason = DisarmReason::Tracking);
  /** The estimate source: the observer pipeline's floating base and the IMU gyro */
  void readEstimate(BaseState & out);
  /** The ground-truth source: the simulator's free joint; false if the robot has no FloatingBase body sensor */
  bool readGroundTruth(BaseState & out);
  /** Read every base source (for the logs, and the observed one) */
  void readBaseSources();
  /** The base state the observation uses, from tracking.base_source: the one place that picks the source */
  const BaseState & observedBase() const;
  /** Robot state for the observation (the observed base and the encoders), into in_ */
  void readRobot();
  void startTracking();
  void infer();
  /** Check a loaded motion against the current policy; empty if usable */
  std::string validate(const TrackingMotion & motion) const;
  void rescan();
  void buildGui();
  double motionTime() const;
  double duration() const;

  NewRLQPController & ctl_;
  bool enabled_ = false;

  // The slot's tracking section
  std::string anchorBody_;
  std::string baseBody_;
  std::string imuSensor_ = "Accelerometer";
  Eigen::Vector3d imuPos_ = Eigen::Vector3d::Zero();
  double cutoffHz_ = 0.0;
  double alpha_ = 1.0;
  double armRampSpeed_ = 0.1;
  double armSettleTime_ = 1.0;
  BaseSource baseSource_ = BaseSource::Estimate;
  double fallHeight_ = 0.5;
  std::string motionsDir_;
  std::vector<std::string> motionFiles_; ///< CSV file names in motionsDir_, sorted
  std::vector<std::string> onnxJointNames_; ///< The policy's joint_names metadata (motion joint order), if any

  // Policy joints, action order
  Eigen::Index nAct_ = 0;
  std::vector<int> dof_; ///< Index in the controller's jointNames (q_rl)
  std::vector<unsigned int> mbcIndex_; ///< Joint index in the robot
  Eigen::VectorXd q0_;
  Eigen::VectorXd scale_;
  Eigen::VectorXd lower_; ///< Joint range of the robot module
  Eigen::VectorXd upper_;
  unsigned long ticksPerStep_ = 1;
  double dt_ = 0.0; ///< Controller tick

  // Motion
  std::string selected_; ///< File name of the selected motion (ready, loading or failed)
  std::unique_ptr<TrackingMotion> motion_; ///< The selected motion once loaded and checked, null otherwise
  std::string motionStatus_ = "none selected";
  std::vector<size_t> motionIndex_; ///< Motion joint of each policy joint
  std::vector<Loader> loaders_;
  uint64_t requestId_ = 0;
  bool rescanRequested_ = false;
  bool guiDirty_ = true;

  // Session
  State state_ = State::Idle;
  unsigned long sessionTick_ = 0;
  unsigned long rampTicks_ = 0;
  unsigned long settleTicks_ = 0;
  Eigen::VectorXd rampStart_; ///< q_rl indexing
  Eigen::VectorXd rampTarget_; ///< q_rl indexing
  double t_ = -1.0; ///< Time from the tracking start (counts up to 0 while ramping and settling), -1 while idle
  size_t frame_ = 0; ///< Motion frame of the next inference
  size_t usedFrame_ = 0; ///< Motion frame of the last inference
  bool filterFresh_ = true;
  bool endReported_ = false;
  bool fallReported_ = false;
  Eigen::VectorXd command_; ///< q0 + scale * action
  Eigen::VectorXd target_; ///< Filtered command (the filter state)
  Eigen::VectorXd lastAction_; ///< Raw action of the last inference
  Eigen::VectorXd obs_;
  TrackingObservationInput in_;
  BaseState est_; ///< The estimate source, read every tick
  BaseState gt_; ///< The ground-truth source, read every tick when available
  bool gtAvailable_ = false;
  Eigen::Vector3d observerAngVel_ = Eigen::Vector3d::Zero(); ///< The observer pipeline's base angular velocity (logged)
};
