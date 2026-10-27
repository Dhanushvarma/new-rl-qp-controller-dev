#include "MotionTracker.h"

#include "NewRLQPController.h"

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/io_utils.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>

// Default motions directory, set by CMake to the installed one
#ifndef NEWRLQP_MOTIONS_DIR
#  define NEWRLQP_MOTIONS_DIR ""
#endif

namespace
{

const std::string LOG_PREFIX = "Tracking_";
const std::vector<std::string> GUI_CATEGORY = {"NewRLQPController", "Tracking"};
/** The observation terms of buildTrackingObservation, in order (the tracking section's `observations`) */
const std::vector<std::string> OBSERVATION_TERMS = {"command",      "motion_anchor_pos_b", "motion_anchor_ori_b",
                                                    "base_lin_vel", "base_ang_vel",        "joint_pos",
                                                    "joint_vel",    "actions"};

/** Position and orientation (body to world) of a body; sva stores the transposed rotation */
void bodyPose(const sva::PTransformd & X_0_b, Eigen::Vector3d & pos, Eigen::Matrix3d & rot)
{
  pos = X_0_b.translation();
  rot = X_0_b.rotation().transpose();
}

/** Heading (yaw) of a body to world rotation [rad] */
double heading(const Eigen::Matrix3d & rot)
{
  return std::atan2(rot(1, 0), rot(0, 0));
}

/** Per-joint MultiBodyConfig values (q, alpha, jointTorque) in reference joint order, NaN for joints that are not 1-dof
 * joints of the multibody (the fingers are fixed in the HRP5P module) */
Eigen::VectorXd toRefOrder(const mc_rbdyn::Robot & robot, const std::vector<std::vector<double>> & mbcValues)
{
  const auto & rjo = robot.refJointOrder();
  Eigen::VectorXd out =
      Eigen::VectorXd::Constant(static_cast<Eigen::Index>(rjo.size()), std::numeric_limits<double>::quiet_NaN());
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    if(!robot.hasJoint(rjo[i]))
    {
      continue;
    }
    const auto & v = mbcValues[robot.jointIndexByName(rjo[i])];
    if(v.size() == 1)
    {
      out(static_cast<Eigen::Index>(i)) = v[0];
    }
  }
  return out;
}

/** A list of names from an ONNX metadata entry (export_deploy.py writes lists as YAML), empty if it is not one */
std::vector<std::string> yamlList(const std::string & text)
{
  if(text.empty())
  {
    return {};
  }
  try
  {
    const std::vector<std::string> names = mc_rtc::Configuration::fromYAMLData(text);
    return names;
  }
  catch(const std::exception &)
  {
    return {};
  }
}

} // namespace

MotionTracker::~MotionTracker()
{
  for(auto & loader : loaders_)
  {
    if(loader.thread.joinable())
    {
      loader.thread.join();
    }
  }
}

const char * MotionTracker::baseSourceName(BaseSource s)
{
  switch(s)
  {
    case BaseSource::Estimate:
      return "estimate";
    case BaseSource::GroundTruth:
      return "ground_truth";
  }
  return "?";
}

const char * MotionTracker::stateName(State s)
{
  switch(s)
  {
    case State::Idle:
      return "idle";
    case State::Ramping:
      return "ramping";
    case State::Settling:
      return "settling";
    case State::Tracking:
      return "tracking";
    case State::Ended:
      return "ended";
    case State::HandingOver:
      return "handing over";
  }
  return "?";
}

void MotionTracker::configure()
{
  auto & ctl = ctl_;
  // A slot is only (re)loaded while disarmed: drop the previous slot's session and motion. A load still running for
  // it finishes in its thread and is ignored (requestId_).
  state_ = State::Idle;
  t_ = -1.0;
  motion_.reset();
  selected_.clear();
  motionStatus_ = "none selected";
  ++requestId_;
  guiDirty_ = true;

  const auto slot = ctl.policySlot();
  const bool hasSection = slot.has("tracking");
  if(hasSection != (ctl.obsFormat_ == 9))
  {
    mc_rtc::log::error_and_throw("[Tracking] policy {}: a tracking slot needs both obs_format 9 and a tracking section",
                                 ctl.currentPolicyIndex);
  }
  enabled_ = hasSection;
  if(!enabled_)
  {
    return;
  }
  if(ctl.velocityAction_)
  {
    mc_rtc::log::error_and_throw("[Tracking] policy {}: a tracking policy takes position actions (velocity_action: "
                                 "false)",
                                 ctl.currentPolicyIndex);
  }
  const auto tracking = slot("tracking");
  std::string kind = "whole_body";
  tracking("kind", kind);
  if(kind != "whole_body")
  {
    mc_rtc::log::error_and_throw("[Tracking] tracking kind {} is not supported (whole_body only)", kind);
  }
  if(tracking.has("observations"))
  {
    const std::vector<std::string> terms = tracking("observations");
    if(terms != OBSERVATION_TERMS)
    {
      mc_rtc::log::error_and_throw("[Tracking] observations [{}] are not obs_format 9's [{}]",
                                   mc_rtc::io::to_string(terms), mc_rtc::io::to_string(OBSERVATION_TERMS));
    }
  }
  anchorBody_ = static_cast<std::string>(tracking("anchor_body"));
  baseBody_ = static_cast<std::string>(tracking("base_body"));
  imuSensor_ = "Accelerometer";
  tracking("imu_sensor", imuSensor_);
  const std::vector<double> imu = tracking("imu_site_pos");
  if(imu.size() != 3)
  {
    mc_rtc::log::error_and_throw("[Tracking] imu_site_pos needs 3 values");
  }
  imuPos_ = Eigen::Vector3d(imu[0], imu[1], imu[2]);
  // Required: it is part of the plant the policy was trained on (0 turns the filter off)
  cutoffHz_ = tracking("action_filter_cutoff_hz");
  armRampSpeed_ = tracking("arm_ramp_speed", 0.1);
  armSettleTime_ = tracking("arm_settle_time", 1.0);
  armHandoverTime_ = tracking("arm_handover_time", 1.0);
  if(!(armRampSpeed_ > 0.0) || !(armSettleTime_ >= 0.0) || !(armHandoverTime_ >= 0.0))
  {
    mc_rtc::log::error_and_throw("[Tracking] arm_ramp_speed must be > 0, arm_settle_time and arm_handover_time >= 0");
  }
  if(tracking.has("use_ground_truth_base"))
  {
    mc_rtc::log::error_and_throw("[Tracking] use_ground_truth_base is replaced by base_source (estimate or "
                                 "ground_truth)");
  }
  const std::string baseSource = tracking("base_source", std::string("estimate"));
  if(baseSource == "estimate")
  {
    baseSource_ = BaseSource::Estimate;
  }
  else if(baseSource == "ground_truth")
  {
    baseSource_ = BaseSource::GroundTruth;
  }
  else
  {
    mc_rtc::log::error_and_throw("[Tracking] base_source {} is not estimate or ground_truth", baseSource);
  }
  fallHeight_ = tracking("fall_height", 0.5);
  motionsDir_ = tracking("motions_dir", std::string(NEWRLQP_MOTIONS_DIR));

  const auto & robot = ctl.robot();
  if(!robot.hasBody(anchorBody_) || !robot.hasBody(baseBody_))
  {
    mc_rtc::log::error_and_throw("[Tracking] the robot has no body {} or {}", anchorBody_, baseBody_);
  }
  if(!robot.hasBodySensor(imuSensor_))
  {
    mc_rtc::log::error_and_throw("[Tracking] the robot has no body sensor {} (imu_sensor)", imuSensor_);
  }
  const auto & imuSensor = robot.bodySensor(imuSensor_);
  if(imuSensor.parentBody() != baseBody_ || !imuSensor.X_b_s().rotation().isIdentity(1e-9))
  {
    mc_rtc::log::error_and_throw("[Tracking] {} is not mounted on {} with a pure translation: its gyro does not read "
                                 "the base angular velocity",
                                 imuSensor_, baseBody_);
  }
  if((imuSensor.X_b_s().translation() - imuPos_).norm() > 1e-3)
  {
    const Eigen::Vector3d mount = imuSensor.X_b_s().translation();
    mc_rtc::log::warning("[Tracking] imu_site_pos ({}, {}, {}) differs from the robot module's {} mount ({}, {}, {}); "
                         "the observation uses imu_site_pos, the policy's",
                         imuPos_.x(), imuPos_.y(), imuPos_.z(), imuSensor_, mount.x(), mount.y(), mount.z());
  }

  // One inference every ticksPerStep_ ticks, as many as the tick allows (10 at 2 ms, 4 at 5 ms for 20 ms)
  dt_ = ctl.timeStep;
  ticksPerStep_ = static_cast<unsigned long>(std::max(1L, std::lround(ctl.policyStepSize / dt_)));
  if(std::abs(static_cast<double>(ticksPerStep_) * dt_ - ctl.policyStepSize) > 1e-9)
  {
    mc_rtc::log::error_and_throw("[Tracking] policy_step_size {} s is not a multiple of the controller tick {} s",
                                 ctl.policyStepSize, dt_);
  }
  alpha_ = cutoffHz_ > 0.0 ? 1.0 - std::exp(-2.0 * M_PI * cutoffHz_ * ctl.policyStepSize) : 1.0;

  // Policy joints, in action order
  nAct_ = static_cast<Eigen::Index>(ctl.refJointOrderRLAction.size());
  dof_.assign(static_cast<size_t>(nAct_), -1);
  mbcIndex_.assign(static_cast<size_t>(nAct_), 0);
  q0_.resize(nAct_);
  scale_.resize(nAct_);
  lower_.resize(nAct_);
  upper_.resize(nAct_);
  for(Eigen::Index j = 0; j < nAct_; ++j)
  {
    const auto & name = ctl.refJointOrderRLAction[static_cast<size_t>(j)];
    const int i = ctl.actionToDofMap[static_cast<size_t>(j)];
    if(i < 0)
    {
      mc_rtc::log::error_and_throw("[Tracking] ref_joint_order joint {} is not an actuated joint of the robot", name);
    }
    dof_[static_cast<size_t>(j)] = i;
    q0_(j) = ctl.q_zero(i);
    scale_(j) = ctl.actionScale(i);
    if(scale_(j) == 0.0)
    {
      mc_rtc::log::error_and_throw("[Tracking] no action_scale for policy joint {}", name);
    }
    const auto index = robot.jointIndexByName(name);
    mbcIndex_[static_cast<size_t>(j)] = index;
    if(robot.ql()[index].size() != 1 || robot.qu()[index].size() != 1)
    {
      mc_rtc::log::error_and_throw("[Tracking] policy joint {} has no joint range", name);
    }
    lower_(j) = robot.ql()[index][0];
    upper_(j) = robot.qu()[index][0];
  }

  // What the policy was exported with, when its metadata says it (mjlab's export_deploy.py writes both)
  const auto actionOrder = yamlList(ctl.rlPolicy->customMetadata("action_joint_order"));
  if(!actionOrder.empty() && actionOrder != ctl.refJointOrderRLAction)
  {
    mc_rtc::log::error_and_throw("[Tracking] ref_joint_order differs from the policy's action_joint_order [{}]",
                                 mc_rtc::io::to_string(actionOrder));
  }
  onnxJointNames_ = yamlList(ctl.rlPolicy->customMetadata("joint_names"));
  if(actionOrder.empty() || onnxJointNames_.empty())
  {
    mc_rtc::log::warning("[Tracking] the policy has no action_joint_order / joint_names metadata: ref_joint_order and "
                         "the motions' joint order are not checked against it");
  }

  command_ = q0_;
  target_ = q0_;
  lastAction_ = Eigen::VectorXd::Zero(nAct_);
  obs_ = Eigen::VectorXd::Zero(ctl.rlPolicy->getObservationSize());
  in_.jointPos = Eigen::VectorXd::Zero(nAct_);
  in_.jointVel = Eigen::VectorXd::Zero(nAct_);
  frame_ = 0;
  usedFrame_ = 0;

  rescan();
  mc_rtc::log::info("[Tracking] policy {} tracks motions: {} CSVs in {}; filter {} Hz (alpha {:.4f}); one inference "
                    "every {} ticks; base_source {}; ARM ramps at {} rad/s, then {} s settle, {} s hand-over",
                    ctl.currentPolicyIndex, motionFiles_.size(), motionsDir_, cutoffHz_, alpha_, ticksPerStep_,
                    baseSourceName(baseSource_), armRampSpeed_, armSettleTime_, armHandoverTime_);
  std::string motion;
  tracking("motion", motion);
  if(!motion.empty())
  {
    select(motion);
  }
}

void MotionTracker::rescan()
{
  motionFiles_.clear();
  std::error_code ec;
  if(!std::filesystem::is_directory(motionsDir_, ec))
  {
    mc_rtc::log::warning("[Tracking] motions directory {} does not exist", motionsDir_);
    return;
  }
  // Every CSV in the directory and its subfolders, by its path relative to the directory (a motion set's folder,
  // e.g. walk/motion_walk_15s.csv), which select() and tracking.motion take
  std::filesystem::recursive_directory_iterator it(motionsDir_, ec);
  for(; !ec && it != std::filesystem::recursive_directory_iterator(); it.increment(ec))
  {
    std::error_code fileEc;
    if(it->is_regular_file(fileEc) && it->path().extension() == ".csv")
    {
      const std::filesystem::path relative = std::filesystem::relative(it->path(), motionsDir_, fileEc);
      if(!fileEc && !relative.empty()) { motionFiles_.push_back(relative.generic_string()); }
    }
  }
  std::sort(motionFiles_.begin(), motionFiles_.end());
}

void MotionTracker::select(const std::string & name)
{
  if(ctl_.policyArmed_)
  {
    mc_rtc::log::error("[Tracking] refusing to change the motion while ARMED -- HOLD first");
    return;
  }
  if(!enabled_)
  {
    mc_rtc::log::error("[Tracking] the current policy does not track motions");
    return;
  }
  selected_ = name;
  motion_.reset();
  motionStatus_ = "loading";
  auto job = std::make_shared<LoadJob>();
  job->id = ++requestId_;
  job->name = name;
  job->path = (std::filesystem::path(motionsDir_) / name).string();
  // Parsing a 60 s motion takes a few hundred ms: off the control loop
  try
  {
    Loader loader;
    loader.job = job;
    loader.thread = std::thread(
        [job]()
        {
          try
          {
            auto motion = std::make_unique<TrackingMotion>();
            motion->load(job->path);
            job->motion = std::move(motion);
          }
          catch(const std::exception & e)
          {
            job->error = e.what();
          }
          job->done = true;
        });
    loaders_.push_back(std::move(loader));
  }
  catch(const std::exception & e)
  {
    motionStatus_ = std::string("error: ") + e.what();
    mc_rtc::log::error("[Tracking] cannot load {}: {}", job->path, e.what());
    return;
  }
  mc_rtc::log::info("[Tracking] loading motion {}", job->path);
}

std::string MotionTracker::validate(const TrackingMotion & motion) const
{
  const auto & ctl = ctl_;
  if(motion.frames() == 0)
  {
    return "no frames";
  }
  if(motion.time.size() >= 2)
  {
    const double period = motion.time[1] - motion.time[0];
    if(std::abs(period - ctl.policyStepSize) > 1e-6)
    {
      return fmt::format("frame period {} s, the policy step is {} s", period, ctl.policyStepSize);
    }
    for(size_t k = 1; k + 1 < motion.time.size(); ++k)
    {
      if(std::abs(motion.time[k + 1] - motion.time[k] - period) > 1e-6)
      {
        return fmt::format("irregular frame times at frame {}", k);
      }
    }
  }
  for(const auto & name : ctl.refJointOrderRLAction)
  {
    if(std::find(motion.jointNames.begin(), motion.jointNames.end(), name) == motion.jointNames.end())
    {
      return fmt::format("no column for policy joint {}", name);
    }
  }
  const Eigen::Index nObs = 2 * static_cast<Eigen::Index>(motion.jointNames.size()) + 15 + 3 * nAct_;
  if(nObs != ctl.rlPolicy->getObservationSize())
  {
    return fmt::format("{} motion joints and {} policy joints give {} observations, the policy takes {}",
                       motion.jointNames.size(), nAct_, nObs, ctl.rlPolicy->getObservationSize());
  }
  if(!onnxJointNames_.empty() && motion.jointNames != onnxJointNames_)
  {
    return "its joint order differs from the policy's joint_names";
  }
  return "";
}

void MotionTracker::update()
{
  for(auto it = loaders_.begin(); it != loaders_.end();)
  {
    if(!it->job->done)
    {
      ++it;
      continue;
    }
    it->thread.join();
    const auto job = it->job;
    it = loaders_.erase(it);
    if(job->id != requestId_)
    {
      continue; // superseded by another selection or another policy
    }
    if(!job->error.empty())
    {
      motionStatus_ = "error: " + job->error;
      mc_rtc::log::error("[Tracking] motion {}: {}", job->name, job->error);
      continue;
    }
    const std::string why = validate(*job->motion);
    if(!why.empty())
    {
      motionStatus_ = "unusable: " + why;
      mc_rtc::log::error("[Tracking] motion {} is unusable with policy {}: {}", job->name, ctl_.currentPolicyIndex,
                         why);
      continue;
    }
    motion_ = std::move(job->motion);
    motionIndex_.clear();
    for(const auto & name : ctl_.refJointOrderRLAction)
    {
      motionIndex_.push_back(static_cast<size_t>(
          std::find(motion_->jointNames.begin(), motion_->jointNames.end(), name) - motion_->jointNames.begin()));
    }
    motionStatus_ = "ready";
    mc_rtc::log::success("[Tracking] motion {} ready: {} frames, {:.2f} s", job->name, motion_->frames(), duration());
  }
  if(rescanRequested_)
  {
    rescanRequested_ = false;
    rescan();
    guiDirty_ = true;
    mc_rtc::log::info("[Tracking] {} motions in {}", motionFiles_.size(), motionsDir_);
  }
  if(guiDirty_)
  {
    guiDirty_ = false;
    buildGui();
  }
  // Both base sources, for the logs, whatever the source and the arming state
  if(enabled_)
  {
    readBaseSources();
  }
}

std::string MotionTracker::armBlocker() const
{
  if(!enabled_)
  {
    return "";
  }
  if(!ctl_.rlPolicy || !ctl_.rlPolicy->isLoaded())
  {
    return "the policy is not loaded";
  }
  if(!ctl_.useQP())
  {
    return "motion tracking runs under the QP only (QP control is bypassed)";
  }
  if(!motion_)
  {
    return selected_.empty() ? "no motion selected" : fmt::format("motion {} is not ready ({})", selected_,
                                                                  motionStatus_);
  }
  if(baseSource_ == BaseSource::GroundTruth
     && (!ctl_.isSimulated() || !ctl_.robot().hasBodySensor("FloatingBase")))
  {
    return "base_source ground_truth is for simulation only";
  }
  return "";
}

bool MotionTracker::start()
{
  const std::string why = armBlocker();
  if(!enabled_ || !why.empty())
  {
    mc_rtc::log::error("[Tracking] cannot arm: {}", enabled_ ? why : "not a tracking policy");
    return false;
  }
  // From the posture target held while disarmed (NewRLQPController::run() keeps q_rl on it), so that arming does not
  // move the target: to the motion's first frame for the policy joints, to q0 for the other actuated joints (where
  // tracking holds them).
  rampStart_ = ctl_.q_rl;
  rampTarget_ = ctl_.q_zero;
  for(Eigen::Index j = 0; j < nAct_; ++j)
  {
    const double q = motion_->jointPos.front()(static_cast<Eigen::Index>(motionIndex_[static_cast<size_t>(j)]));
    rampTarget_(dof_[static_cast<size_t>(j)]) = std::clamp(q, lower_(j), upper_(j));
  }
  Eigen::Index worst = 0;
  const double maxOffset = (rampTarget_ - rampStart_).cwiseAbs().maxCoeff(&worst);
  const double rampTime = maxOffset < 1e-4 ? 0.0 : std::max(1.0, maxOffset / armRampSpeed_);
  rampTicks_ = static_cast<unsigned long>(std::lround(rampTime / dt_));
  settleTicks_ = static_cast<unsigned long>(std::lround(armSettleTime_ / dt_));
  // Whole policy steps, so that the inferences keep their cadence when the motion clock starts
  const double stepTime = dt_ * static_cast<double>(ticksPerStep_);
  handoverTicks_ = static_cast<unsigned long>(std::ceil(armHandoverTime_ / stepTime - 1e-9)) * ticksPerStep_;
  sessionTick_ = 0;
  state_ = rampTicks_ > 0 ? State::Ramping : State::Settling;
  t_ = -static_cast<double>(rampTicks_ + settleTicks_ + handoverTicks_) * dt_;
  frame_ = 0;
  usedFrame_ = 0;
  filterFresh_ = true;
  endReported_ = false;
  fallReported_ = false;
  lastAction_.setZero();
  mc_rtc::log::warning("[Tracking] ARMED on {}: {:.2f} s min-jerk ramp to its first frame (largest move {:.3f} rad, "
                       "{}), {:.2f} s settle, {:.2f} s hand-over, then {:.2f} s of tracking",
                       selected_, rampTime, maxOffset, ctl_.jointNames[static_cast<size_t>(worst)],
                       static_cast<double>(settleTicks_) * dt_, static_cast<double>(handoverTicks_) * dt_, duration());
  return true;
}

void MotionTracker::fail(const std::string & why, DisarmReason reason)
{
  ctl_.disarm(reason);
  mc_rtc::log::error("[Tracking] {}: policy DISARMED", why);
  stop();
}

void MotionTracker::stop()
{
  if(state_ == State::Idle)
  {
    return;
  }
  mc_rtc::log::warning("[Tracking] HOLD while {} (frame {}, motion time {:.2f} s): posture target frozen at the last "
                       "command",
                       stateName(state_), usedFrame_, motionTime());
  state_ = State::Idle;
  t_ = -1.0;
}

void MotionTracker::run()
{
  if(state_ == State::Idle)
  {
    return;
  }
  try
  {
    if(!ctl_.useQP())
    {
      fail("QP control was bypassed: motion tracking runs under the QP only");
      return;
    }
    readRobot();
    const unsigned long k = sessionTick_++;
    const unsigned long handoverStart = rampTicks_ + settleTicks_;
    const unsigned long trackStart = handoverStart + handoverTicks_;
    t_ = (static_cast<double>(k) - static_cast<double>(trackStart)) * dt_;
    if(k < rampTicks_)
    {
      // Min-jerk: zero velocity and acceleration at both ends
      double s = static_cast<double>(k + 1) / static_cast<double>(rampTicks_);
      s = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
      ctl_.q_rl = rampStart_ + s * (rampTarget_ - rampStart_);
      state_ = State::Ramping;
    }
    else if(k < handoverStart)
    {
      ctl_.q_rl = rampTarget_;
      state_ = State::Settling;
    }
    else if(k < trackStart)
    {
      const unsigned long h = k - handoverStart;
      if(h == 0)
      {
        startHandover();
      }
      if(h % ticksPerStep_ == 0)
      {
        infer();
      }
      // infer() disarms on an error (state Idle): leave the posture target where it is
      if(state_ == State::HandingOver)
      {
        // Min-jerk from the first frame to the policy's filtered target, every tick; at the last tick the target
        double s = static_cast<double>(h + 1) / static_cast<double>(handoverTicks_);
        s = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
        for(Eigen::Index j = 0; j < nAct_; ++j)
        {
          const int i = dof_[static_cast<size_t>(j)];
          const double q = rampTarget_(i) + s * (target_(j) - rampTarget_(i));
          ctl_.q_rl(i) = std::clamp(q, lower_(j), upper_(j));
        }
      }
    }
    else
    {
      const unsigned long tick = k - trackStart;
      if(tick == 0)
      {
        startTracking();
      }
      if(tick % ticksPerStep_ == 0)
      {
        infer();
      }
    }
    // Simulation only: the simulator's base height
    if(!fallReported_ && ctl_.isSimulated() && ctl_.robot().hasBodySensor("FloatingBase"))
    {
      const double z = ctl_.robot().bodySensor("FloatingBase").position().z();
      if(z < fallHeight_)
      {
        fallReported_ = true;
        mc_rtc::log::warning("[Tracking] FALL: ground-truth base z = {:.3f} m below {} m at t = {:.3f} s", z,
                             fallHeight_, t_);
      }
    }
  }
  catch(const std::exception & e)
  {
    fail(e.what());
  }
}

void MotionTracker::readEstimate(BaseState & out)
{
  const auto & real = ctl_.realRobot(ctl_.robot().name());
  // The real robot's floating base as the observer pipeline sets it, and its kinematics (encoders); the angular
  // velocity from the gyro, MCWaiko's being identically zero (see utils.cpp, obs_format 0)
  const sva::MotionVecd & velB = real.mbc().bodyVelB[real.bodyIndexByName(baseBody_)];
  out.linVel = velB.linear();
  observerAngVel_ = velB.angular();
  out.angVel = real.bodySensor(imuSensor_).angularVelocity();
  bodyPose(real.bodyPosW(anchorBody_), out.anchorPos, out.anchorRot);
}

bool MotionTracker::readGroundTruth(BaseState & out)
{
  const auto & control = ctl_.robot();
  if(!control.hasBodySensor("FloatingBase"))
  {
    return false;
  }
  // The simulator's free joint for the base, the real robot's kinematics from the base to the anchor
  const auto & real = ctl_.realRobot(control.name());
  const auto & bs = control.bodySensor("FloatingBase");
  const Eigen::Matrix3d rot = bs.orientation().inverse().toRotationMatrix();
  const sva::PTransformd X_0_base(Eigen::Matrix3d(rot.transpose()), bs.position());
  const sva::PTransformd X_base_anchor = real.bodyPosW(anchorBody_) * real.bodyPosW(baseBody_).inv();
  bodyPose(X_base_anchor * X_0_base, out.anchorPos, out.anchorRot);
  out.linVel = rot.transpose() * bs.linearVelocity();
  out.angVel = bs.angularVelocity();
  return true;
}

void MotionTracker::readBaseSources()
{
  readEstimate(est_);
  gtAvailable_ = readGroundTruth(gt_);
}

const MotionTracker::BaseState & MotionTracker::observedBase() const
{
  switch(baseSource_)
  {
    case BaseSource::Estimate:
      return est_;
    case BaseSource::GroundTruth:
      if(!gtAvailable_)
      {
        throw std::runtime_error("base_source ground_truth needs the FloatingBase body sensor");
      }
      return gt_;
  }
  throw std::runtime_error("unknown base_source");
}

double MotionTracker::observedTilt() const
{
  // The angle between the anchor's z axis and the world's: anchorRot is anchor to world, so its (2, 2) is their dot
  try
  {
    return std::acos(std::clamp(observedBase().anchorRot(2, 2), -1.0, 1.0));
  }
  catch(const std::exception &)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

void MotionTracker::readRobot()
{
  readBaseSources();
  const BaseState & base = observedBase();
  in_.baseLinVel = base.linVel;
  in_.baseAngVel = base.angVel;
  in_.anchorPos = base.anchorPos;
  in_.anchorRot = base.anchorRot;

  const auto & real = ctl_.realRobot(ctl_.robot().name());
  for(Eigen::Index j = 0; j < nAct_; ++j)
  {
    const auto index = mbcIndex_[static_cast<size_t>(j)];
    in_.jointPos(j) = real.mbc().q[index][0];
    in_.jointVel(j) = real.mbc().alpha[index][0];
  }
}

void MotionTracker::startHandover()
{
  // The motion's first anchor gets the robot anchor's horizontal position and heading (readRobot() ran this tick)
  motion_->alignTo(in_.anchorPos, in_.anchorRot);
  frame_ = 0;
  usedFrame_ = 0;
  filterFresh_ = true;
  endReported_ = false;
  lastAction_.setZero();
  state_ = State::HandingOver;
  mc_rtc::log::info("[Tracking] handing over to the policy on {}'s first frame: {:.2f} s blend from the first frame "
                    "to its target",
                    selected_, static_cast<double>(handoverTicks_) * dt_);
}

void MotionTracker::startTracking()
{
  // After a hand-over the policy already runs on the aligned motion: keep its filter and previous action
  if(handoverTicks_ == 0)
  {
    // The motion's first anchor gets the robot anchor's horizontal position and heading (readRobot() ran this tick)
    motion_->alignTo(in_.anchorPos, in_.anchorRot);
    filterFresh_ = true;
    lastAction_.setZero();
  }
  frame_ = 0;
  usedFrame_ = 0;
  endReported_ = false;
  state_ = State::Tracking;
  mc_rtc::log::success("[Tracking] tracking {} from frame 0: robot anchor at ({:.3f}, {:.3f}, {:.3f}) heading {:.1f} "
                       "deg ({}), one inference every {} ticks",
                       selected_, in_.anchorPos.x(), in_.anchorPos.y(), in_.anchorPos.z(),
                       heading(in_.anchorRot) * 180.0 / M_PI, baseSourceName(baseSource_), ticksPerStep_);
}

Eigen::VectorXd MotionTracker::observation()
{
  if(!enabled_ || !motion_)
  {
    throw std::runtime_error("[Tracking] obs_format 9 needs a tracking policy with a loaded motion");
  }
  readRobot();
  in_.refJointPos = motion_->jointPos[usedFrame_];
  in_.refJointVel = motion_->jointVel[usedFrame_];
  in_.refAnchorPos = motion_->anchorPos(usedFrame_);
  in_.refAnchorRot = motion_->anchorRot(usedFrame_);
  in_.q0 = q0_;
  in_.lastAction = lastAction_;
  return buildTrackingObservation(in_, imuPos_);
}

void MotionTracker::infer()
{
  usedFrame_ = std::min(frame_, motion_->frames() - 1);
  // obs_format 9 is observation(), which reads the robot (in_) for this step
  const Eigen::VectorXd obs = ctl_.utilsClass.getCurrentObservation(ctl_);
  if(!obs.allFinite())
  {
    fail("non-finite observation");
    return;
  }
  // A failed inference throws (no substitute action): disarm, the posture target stays at the last command
  Eigen::VectorXd action;
  try
  {
    action = ctl_.rlPolicy->predict(obs);
  }
  catch(const std::exception & e)
  {
    fail(fmt::format("policy inference failed ({})", e.what()), DisarmReason::Inference);
    return;
  }
  if(action.size() != nAct_ || !action.allFinite())
  {
    fail("the policy gave no valid action");
    return;
  }
  command_ = q0_ + scale_.cwiseProduct(action);
  // mjlab's FilteredJointPositionAction: the filter starts at the measured joint angles
  if(filterFresh_)
  {
    target_ = in_.jointPos;
    filterFresh_ = false;
  }
  target_ += alpha_ * (command_ - target_);
  // During the hand-over run() blends q_rl towards target_ every tick
  const bool handingOver = state_ == State::HandingOver;
  for(Eigen::Index j = 0; j < nAct_; ++j)
  {
    const int i = dof_[static_cast<size_t>(j)];
    if(!handingOver)
    {
      ctl_.q_rl(i) = std::clamp(target_(j), lower_(j), upper_(j));
    }
    ctl_.currentActionScaled(i) = scale_(j) * action(j);
  }
  // The observation's previous action is the raw network output
  lastAction_ = action;
  obs_ = obs;
  ctl_.currentObservation = obs;
  ctl_.currentAction = action;
  if(handingOver)
  {
    return; // the reference stays on the first frame
  }
  if(!endReported_ && frame_ + 1 >= motion_->frames())
  {
    endReported_ = true;
    state_ = State::Ended;
    mc_rtc::log::success("[Tracking] END: last frame of {} at t = {:.3f} s, the reference holds it -- HOLD to stop",
                         selected_, t_);
  }
  ++frame_;
}

double MotionTracker::motionTime() const
{
  if(!motion_)
  {
    return 0.0;
  }
  if(usedFrame_ < motion_->time.size())
  {
    return motion_->time[usedFrame_] - motion_->time.front();
  }
  return static_cast<double>(usedFrame_) * ctl_.policyStepSize;
}

double MotionTracker::duration() const
{
  if(!motion_)
  {
    return 0.0;
  }
  if(motion_->time.size() >= 2)
  {
    return motion_->time.back() - motion_->time.front();
  }
  return static_cast<double>(motion_->frames() - 1) * ctl_.policyStepSize;
}

void MotionTracker::buildGui()
{
  auto gui = ctl_.gui();
  if(!gui)
  {
    return;
  }
  gui->removeCategory(GUI_CATEGORY);
  if(!enabled_)
  {
    return;
  }
  using namespace mc_rtc::gui;
  gui->addElement(
      GUI_CATEGORY, Label("Tracking state", [this]() { return std::string(stateName(state_)); }),
      ComboInput(
          "Motion (disarm to change)", motionFiles_, [this]() { return selected_; },
          [this](const std::string & name) { select(name); }),
      Label("Motion status", [this]() { return motionStatus_; }),
      Label("Duration [s]", [this]() { return fmt::format("{:.2f}", duration()); }),
      Label("Motion time [s]", [this]() { return fmt::format("{:.2f}", motionTime()); }),
      Label("Frame", [this]() { return motion_ ? fmt::format("{} / {}", usedFrame_, motion_->frames() - 1) : "-"; }),
      Label("Motions directory", [this]() { return motionsDir_; }),
      Button("Rescan motions directory", [this]() { rescanRequested_ = true; }));
}

void MotionTracker::addLog()
{
  auto & logger = ctl_.logger();
  const auto * control = &ctl_.robot();
  const auto * real = &ctl_.realRobot(control->name());
  auto add = [&](const std::string & name, auto && get)
  { logger.addLogEntry(LOG_PREFIX + name, std::forward<decltype(get)>(get)); };
  add("policyIndex", [this]() { return static_cast<int>(ctl_.currentPolicyIndex); });
  add("motion", [this]() -> const std::string & { return selected_; });
  // 0 idle, 1 ramping, 2 settling, 3 tracking, 4 ended
  add("state", [this]() { return static_cast<int>(state_); });
  // From the tracking start [s]: counts up to 0 while ramping and settling, -1 while idle
  add("t", [this]() { return t_; });
  add("frame", [this]() { return static_cast<double>(usedFrame_); });
  add("motionTime", [this]() { return motionTime(); });
  add("action", [this]() -> const Eigen::VectorXd & { return lastAction_; });
  add("target", [this]() -> const Eigen::VectorXd & { return target_; });
  // The observed base (from tracking.base_source, as of the last observation) and the reference anchor
  add("baseSource", [this]() { return static_cast<int>(baseSource_); }); // 0 estimate, 1 ground truth
  add("anchorPos", [this]() -> const Eigen::Vector3d & { return in_.anchorPos; });
  add("anchorQuat", [this]() { return Eigen::Quaterniond(in_.anchorRot).normalized(); });
  add("baseLinVel", [this]() -> const Eigen::Vector3d & { return in_.baseLinVel; });
  add("baseAngVel", [this]() -> const Eigen::Vector3d & { return in_.baseAngVel; });
  add("refAnchorPos", [this]() -> const Eigen::Vector3d & { return in_.refAnchorPos; });
  add("refAnchorQuat", [this]() { return Eigen::Quaterniond(in_.refAnchorRot).normalized(); });
  // Every source, every tick, whatever the observed one
  add("estAnchorPos", [this]() -> const Eigen::Vector3d & { return est_.anchorPos; });
  add("estAnchorQuat", [this]() { return Eigen::Quaterniond(est_.anchorRot).normalized(); });
  add("estBaseLinVel", [this]() -> const Eigen::Vector3d & { return est_.linVel; });
  add("estBaseAngVel", [this]() -> const Eigen::Vector3d & { return est_.angVel; });
  add("observerAngVel", [this]() -> const Eigen::Vector3d & { return observerAngVel_; });
  if(control->hasBodySensor("FloatingBase"))
  {
    add("gtAnchorPos", [this]() -> const Eigen::Vector3d & { return gt_.anchorPos; });
    add("gtAnchorQuat", [this]() { return Eigen::Quaterniond(gt_.anchorRot).normalized(); });
    add("gtBaseLinVel", [this]() -> const Eigen::Vector3d & { return gt_.linVel; });
    add("gtBaseAngVel", [this]() -> const Eigen::Vector3d & { return gt_.angVel; });
  }
  add("baseEst_pos", [real]() -> Eigen::Vector3d { return real->posW().translation(); });
  add("baseEst_quat", [real]() { return Eigen::Quaterniond(real->posW().rotation().transpose()).normalized(); });
  if(control->hasBodySensor("FloatingBase"))
  {
    const auto * bs = &control->bodySensor("FloatingBase");
    add("baseGT_pos", [bs]() -> const Eigen::Vector3d & { return bs->position(); });
    add("baseGT_quat", [bs]() { return Eigen::Quaterniond(bs->orientation().inverse()).normalized(); });
    add("baseGT_linVelWorld", [bs]() -> const Eigen::Vector3d & { return bs->linearVelocity(); });
    add("baseGT_angVelBody", [bs]() -> const Eigen::Vector3d & { return bs->angularVelocity(); });
  }
  // With verbose_log only (NewRLQPController::verboseLog_): the observation, the command before the filter, the
  // PostureTask's target (the policy joints of NewRLQPController_RL_q), and the joint states in reference order (the
  // encoders are in qIn, the commanded angles in NewRLQPController_ctrl_q)
  if(!ctl_.verboseLog_)
  {
    return;
  }
  add("obs", [this]() -> const Eigen::VectorXd & { return obs_; });
  add("command", [this]() -> const Eigen::VectorXd & { return command_; });
  // The PostureTask's target of the policy joints, action order
  add("postureTarget",
      [this]() -> Eigen::VectorXd
      {
        Eigen::VectorXd out = Eigen::VectorXd::Constant(nAct_, std::numeric_limits<double>::quiet_NaN());
        auto pt = ctl_.getPostureTask(ctl_.robot().name());
        if(!pt)
        {
          return out;
        }
        const auto & posture = pt->posture();
        for(Eigen::Index j = 0; j < nAct_; ++j)
        {
          const auto & p = posture[mbcIndex_[static_cast<size_t>(j)]];
          if(!p.empty())
          {
            out(j) = p[0];
          }
        }
        return out;
      });
  add("qReal", [real]() { return toRefOrder(*real, real->mbc().q); });
  add("alphaReal", [real]() { return toRefOrder(*real, real->mbc().alpha); });
  add("tauReal", [real]() { return toRefOrder(*real, real->mbc().jointTorque); });
  add("qControl", [control]() { return toRefOrder(*control, control->mbc().q); });
  add("alphaControl", [control]() { return toRefOrder(*control, control->mbc().alpha); });
}
