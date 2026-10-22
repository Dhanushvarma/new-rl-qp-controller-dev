#include "NewRLQPController_Initial.h"

#include "../NewRLQPController.h"

void NewRLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void NewRLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  // Deliberately does NOT call start_rl_state. Loading the controller and
  // running the policy are separate events on real hardware: the ticker can
  // come up, the QP can stabilise and the operator can inspect the GUI before
  // anything moves. start_rl_state runs on the arming edge in run() instead,
  // so its measured-posture capture and ramp to q_zero start from wherever the
  // robot actually is at that moment, not from where it was at load time.
  mc_rtc::log::info("[NewRLQPController_Initial] loaded, policy held -- press 'ARM policy' in the GUI to start");
}

bool NewRLQPController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<NewRLQPController &>(ctl_);

  if(!ctl.policyArmed_)
  {
    // Held: no inference, no posture write, so the PostureTask keeps the
    // posture mc_rtc captured at reset. Clearing rlStarted_ means a re-arm
    // after a disarm re-captures the posture and ramps again, rather than
    // resuming from a stale target.
    if(rlStarted_) { ctl.tracker_.stop(); }
    rlStarted_ = false;
    return false;
  }

  // Tracking slots (obs_format 9) run MotionTracker: ramp to the motion's first
  // frame, settle, track; the others the go-to-init ramp and run_rl_state.
  const bool tracking = ctl.tracker_.enabled();
  if(!rlStarted_)
  {
    if(tracking)
    {
      if(!ctl.tracker_.start())
      {
        ctl.disarm(DisarmReason::Tracking);
        return false;
      }
    }
    else { ctl.utilsClass.start_rl_state(ctl, "RL_State"); }
    rlStarted_ = true;
  }

  if(tracking) { ctl.tracker_.run(); }
  else { ctl.utilsClass.run_rl_state(ctl); }
  // MotionTracker disarms on an error: leave the posture target where it is
  if(!ctl.policyArmed_) { return false; }

  auto pt = ctl.getPostureTask(ctl.robot().name());
  if(!pt) { return false; }
  auto posture = pt->posture();
  for(int i = 0; i < ctl.nbActuatedJoints; ++i)
  {
    const int mcIdx = ctl.robot().jointIndexByName(ctl.jointNames[i]);
    posture[mcIdx][0] = ctl.q_rl[i];
  }
  pt->posture(posture);
  return false;
}

void NewRLQPController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<NewRLQPController &>(ctl_);
}

EXPORT_SINGLE_STATE("NewRLQPController_Initial", NewRLQPController_Initial)
