#pragma once

/** Why the policy was last disarmed (HOLD), logged as NewRLQPController_disarmReason (its int value) and shown in the
 * GUI. Set by NewRLQPController::disarm(); back to None when the policy is armed or the controller resets. */
enum class DisarmReason
{
  None = 0, ///< Armed, or not disarmed since the controller started
  Operator = 1, ///< HOLD from the GUI
  Runaway = 2, ///< Runaway guard: a measured joint speed above runaway_disarm_vel
  Tilt = 3, ///< Fall guard: the observed torso tilt above fall_disarm_tilt
  Inference = 4, ///< The policy's inference failed
  Tracking = 5 ///< Another motion tracking error (MotionTracker::fail), or arming refused by it
};

inline const char * disarmReasonName(DisarmReason reason)
{
  switch(reason)
  {
    case DisarmReason::None:
      return "none";
    case DisarmReason::Operator:
      return "operator (HOLD)";
    case DisarmReason::Runaway:
      return "runaway guard";
    case DisarmReason::Tilt:
      return "fall guard (tilt)";
    case DisarmReason::Inference:
      return "inference failed";
    case DisarmReason::Tracking:
      return "tracking error";
  }
  return "?";
}
