#pragma once

#include <Eigen/Core>

/** Inputs of a motion tracking policy's observation, as the controller has them.
 *
 * Frames: "base" is the floating base body (its origin and axes), "world" the estimator's world, "anchor" the
 * motion's anchor body. Rotations map vectors from the body to the world frame.
 */
struct TrackingObservationInput
{
  Eigen::Vector3d baseLinVel = Eigen::Vector3d::Zero(); ///< Base origin velocity, base frame
  Eigen::Vector3d baseAngVel = Eigen::Vector3d::Zero(); ///< Base angular velocity, base frame
  Eigen::Vector3d anchorPos = Eigen::Vector3d::Zero(); ///< Robot anchor position, world frame
  Eigen::Matrix3d anchorRot = Eigen::Matrix3d::Identity(); ///< Robot anchor orientation
  Eigen::Vector3d refAnchorPos = Eigen::Vector3d::Zero(); ///< Reference anchor position, world frame
  Eigen::Matrix3d refAnchorRot = Eigen::Matrix3d::Identity(); ///< Reference anchor orientation
  Eigen::VectorXd refJointPos; ///< Reference joint angles, all joints in the motion's order
  Eigen::VectorXd refJointVel; ///< Reference joint velocities, same order
  Eigen::VectorXd jointPos; ///< Measured angles of the policy joints, action order
  Eigen::VectorXd jointVel; ///< Measured velocities of the policy joints, action order
  Eigen::VectorXd q0; ///< Default angles of the policy joints, action order
  Eigen::VectorXd lastAction; ///< The policy's previous raw output (zero before the first)
};

/** The observation of mjlab's HRP5P motion tracking policies, term by term in mjlab's order:
 *
 * - command: reference joint angles, then reference joint velocities
 * - motion_anchor_pos_b: reference anchor position in the robot anchor frame
 * - motion_anchor_ori_b: the first two columns of the reference anchor orientation in the robot anchor frame, row by
 *   row
 * - base_lin_vel: velocity of the IMU site (at imuPos in the base frame, aligned with it), base frame
 * - base_ang_vel: base angular velocity, base frame
 * - joint_pos: policy joint angles minus their default angles
 * - joint_vel: policy joint velocities
 * - actions: the previous raw action
 *
 * Its size is 2 * (motion joints) + 15 + 3 * (policy joints): 220 for HRP5P (53 motion joints, 33 policy joints).
 */
Eigen::VectorXd buildTrackingObservation(const TrackingObservationInput & in, const Eigen::Vector3d & imuPos);
