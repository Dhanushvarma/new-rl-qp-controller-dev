#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

/** A reference motion for a tracking policy, one frame per policy step.
 *
 * Read from the motion CSV of mjlab's scripts/hrp5p/export_deploy.py: a header row, then one row per frame with
 * `t`, `q_<joint>` for every joint, `qd_<joint>` for every joint, the anchor position `anchor_x/y/z` and its
 * orientation as a quaternion `anchor_qw/qx/qy/qz` (world frame of the motion). The motions of link tracking policies
 * also hold each target link's pose, `link_<name>_x/y/z/qw/qx/qy/qz`.
 *
 * alignTo() moves the motion so that its first frame's anchor has the robot anchor's horizontal position and heading;
 * heights, roll and pitch stay the motion's. The aligned anchor pose of every frame then follows from the same yaw
 * rotation and offset.
 */
struct TrackingMotion
{
  /** Load the CSV, throw on a malformed file */
  void load(const std::string & path);

  size_t frames() const
  {
    return jointPos.size();
  }

  /** Align the first frame's anchor to the robot anchor (position and orientation in the world frame) */
  void alignTo(const Eigen::Vector3d & robotAnchorPos, const Eigen::Matrix3d & robotAnchorRot);

  /** Anchor position of frame k after alignTo(), world frame */
  Eigen::Vector3d anchorPos(size_t k) const;

  /** Anchor orientation of frame k after alignTo(), anchor to world */
  Eigen::Matrix3d anchorRot(size_t k) const;

  /** Index of a target link in linkNames, throw if the motion has no such link */
  size_t linkIndex(const std::string & name) const;

  /** Position of target link i in frame k after alignTo(), world frame */
  Eigen::Vector3d linkPos(size_t k, size_t i) const;

  /** Orientation of target link i in frame k after alignTo(), link to world */
  Eigen::Matrix3d linkRot(size_t k, size_t i) const;

  std::vector<double> time; ///< Per frame, the `t` column (empty if the file has none)
  std::vector<std::string> jointNames; ///< Motion joint order
  std::vector<Eigen::VectorXd> jointPos; ///< Per frame, motion joint order
  std::vector<Eigen::VectorXd> jointVel; ///< Per frame, motion joint order
  std::vector<Eigen::Vector3d> rawAnchorPos; ///< Per frame, as in the file
  std::vector<Eigen::Quaterniond> rawAnchorQuat; ///< Per frame, as in the file
  std::vector<std::string> linkNames; ///< Target links with pose columns, file order
  std::vector<std::vector<Eigen::Vector3d>> rawLinkPos; ///< Per frame, per target link, as in the file
  std::vector<std::vector<Eigen::Quaterniond>> rawLinkQuat; ///< Per frame, per target link, as in the file

private:
  Eigen::Matrix3d alignRot_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d alignFrom_ = Eigen::Vector3d::Zero(); ///< First frame's horizontal anchor position
  Eigen::Vector3d alignTo_ = Eigen::Vector3d::Zero(); ///< Robot's horizontal anchor position
};
