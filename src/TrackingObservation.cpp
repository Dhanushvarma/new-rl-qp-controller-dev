#include "TrackingObservation.h"

#include <Eigen/Geometry>

Eigen::VectorXd buildTrackingObservation(const TrackingObservationInput & in, const Eigen::Vector3d & imuPos)
{
  const Eigen::Index size = 2 * in.refJointPos.size() + 3 + 6 + 3 + 3 + 2 * in.jointPos.size() + in.lastAction.size();
  Eigen::VectorXd obs(size);
  Eigen::Index offset = 0;
  auto put = [&](const Eigen::VectorXd & v)
  {
    obs.segment(offset, v.size()) = v;
    offset += v.size();
  };

  put(in.refJointPos);
  put(in.refJointVel);
  put(in.anchorRot.transpose() * (in.refAnchorPos - in.anchorPos));
  const Eigen::Matrix3d rel = in.anchorRot.transpose() * in.refAnchorRot;
  Eigen::VectorXd ori(6);
  ori << rel(0, 0), rel(0, 1), rel(1, 0), rel(1, 1), rel(2, 0), rel(2, 1);
  put(ori);
  put(in.baseLinVel + in.baseAngVel.cross(imuPos));
  put(in.baseAngVel);
  put(in.jointPos - in.q0);
  put(in.jointVel);
  put(in.lastAction);
  return obs;
}
