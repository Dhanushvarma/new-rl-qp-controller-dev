/** Checks buildTrackingObservation, the builder of the tracking observation (obs_format 9), against mjlab.
 *
 * Reads a golden CSV of mjlab's scripts/hrp5p/export_deploy.py: one row per policy step with the observation's
 * inputs as the controller has them and the observation mjlab computed. Rebuilds the observation from the inputs and
 * prints the largest difference per term; fails above 1e-5.
 *
 * Usage: test_tracking_observation <golden.csv> <imu_x> <imu_y> <imu_z>
 */
#include "TrackingObservation.h"

#include <Eigen/Geometry>

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while(std::getline(ss, cell, ','))
  {
    cells.push_back(cell);
  }
  return cells;
}

} // namespace

int main(int argc, char ** argv)
{
  if(argc != 5)
  {
    std::cerr << "Usage: " << argv[0] << " <golden.csv> <imu_x> <imu_y> <imu_z>\n";
    return 2;
  }
  const Eigen::Vector3d imu(std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
  std::ifstream file(argv[1]);
  std::string line;
  if(!file || !std::getline(file, line))
  {
    std::cerr << "Cannot read " << argv[1] << "\n";
    return 2;
  }
  // Columns are <name>_<index>; group them by name, in order
  std::map<std::string, std::vector<size_t>> columns;
  const auto header = split(line);
  for(size_t i = 0; i < header.size(); ++i)
  {
    columns[header[i].substr(0, header[i].rfind('_'))].push_back(i);
  }
  const std::vector<std::pair<std::string, int>> terms = {{"command", 0},      {"anchor_pos_b", 3}, {"anchor_ori_b", 6},
                                                          {"base_lin_vel", 3}, {"base_ang_vel", 3}, {"joint_pos", 0},
                                                          {"joint_vel", 0},    {"actions", 0}};
  std::vector<double> worst(terms.size(), 0.0);
  size_t rows = 0;
  while(std::getline(file, line))
  {
    const auto cells = split(line);
    auto vec = [&](const std::string & name)
    {
      const auto & idx = columns.at(name);
      Eigen::VectorXd v(static_cast<Eigen::Index>(idx.size()));
      for(size_t i = 0; i < idx.size(); ++i)
      {
        v(static_cast<Eigen::Index>(i)) = std::stod(cells[idx[i]]);
      }
      return v;
    };
    auto rot = [&](const std::string & name)
    {
      const Eigen::VectorXd q = vec(name);
      return Eigen::Quaterniond(q(0), q(1), q(2), q(3)).normalized().toRotationMatrix();
    };
    TrackingObservationInput in;
    in.baseLinVel = vec("base_lin_vel_b");
    in.baseAngVel = vec("base_ang_vel_b");
    in.anchorPos = vec("anchor_pos_w");
    in.anchorRot = rot("anchor_quat_w");
    in.refAnchorPos = vec("ref_anchor_pos_w");
    in.refAnchorRot = rot("ref_anchor_quat_w");
    in.refJointPos = vec("ref_joint_pos");
    in.refJointVel = vec("ref_joint_vel");
    in.jointPos = vec("joint_pos");
    in.jointVel = vec("joint_vel");
    in.q0 = vec("q0");
    in.lastAction = vec("last_action");
    const Eigen::VectorXd obs = buildTrackingObservation(in, imu);
    const Eigen::VectorXd expected = vec("obs");
    if(obs.size() != expected.size())
    {
      std::cerr << "Observation size " << obs.size() << ", mjlab's " << expected.size() << "\n";
      return 1;
    }
    const Eigen::Index nRef = in.refJointPos.size(), nAct = in.jointPos.size();
    const std::vector<Eigen::Index> sizes = {2 * nRef, 3, 6, 3, 3, nAct, nAct, nAct};
    Eigen::Index offset = 0;
    for(size_t t = 0; t < terms.size(); ++t)
    {
      const double d = (obs.segment(offset, sizes[t]) - expected.segment(offset, sizes[t])).cwiseAbs().maxCoeff();
      worst[t] = std::max(worst[t], d);
      offset += sizes[t];
    }
    ++rows;
  }
  bool ok = rows > 0;
  for(size_t t = 0; t < terms.size(); ++t)
  {
    std::cout << terms[t].first << ": max |difference| " << worst[t] << "\n";
    ok = ok && worst[t] < 1e-5;
  }
  std::cout << rows << " rows: " << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
