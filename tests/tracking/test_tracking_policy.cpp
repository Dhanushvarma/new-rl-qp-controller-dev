/** Checks the tracking policy as the controller runs it (RLPolicyInterface) against mjlab.
 *
 * Reads a golden CSV of mjlab's scripts/hrp5p/export_deploy.py (one row per policy step, with the observation `obs_*`
 * and the policy's raw output `action_*`), runs the ONNX policy on each observation and prints the largest difference
 * to mjlab's action; fails above 1e-4 (the policy runs in float32), or if a wrong-size observation does not throw.
 *
 * Usage: test_tracking_policy <golden.csv> <policy.onnx>
 */
#include "RLPolicyInterface.h"

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
  if(argc != 3)
  {
    std::cerr << "Usage: " << argv[0] << " <golden.csv> <policy.onnx>\n";
    return 2;
  }
  std::ifstream file(argv[1]);
  std::string line;
  if(!file || !std::getline(file, line))
  {
    std::cerr << "Cannot read " << argv[1] << "\n";
    return 2;
  }
  RLPolicyInterface policy(argv[2]);
  if(!policy.isLoaded())
  {
    std::cerr << "Cannot load " << argv[2] << "\n";
    return 1;
  }
  // Columns are <name>_<index>; group them by name, in order
  std::map<std::string, std::vector<size_t>> columns;
  const auto header = split(line);
  for(size_t i = 0; i < header.size(); ++i)
  {
    columns[header[i].substr(0, header[i].rfind('_'))].push_back(i);
  }
  const auto & obsCols = columns.at("obs");
  const auto & actionCols = columns.at("action");
  if(static_cast<int>(obsCols.size()) != policy.getObservationSize()
     || static_cast<int>(actionCols.size()) != policy.getActionSize())
  {
    std::cerr << "The golden CSV has " << obsCols.size() << " observations and " << actionCols.size()
              << " actions, the policy " << policy.getObservationSize() << " and " << policy.getActionSize() << "\n";
    return 1;
  }
  double worst = 0.0;
  size_t rows = 0;
  while(std::getline(file, line))
  {
    const auto cells = split(line);
    Eigen::VectorXd obs(static_cast<Eigen::Index>(obsCols.size()));
    for(size_t i = 0; i < obsCols.size(); ++i)
    {
      obs(static_cast<Eigen::Index>(i)) = std::stod(cells[obsCols[i]]);
    }
    const Eigen::VectorXd action = policy.predict(obs);
    for(size_t i = 0; i < actionCols.size(); ++i)
    {
      worst = std::max(worst, std::abs(action(static_cast<Eigen::Index>(i)) - std::stod(cells[actionCols[i]])));
    }
    ++rows;
  }
  // A failed inference throws rather than answering zeros (a jump to q0): the controller disarms on it
  bool throws = false;
  try
  {
    policy.predict(Eigen::VectorXd::Zero(policy.getObservationSize() + 1));
  }
  catch(const std::runtime_error &)
  {
    throws = true;
  }
  const bool ok = rows > 0 && worst < 1e-4 && throws;
  std::cout << "action: max |difference| " << worst << "\n"
            << "wrong observation size: " << (throws ? "throws" : "does not throw") << "\n"
            << rows << " rows: " << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
