#include "TrackingMotion.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace
{

std::vector<std::string> splitCsv(std::string line)
{
  if(!line.empty() && line.back() == '\r')
  {
    line.pop_back();
  }
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while(std::getline(ss, cell, ','))
  {
    cells.push_back(cell);
  }
  return cells;
}

double yaw(const Eigen::Matrix3d & rot)
{
  return std::atan2(rot(1, 0), rot(0, 0));
}

} // namespace

void TrackingMotion::load(const std::string & path)
{
  std::ifstream file(path);
  if(!file)
  {
    throw std::runtime_error("Cannot open motion file " + path);
  }
  std::string line;
  if(!std::getline(file, line))
  {
    throw std::runtime_error("Empty motion file " + path);
  }
  const auto header = splitCsv(line);

  jointNames.clear();
  std::vector<size_t> qCols, qdCols;
  for(size_t i = 0; i < header.size(); ++i)
  {
    if(header[i].rfind("q_", 0) == 0)
    {
      jointNames.push_back(header[i].substr(2));
      qCols.push_back(i);
    }
    else if(header[i].rfind("qd_", 0) == 0)
    {
      qdCols.push_back(i);
    }
  }
  auto column = [&](const std::string & name)
  {
    for(size_t i = 0; i < header.size(); ++i)
    {
      if(header[i] == name)
      {
        return i;
      }
    }
    throw std::runtime_error("Motion file " + path + " has no column " + name);
  };
  size_t timeCol = header.size();
  for(size_t i = 0; i < header.size(); ++i)
  {
    if(header[i] == "t")
    {
      timeCol = i;
    }
  }
  const std::vector<size_t> anchorCols = {column("anchor_x"),  column("anchor_y"),  column("anchor_z"),
                                          column("anchor_qw"), column("anchor_qx"), column("anchor_qy"),
                                          column("anchor_qz")};
  // Target link poses: link_<name>_x .. link_<name>_qz
  linkNames.clear();
  std::vector<std::vector<size_t>> linkCols;
  const std::vector<std::string> poseSuffix = {"_x", "_y", "_z", "_qw", "_qx", "_qy", "_qz"};
  for(const auto & h : header)
  {
    if(h.rfind("link_", 0) == 0 && h.size() > 7 && h.compare(h.size() - 2, 2, "_x") == 0)
    {
      const std::string name = h.substr(5, h.size() - 7);
      linkNames.push_back(name);
      std::vector<size_t> cols;
      for(const auto & suffix : poseSuffix)
      {
        cols.push_back(column("link_" + name + suffix));
      }
      linkCols.push_back(cols);
    }
  }
  if(jointNames.empty() || qdCols.size() != qCols.size())
  {
    throw std::runtime_error("Motion file " + path + " needs q_<joint> and qd_<joint> for the same joints");
  }
  for(size_t j = 0; j < qCols.size(); ++j)
  {
    if(header[qdCols[j]] != "qd_" + jointNames[j])
    {
      throw std::runtime_error("Motion file " + path + ": qd_ columns are not in the q_ columns' order");
    }
  }

  time.clear();
  jointPos.clear();
  jointVel.clear();
  rawAnchorPos.clear();
  rawAnchorQuat.clear();
  rawLinkPos.clear();
  rawLinkQuat.clear();
  const Eigen::Index n = static_cast<Eigen::Index>(jointNames.size());
  while(std::getline(file, line))
  {
    if(line.empty() || line == "\r")
    {
      continue;
    }
    const auto cells = splitCsv(line);
    if(cells.size() != header.size())
    {
      throw std::runtime_error("Motion file " + path + ": row " + std::to_string(jointPos.size() + 1)
                               + " has the wrong number of cells");
    }
    Eigen::VectorXd q(n), qd(n);
    for(Eigen::Index j = 0; j < n; ++j)
    {
      q(j) = std::stod(cells[qCols[static_cast<size_t>(j)]]);
      qd(j) = std::stod(cells[qdCols[static_cast<size_t>(j)]]);
    }
    if(timeCol < header.size())
    {
      time.push_back(std::stod(cells[timeCol]));
    }
    jointPos.push_back(q);
    jointVel.push_back(qd);
    rawAnchorPos.emplace_back(std::stod(cells[anchorCols[0]]), std::stod(cells[anchorCols[1]]),
                              std::stod(cells[anchorCols[2]]));
    rawAnchorQuat.push_back(Eigen::Quaterniond(std::stod(cells[anchorCols[3]]), std::stod(cells[anchorCols[4]]),
                                               std::stod(cells[anchorCols[5]]), std::stod(cells[anchorCols[6]]))
                                .normalized());
    std::vector<Eigen::Vector3d> linkPos;
    std::vector<Eigen::Quaterniond> linkQuat;
    for(const auto & cols : linkCols)
    {
      linkPos.emplace_back(std::stod(cells[cols[0]]), std::stod(cells[cols[1]]), std::stod(cells[cols[2]]));
      linkQuat.push_back(Eigen::Quaterniond(std::stod(cells[cols[3]]), std::stod(cells[cols[4]]),
                                            std::stod(cells[cols[5]]), std::stod(cells[cols[6]]))
                             .normalized());
    }
    rawLinkPos.push_back(linkPos);
    rawLinkQuat.push_back(linkQuat);
  }
  if(jointPos.empty())
  {
    throw std::runtime_error("Motion file " + path + " has no frames");
  }
  alignRot_.setIdentity();
  alignFrom_.setZero();
  alignTo_.setZero();
}

void TrackingMotion::alignTo(const Eigen::Vector3d & robotAnchorPos, const Eigen::Matrix3d & robotAnchorRot)
{
  const double dYaw = yaw(robotAnchorRot) - yaw(rawAnchorQuat.front().toRotationMatrix());
  alignRot_ = Eigen::AngleAxisd(dYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  alignFrom_ = Eigen::Vector3d(rawAnchorPos.front().x(), rawAnchorPos.front().y(), 0.0);
  alignTo_ = Eigen::Vector3d(robotAnchorPos.x(), robotAnchorPos.y(), 0.0);
}

Eigen::Vector3d TrackingMotion::anchorPos(size_t k) const
{
  return alignRot_ * (rawAnchorPos[k] - alignFrom_) + alignTo_;
}

Eigen::Matrix3d TrackingMotion::anchorRot(size_t k) const
{
  return alignRot_ * rawAnchorQuat[k].toRotationMatrix();
}

size_t TrackingMotion::linkIndex(const std::string & name) const
{
  for(size_t i = 0; i < linkNames.size(); ++i)
  {
    if(linkNames[i] == name)
    {
      return i;
    }
  }
  throw std::runtime_error("The motion has no pose columns for target link " + name);
}

Eigen::Vector3d TrackingMotion::linkPos(size_t k, size_t i) const
{
  return alignRot_ * (rawLinkPos[k][i] - alignFrom_) + alignTo_;
}

Eigen::Matrix3d TrackingMotion::linkRot(size_t k, size_t i) const
{
  return alignRot_ * rawLinkQuat[k][i].toRotationMatrix();
}
