// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#include "uav_planning/astar_3d.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "tinyxml2.h"

namespace uav_planning
{
namespace
{
double distance(const Point3D & a, const Point3D & b)
{
  return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

Point3D translation(const tinyxml2::XMLElement * element)
{
  Point3D offset{};
  const auto * pose = element->FirstChildElement("pose");
  if (!pose) {
    return offset;
  }
  if (pose->Attribute("relative_to") || !pose->GetText()) {
    throw std::invalid_argument("建筑pose需要当前world下的无旋转坐标，不支持relative_to。");
  }
  std::istringstream input(pose->GetText());
  std::array<double, 6> values{};
  for (double & value : values) {
    if (!(input >> value) || !std::isfinite(value)) {
      throw std::invalid_argument("建筑pose需要6个有限数值。");
    }
  }
  if (std::abs(values[3]) + std::abs(values[4]) + std::abs(values[5]) > 1e-9) {
    throw std::invalid_argument("当前栅格读取器只支持无旋转的长方体建筑。");
  }
  std::copy_n(values.begin(), 3, offset.begin());
  return offset;
}
}  // namespace

std::vector<Box3D> load_buildings(const std::string & world_file)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(world_file.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::invalid_argument("不能读取SDF：" + world_file);
  }
  const auto * sdf = document.FirstChildElement("sdf");
  const auto * world = sdf ? sdf->FirstChildElement("world") : nullptr;
  if (!world) {
    throw std::invalid_argument("SDF缺少world。");
  }
  std::vector<Box3D> boxes;
  for (auto * model = world->FirstChildElement("model"); model;
    model = model->NextSiblingElement("model"))
  {
    const char * name = model->Attribute("name");
    if (!name || std::string(name).find("building_") != 0) {
      continue;
    }
    const auto * fixed = model->FirstChildElement("static");
    if (!fixed || !fixed->BoolText()) {
      throw std::invalid_argument("building_*必须是static建筑。");
    }
    const auto model_offset = translation(model);
    std::size_t count = 0;
    for (auto * link = model->FirstChildElement("link"); link;
      link = link->NextSiblingElement("link"))
    {
      const auto link_offset = translation(link);
      for (auto * collision = link->FirstChildElement("collision"); collision;
        collision = collision->NextSiblingElement("collision"))
      {
        const auto collision_offset = translation(collision);
        const auto * geometry = collision->FirstChildElement("geometry");
        const auto * box = geometry ? geometry->FirstChildElement("box") : nullptr;
        const auto * size = box ? box->FirstChildElement("size") : nullptr;
        if (!size || !size->GetText()) {
          throw std::invalid_argument("building_*的collision必须使用box/size。");
        }
        std::istringstream input(size->GetText());
        Box3D bounds{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
          double length;
          if (!(input >> length) || !std::isfinite(length) || length <= 0.0) {
            throw std::invalid_argument("建筑尺寸需要3个正有限数值。");
          }
          const double center = model_offset[axis] + link_offset[axis] + collision_offset[axis];
          bounds.min[axis] = center - length / 2.0;
          bounds.max[axis] = center + length / 2.0;
        }
        boxes.push_back(bounds);
        ++count;
      }
    }
    if (count == 0) {
      throw std::invalid_argument("building_*缺少collision，拒绝生成不完整地图。");
    }
  }
  if (boxes.empty()) {
    throw std::invalid_argument("SDF没有可读取的building_*长方体建筑。");
  }
  return boxes;
}

AStar3D::AStar3D(
  const Point3D & lower, const Point3D & upper, double resolution,
  double clearance, const std::vector<Box3D> & buildings)
: lower_(lower), upper_(upper), resolution_(resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(clearance) || clearance < 0.0)
  {
    throw std::invalid_argument("栅格分辨率必须为正，净空必须非负。");
  }
  std::size_t count = 1;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double cells = std::floor((upper[axis] - lower[axis]) / resolution) + 1.0;
    if (!std::isfinite(lower[axis]) || !std::isfinite(upper[axis]) ||
      lower[axis] >= upper[axis] || !std::isfinite(cells) || cells < 1 || cells > 5000000)
    {
      throw std::invalid_argument("栅格边界或尺寸无效。");
    }
    shape_[axis] = static_cast<int>(cells);
    count *= shape_[axis];
    if (count > 5000000) {
      throw std::invalid_argument("演示地图最多500万格，请增大resolution。");
    }
  }
  obstacles_ = buildings;
  for (auto & box : obstacles_) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      // 净空已包含机体和跟踪余量；再外扩半格，保守处理栅格离散边界。
      box.min[axis] -= clearance + resolution / 2.0;
      box.max[axis] += clearance + resolution / 2.0;
    }
  }
  occupied_.resize(count);
  for (int id = 0; id < static_cast<int>(count); ++id) {
    const auto point = position(cell(id));
    occupied_[id] = !segment_free(point, point);
  }
}

bool AStar3D::inside(const Point3D & point) const
{
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(point[axis]) || point[axis] < lower_[axis] || point[axis] > upper_[axis]) {
      return false;
    }
  }
  return true;
}

AStar3D::Cell AStar3D::cell(const Point3D & point) const
{
  Cell result{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    result[axis] = static_cast<int>(std::lround((point[axis] - lower_[axis]) / resolution_));
  }
  return result;
}

AStar3D::Cell AStar3D::cell(int id) const
{
  return {id % shape_[0], (id / shape_[0]) % shape_[1], id / (shape_[0] * shape_[1])};
}

int AStar3D::index(const Cell & cell) const
{
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (cell[axis] < 0 || cell[axis] >= shape_[axis]) {
      return -1;
    }
  }
  return (cell[2] * shape_[1] + cell[1]) * shape_[0] + cell[0];
}

Point3D AStar3D::position(const Cell & cell) const
{
  return {lower_[0] + cell[0] * resolution_, lower_[1] + cell[1] * resolution_,
    lower_[2] + cell[2] * resolution_};
}

bool AStar3D::segment_free(const Point3D & start, const Point3D & goal) const
{
  if (!inside(start) || !inside(goal)) {
    return false;
  }
  for (const auto & box : obstacles_) {
    // Slab线段/长方体相交检查：整段检查，不能只检查端点或稀疏采样。
    double first = 0.0, last = 1.0;
    for (std::size_t axis = 0; axis < 3 && first <= last; ++axis) {
      const double delta = goal[axis] - start[axis];
      if (std::abs(delta) < 1e-12) {
        if (start[axis] < box.min[axis] || start[axis] > box.max[axis]) {
          first = 2.0;
        }
      } else {
        const double a = (box.min[axis] - start[axis]) / delta;
        const double b = (box.max[axis] - start[axis]) / delta;
        first = std::max(first, std::min(a, b));
        last = std::min(last, std::max(a, b));
      }
    }
    if (first <= last) {
      return false;
    }
  }
  return true;
}

SearchResult AStar3D::plan(const Point3D & start, const Point3D & goal) const
{
  SearchResult result;
  if (!inside(start) || !inside(goal)) {
    return result;
  }
  const int begin = index(cell(start)), end = index(cell(goal));
  if (begin < 0 || end < 0 || occupied_[begin] || occupied_[end] ||
    !segment_free(start, position(cell(begin))) || !segment_free(position(cell(end)), goal))
  {
    return result;
  }
  const auto end_position = position(cell(end));
  std::vector<double> cost(occupied_.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(occupied_.size(), -1);
  std::vector<unsigned char> closed(occupied_.size(), 0);
  using Entry = std::pair<double, int>;  // f=g+h，优先取f最小的栅格。
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  cost[begin] = 0.0;
  open.emplace(distance(position(cell(begin)), end_position), begin);
  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[current]) {
      continue;
    }
    closed[current] = 1;
    ++result.expanded;
    if (current == end) {
      std::vector<Point3D> reversed;
      for (int id = end; id != -1; id = parent[id]) {
        reversed.push_back(position(cell(id)));
      }
      result.points.push_back(start);
      result.points.insert(result.points.end(), reversed.rbegin(), reversed.rend());
      result.points.push_back(goal);
      result.points.erase(
        std::unique(result.points.begin(), result.points.end()),
        result.points.end());
      return result;
    }
    const auto here = cell(current);
    const auto here_position = position(here);
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const Cell neighbor{here[0] + dx, here[1] + dy, here[2] + dz};
          const int next = index(neighbor);
          if (next < 0 || occupied_[next] || closed[next]) {
            continue;
          }
          const auto point = position(neighbor);
          const double candidate = cost[current] + distance(here_position, point);
          // 26邻域斜向连线同样做连续碰撞检查，禁止从建筑边角穿过去。
          if (candidate < cost[next] && segment_free(here_position, point)) {
            parent[next] = current;
            cost[next] = candidate;
            open.emplace(candidate + distance(point, end_position), next);
          }
        }
      }
    }
  }
  return result;
}

std::vector<Point3D> AStar3D::simplify(const std::vector<Point3D> & path) const
{
  if (path.empty()) {
    return {};
  }
  std::vector<Point3D> result{path.front()};
  for (std::size_t current = 0; current + 1 < path.size(); ) {
    std::size_t next = path.size() - 1;
    while (next > current + 1 && !segment_free(path[current], path[next])) {
      --next;
    }
    if (!segment_free(path[current], path[next])) {
      return {};
    }
    result.push_back(path[next]);
    current = next;
  }
  return result;
}
}  // namespace uav_planning
