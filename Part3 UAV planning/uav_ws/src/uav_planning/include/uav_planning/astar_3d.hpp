// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef UAV_PLANNING__ASTAR_3D_HPP_
#define UAV_PLANNING__ASTAR_3D_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace uav_planning
{
using Point3D = std::array<double, 3>;

struct Box3D
{
  Point3D min;
  Point3D max;
};

// 当前SDF的world直属building_*模型：static、无旋转、长方体collision。
// 不读取visual、水面标记或X500；不支持的建筑几何直接报错，不静默漏掉障碍。
std::vector<Box3D> load_buildings(const std::string & world_file);

struct SearchResult
{
  std::vector<Point3D> points;
  std::size_t expanded{0};
};

// map/ENU，单位米。26邻域A*，边代价和启发函数均为三维欧氏距离。
class AStar3D
{
public:
  AStar3D(
    const Point3D & lower, const Point3D & upper, double resolution,
    double clearance, const std::vector<Box3D> & buildings);

  // 无路、端点越界或落入膨胀障碍时返回空路径，绝不退回直接连终点。
  SearchResult plan(const Point3D & start, const Point3D & goal) const;
  std::vector<Point3D> simplify(const std::vector<Point3D> & path) const;
  bool segment_free(const Point3D & start, const Point3D & goal) const;
  std::size_t cell_count() const {return occupied_.size();}

private:
  using Cell = std::array<int, 3>;
  bool inside(const Point3D & point) const;
  Cell cell(const Point3D & point) const;
  Cell cell(int index) const;
  int index(const Cell & cell) const;
  Point3D position(const Cell & cell) const;

  Point3D lower_, upper_;
  double resolution_;
  Cell shape_{};
  std::vector<Box3D> obstacles_;
  std::vector<unsigned char> occupied_;
};
}  // namespace uav_planning

#endif  // UAV_PLANNING__ASTAR_3D_HPP_
