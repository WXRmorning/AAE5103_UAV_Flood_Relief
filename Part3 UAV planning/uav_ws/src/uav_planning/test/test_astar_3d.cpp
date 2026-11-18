// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "uav_planning/astar_3d.hpp"

namespace
{
using uav_planning::AStar3D;
using uav_planning::Point3D;

TEST(AStar3D, OpenSpaceKeepsExactEndpoints)
{
  const AStar3D planner({0, 0, 0}, {6, 6, 6}, 0.5, 0.0, {});
  const Point3D start{0.23, 0.37, 0.41}, goal{5.1, 5.2, 5.3};
  const auto search = planner.plan(start, goal);
  ASSERT_FALSE(search.points.empty());
  EXPECT_GT(search.expanded, 0u);  // 搜索确实执行，不是直接把输入当输出。
  const auto path = planner.simplify(search.points);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);
}

TEST(AStar3D, WallForcesAnActualThreeDimensionalOverflight)
{
  // 横墙封住全部Y，只有从Z>3.25越过墙顶才能抵达，检验真正的三维搜索。
  const AStar3D planner({0, 0, 0}, {8, 6, 6}, 0.5, 0.0,
    {{{3, -1, -1}, {5, 7, 3}}});
  const Point3D start{1, 3, 1}, goal{7, 3, 1};
  EXPECT_FALSE(planner.segment_free(start, goal));
  const auto path = planner.simplify(planner.plan(start, goal).points);
  ASSERT_GT(path.size(), 2u);
  bool above = false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(planner.segment_free(path[i - 1], path[i]));
    above = above || path[i][2] > 3.25;
  }
  EXPECT_TRUE(above);
}

TEST(AStar3D, SealedWallHasNoPath)
{
  const AStar3D planner({0, 0, 0}, {8, 6, 6}, 0.5, 0.0,
    {{{3, -1, -1}, {5, 7, 7}}});
  EXPECT_TRUE(planner.plan({1, 3, 1}, {7, 3, 1}).points.empty());
}

TEST(AStar3D, InflatedObstacleAndInvalidEndpointsAreRejected)
{
  const AStar3D planner({0, 0, 0}, {8, 8, 8}, 0.5, 1.0,
    {{{3, 3, 3}, {5, 5, 5}}});
  EXPECT_TRUE(planner.plan({1, 1, 1}, {2, 4, 4}).points.empty());
  EXPECT_TRUE(planner.plan({1, 1, 1}, {9, 1, 1}).points.empty());
  EXPECT_TRUE(planner.plan({1, 1, 1}, {NAN, 1, 1}).points.empty());
}

TEST(AStar3D, DiagonalAndSimplificationCannotCutBuildingCorners)
{
  const AStar3D planner({0, 0, 0}, {6, 6, 6}, 0.5, 0.0,
    {{{2, 2, 0}, {4, 4, 6}}});
  EXPECT_FALSE(planner.segment_free({1, 3, 2}, {3, 1, 2}));
  const auto path = planner.simplify(planner.plan({1, 3, 2}, {5, 3, 2}).points);
  ASSERT_GT(path.size(), 2u);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(planner.segment_free(path[i - 1], path[i]));
  }
}

TEST(AStar3D, PreviousNineAnchorRouteRemainsValid)
{
  const auto boxes = uav_planning::load_buildings(TEST_WORLD_FILE);
  ASSERT_EQ(boxes.size(), 11u);
  const AStar3D planner({-26, -20, 1}, {26, 20, 18}, 0.5, 1.5, boxes);
  const std::vector<Point3D> anchors{
    {-24, -18, 16}, {-6, 15.959, 16}, {-6, 15.959, 10}, {-6, 15.959, 16},
    {16, 17, 16}, {16, 17, 14}, {16, 17, 16}, {24, 4, 16}, {24, 4, 7}};
  Point3D start{-24, -18, 2.227};
  for (const auto & anchor : anchors) {
    const auto search = planner.plan(start, anchor);
    EXPECT_GT(search.expanded, 0u);
    const auto path = planner.simplify(search.points);
    ASSERT_EQ(path.size(), 2u);
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), anchor);
    start = anchor;
  }
}

TEST(AStar3D, ThreeMissionTargetsAllowCollisionFreeDiagonalLegs)
{
  const AStar3D planner(
    {-26, -20, 1}, {26, 20, 18}, 0.5, 1.5,
    uav_planning::load_buildings(TEST_WORLD_FILE));
  const std::vector<Point3D> targets{{-6, 15.959, 10}, {16, 17, 14}, {24, 4, 7}};
  Point3D start{-24, -18, 2.227};
  EXPECT_FALSE(planner.segment_free(start, targets.front()));  // 不能直接斜穿B1。
  std::size_t diagonal_segments = 0;
  double length = 0.0;
  for (const auto & target : targets) {
    const auto path = planner.simplify(planner.plan(start, target).points);
    ASSERT_GE(path.size(), 2u);
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), target);  // 每段必须保留对应P2/P3/P4访问点。
    for (std::size_t i = 1; i < path.size(); ++i) {
      EXPECT_TRUE(planner.segment_free(path[i - 1], path[i]));
      const double xy = std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
      const double z = std::abs(path[i][2] - path[i - 1][2]);
      diagonal_segments += xy > 1e-6 && z > 1e-6;
      length += std::hypot(xy, z);
    }
    start = target;
  }
  EXPECT_GT(diagonal_segments, 0u);
  EXPECT_LT(length, 116.31);  // 当前地图下，解除16米中转约束后应短于旧路线。
}
}  // namespace
