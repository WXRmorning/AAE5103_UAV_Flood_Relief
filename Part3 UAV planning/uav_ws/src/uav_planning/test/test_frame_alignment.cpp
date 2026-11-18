// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "uav_planning/frame_alignment.hpp"

namespace
{

using uav_planning::FrameAlignment;
using Position = FrameAlignment::Position;

// 两个参考点都刻意取非零值，避免漏写平移时测试仍然通过。
// NED 数值只是数学测试数据，不是本次/下次仿真的固定配置。
constexpr Position kMapReference{-24.0, -18.0, 0.227};
constexpr Position kNedReference{1.0, 2.0, -0.3};

void expect_position_near(const Position & actual, const Position & expected)
{
  for (std::size_t axis = 0; axis < actual.size(); ++axis) {
    EXPECT_NEAR(actual[axis], expected[axis], 1e-12) << "axis=" << axis;
  }
}

TEST(FrameAlignment, RequiresInitialization)
{
  FrameAlignment alignment;
  EXPECT_FALSE(alignment.is_initialized());
  EXPECT_THROW(alignment.map_to_ned(kMapReference), std::logic_error);
  EXPECT_THROW(alignment.ned_to_map(kNedReference), std::logic_error);
}

TEST(FrameAlignment, ReferenceMapsInBothDirections)
{
  FrameAlignment alignment;
  alignment.initialize(kMapReference, kNedReference);
  EXPECT_TRUE(alignment.is_initialized());
  expect_position_near(alignment.map_to_ned(kMapReference), kNedReference);
  expect_position_near(alignment.ned_to_map(kNedReference), kMapReference);
}

TEST(FrameAlignment, MapEastIsNedEast)
{
  FrameAlignment alignment;
  alignment.initialize(kMapReference, kNedReference);
  for (double offset : {-2.0, 2.0}) {
    Position map = kMapReference;
    Position ned = kNedReference;
    map[0] += offset;
    ned[1] += offset;
    expect_position_near(alignment.map_to_ned(map), ned);
    expect_position_near(alignment.ned_to_map(ned), map);
  }
}

TEST(FrameAlignment, MapNorthIsNedNorth)
{
  FrameAlignment alignment;
  alignment.initialize(kMapReference, kNedReference);
  for (double offset : {-2.0, 2.0}) {
    Position map = kMapReference;
    Position ned = kNedReference;
    map[1] += offset;
    ned[0] += offset;
    expect_position_near(alignment.map_to_ned(map), ned);
    expect_position_near(alignment.ned_to_map(ned), map);
  }
}

TEST(FrameAlignment, MapUpIsNedNegativeDown)
{
  FrameAlignment alignment;
  alignment.initialize(kMapReference, kNedReference);
  for (double offset : {-2.0, 2.0}) {
    Position map = kMapReference;
    Position ned = kNedReference;
    map[2] += offset;
    ned[2] -= offset;
    expect_position_near(alignment.map_to_ned(map), ned);
    expect_position_near(alignment.ned_to_map(ned), map);
  }
}

TEST(FrameAlignment, KnownGoalUsesOffsetsInAllAxes)
{
  FrameAlignment alignment;
  alignment.initialize(kMapReference, kNedReference);
  expect_position_near(alignment.map_to_ned({-21.0, -16.0, 2.0}), {3.0, 5.0, -2.073});
}

TEST(FrameAlignment, RoundTripForPositiveAndNegativePositions)
{
  FrameAlignment alignment;
  alignment.initialize(kMapReference, kNedReference);
  for (const Position & point : {Position{-21.0, -16.0, 2.0}, Position{-27.0, -20.0, -1.0},
      Position{0.0, 0.0, 0.0}, Position{3.0, 5.0, -2.073}})
  {
    expect_position_near(alignment.ned_to_map(alignment.map_to_ned(point)), point);
    expect_position_near(alignment.map_to_ned(alignment.ned_to_map(point)), point);
  }
}

TEST(FrameAlignment, RejectsNonFiniteReferences)
{
  FrameAlignment alignment;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(alignment.initialize({-24.0, -18.0, nan}, kNedReference), std::invalid_argument);
  EXPECT_THROW(alignment.initialize(kMapReference, {nan, 2.0, -0.3}), std::invalid_argument);
  EXPECT_FALSE(alignment.is_initialized());
}

TEST(FrameAlignment, RejectsNonFiniteInputs)
{
  FrameAlignment alignment;
  alignment.initialize(kMapReference, kNedReference);
  const double inf = std::numeric_limits<double>::infinity();
  for (double value : {std::numeric_limits<double>::quiet_NaN(), inf, -inf}) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      Position point = kMapReference;
      point[axis] = value;
      EXPECT_THROW(alignment.map_to_ned(point), std::invalid_argument);
      EXPECT_THROW(alignment.ned_to_map(point), std::invalid_argument);
    }
  }
}

}  // namespace
