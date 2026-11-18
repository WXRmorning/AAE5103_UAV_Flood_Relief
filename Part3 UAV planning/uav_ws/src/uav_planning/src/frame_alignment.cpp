// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <stdexcept>

#include "uav_planning/frame_alignment.hpp"

namespace uav_planning
{
namespace
{

// 同一检查复用于参考点和待转换位置，拒绝 NaN/Inf，不用零坐标掩盖错误
void require_finite(const FrameAlignment::Position & position)
{
  for (double value : position) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("位置坐标必须是有限数值，不能包含 NaN 或 Inf。");
    }
  }
}

}  // namespace

void FrameAlignment::initialize(
  const Position & map_reference, const Position & ned_reference)
{
  require_finite(map_reference);
  require_finite(ned_reference);
  map_reference_ = map_reference;
  ned_reference_ = ned_reference;
  initialized_ = true;
}

FrameAlignment::Position FrameAlignment::map_to_ned(const Position & map_position) const
{
  if (!initialized_) {
    throw std::logic_error("请先使用地图/PX4 对应参考点初始化 FrameAlignment。");
  }
  require_finite(map_position);

  // 先求相对参考点的 ENU 位移，再交换东/北、反转高度方向，最后加 NED 参考点。
  return {
    ned_reference_[0] + (map_position[1] - map_reference_[1]),
    ned_reference_[1] + (map_position[0] - map_reference_[0]),
    ned_reference_[2] - (map_position[2] - map_reference_[2])
  };
}

FrameAlignment::Position FrameAlignment::ned_to_map(const Position & ned_position) const
{
  if (!initialized_) {
    throw std::logic_error("请先使用地图/PX4 对应参考点初始化 FrameAlignment。");
  }
  require_finite(ned_position);

  // ENU/NED 的轴交换及 Z 反向是互逆的；参考点平移也要随方向交换。
  return {
    map_reference_[0] + (ned_position[1] - ned_reference_[1]),
    map_reference_[1] + (ned_position[0] - ned_reference_[0]),
    map_reference_[2] - (ned_position[2] - ned_reference_[2])
  };
}

}  // namespace uav_planning
