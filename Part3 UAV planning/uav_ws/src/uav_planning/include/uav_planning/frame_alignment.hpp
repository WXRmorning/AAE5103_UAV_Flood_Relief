// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef UAV_PLANNING__FRAME_ALIGNMENT_HPP_
#define UAV_PLANNING__FRAME_ALIGNMENT_HPP_

#include <array>

namespace uav_planning
{

// 只转换“位置”，单位米；不处理姿态、机体坐标或额外地图偏航。
// map 为 ENU（东、北、上），PX4 本地位置为 NED（北、东、下）。
class FrameAlignment
{
public:
  using Position = std::array<double, 3>;

  // 两个参考位置必须对应同一时刻、同一个机体 base_link 原点。
  // T0 的 map 参考点已考虑模型偏移，本模块不再额外叠加 0.24 m。
  // 节点负责在地面锁定一次有效参考，本模块不订阅消息或判断飞行状态。
  void initialize(const Position & map_reference, const Position & ned_reference);

  bool is_initialized() const {return initialized_;}

  // 地图绝对位置 -> PX4 本地 NED 绝对位置，不是机体相对位移。
  Position map_to_ned(const Position & map_position) const;

  // PX4 本地 NED 位置 -> 地图绝对位置，用于反馈和目标误差计算。
  Position ned_to_map(const Position & ned_position) const;

private:
  Position map_reference_{};
  Position ned_reference_{};
  bool initialized_{false};
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__FRAME_ALIGNMENT_HPP_
