// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef UAV_PLANNING__GOTO_POINT_RUNNER_HPP_
#define UAV_PLANNING__GOTO_POINT_RUNNER_HPP_

// 两个执行入口共用同一套PX4反馈、航点跟踪与故障处理。
int run_goto_point(int argc, char ** argv, bool return_hover);

#endif  // UAV_PLANNING__GOTO_POINT_RUNNER_HPP_
