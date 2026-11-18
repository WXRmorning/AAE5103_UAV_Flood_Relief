#!/usr/bin/env bash
# Copyright 2026 uav_planning contributors
# SPDX-License-Identifier: Apache-2.0

# 使用官方完整启动流程，不复制/修改 PX4 源码中的 rcS。
. etc/init.d-posix/rcS

# 本地 SITL 不连接地面站。X500 机型会 set-default NAV_DLL_ACT=2，
# 因此在官方机型加载完成后明确设为0，避免前面的环境参数被默认值覆盖。
# 其余需要在模块启动前生效的参数（例如 DDS 时钟）仍由 launch 的环境变量提供。
param set NAV_DLL_ACT 0
