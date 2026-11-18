// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

/*
 * PX4 Offboard 起飞/悬停，沿用 takeoff_hold_node 的文件名和可执行程序名。
 * 主线：保存 PX4 反馈 -> 锁定起点 -> 预发心跳 -> 请求模式/解锁 -> 发布位置目标。
 * PX4 负责位置、速度、姿态和电机控制，ROS 2 不再计算旧 X4 的 P 速度指令。
 * 位置使用 PX4 本地 NED（北、东、下），不能直接填 Gazebo 世界 ENU 坐标。
 * target_altitude 为相对起点上升高度；首次有效位置只锁定一次。
 * 只做单机仿真起飞/悬停，没有路径搜索、地图坐标转换或六态状态机。
 */

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace
{
// PX4 v1.16 中部分 DDS 话题带版本后缀，例如 vehicle_status_v1。
// 从消息定义自动取版本，避免把所有反馈话题都写成没有后缀的名字。
std::string versioned_topic(const std::string & name, uint32_t version)
{
  return version == 0 ? name : name + "_v" + std::to_string(version);
}
}  // namespace

class TakeoffHoldNode : public rclcpp::Node
{
public:
  TakeoffHoldNode()
  : Node("takeoff_hold_node")
  {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    altitude_ = declare_parameter<double>("target_altitude", 5.0, read_only);
    rate_ = declare_parameter<double>("control_rate", 20.0, read_only);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 1.0, read_only);
    auto_arm_ = declare_parameter<bool>("auto_arm", true, read_only);
    if (!std::isfinite(altitude_) || altitude_ < 0.5 || altitude_ > 30.0 ||
      !std::isfinite(rate_) || rate_ < 5.0 || rate_ > 100.0 ||
      !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "target_altitude must be [0.5,30], control_rate [5,100], feedback_timeout > 0");
    }

    mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    target_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    position_sub_ = create_subscription<VehicleLocalPosition>(
      versioned_topic("/fmu/out/vehicle_local_position", VehicleLocalPosition::MESSAGE_VERSION),
      rclcpp::SensorDataQoS(),
      [this](VehicleLocalPosition::SharedPtr message) {
        position_ = *message;
        position_received_ = now();
        have_position_ = true;
      });
    status_sub_ = create_subscription<VehicleStatus>(
      versioned_topic("/fmu/out/vehicle_status", VehicleStatus::MESSAGE_VERSION),
      rclcpp::SensorDataQoS(),
      [this](VehicleStatus::SharedPtr message) {
        status_ = *message;
        status_received_ = now();
        have_status_ = true;
      });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      versioned_topic("/fmu/out/vehicle_command_ack", VehicleCommandAck::MESSAGE_VERSION),
      rclcpp::SensorDataQoS(),
      [this](VehicleCommandAck::SharedPtr message) {
        if (message->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
        message->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
        {
          RCLCPP_INFO(
            get_logger(), "PX4 命令回执：command=%u, result=%u（0为接受，最终以状态反馈为准）。",
            message->command, message->result);
        }
      });

    // 使用 ROS 时钟：launch 配套 use_sim_time=true，Gazebo 暂停时控制定时器也暂停。
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_),
      std::bind(&TakeoffHoldNode::control_step, this));
    RCLCPP_INFO(
      get_logger(), "等待 PX4 有效本地位置与状态，计划相对上升 %.2f m。auto_arm=%s",
      altitude_, auto_arm_ ? "true" : "false");
  }

private:
  // 这些固定阈值只服务当前起飞/悬停演示，不扩展为大量 YAML 参数。
  static constexpr double kWarmup = 2.0, kRequestInterval = 1.0;
  static constexpr double kRequestTimeout = 60.0, kTakeoffTimeout = 45.0;

  uint64_t timestamp() const
  {
    return static_cast<uint64_t>(now().nanoseconds() / 1000);
  }

  void send_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand message{};
    message.timestamp = timestamp();
    message.command = command;
    message.param1 = param1;
    message.param2 = param2;
    message.param5 = std::numeric_limits<float>::quiet_NaN();
    message.param6 = std::numeric_limits<float>::quiet_NaN();
    message.param7 = std::numeric_limits<float>::quiet_NaN();
    message.target_system = 1;  // 本次固定为 PX4 instance 0，对应 MAV_SYS_ID=1。
    message.target_component = 1;
    message.source_system = 1;
    message.source_component = 1;
    message.from_external = true;
    command_pub_->publish(message);
  }

  void stop_control(const char * reason, bool request_land = false)
  {
    if (stopped_) {
      return;
    }
    stopped_ = true;
    RCLCPP_ERROR(get_logger(), "%s 停止心跳/目标发布，检查后重启节点。", reason);
    if (request_land && status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
      send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }
    // 不持续重发解锁或抢回 Offboard；失联时由 PX4 配置的 Offboard-loss 降落接管。
  }

  void publish_target()
  {
    OffboardControlMode mode{};
    mode.timestamp = timestamp();
    mode.position = true;  // 其余布尔项保持 false，使用 PX4 内部位置控制环。
    mode_pub_->publish(mode);

    const float unused = std::numeric_limits<float>::quiet_NaN();
    TrajectorySetpoint target{};
    target.timestamp = timestamp();
    target.position = target_;
    target.yaw = target_yaw_;
    // 不使用的速度/加速度设 NaN，避免无意中增加前馈控制。
    target.velocity.fill(unused);
    target.acceleration.fill(unused);
    target.jerk.fill(unused);
    target.yawspeed = unused;
    target_pub_->publish(target);
  }

  void control_step()
  {
    if (stopped_ || now().nanoseconds() <= 0) {
      return;
    }
    const auto time = now();
    if (!startup_since_) {
      startup_since_ = time;
    }
    const bool fresh = have_position_ && have_status_ &&
      (time - position_received_).seconds() <= feedback_timeout_ &&
      (time - status_received_).seconds() <= feedback_timeout_;
    const bool valid = fresh && position_.xy_valid && position_.z_valid &&
      position_.v_xy_valid && position_.v_z_valid &&
      std::isfinite(position_.x) && std::isfinite(position_.y) &&
      std::isfinite(position_.z) && std::isfinite(position_.heading);
    if (!valid) {
      if (active_ || (time - *startup_since_).seconds() > kRequestTimeout) {
        stop_control("PX4 定位/状态缺失、无效或过期。", true);
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "等待 PX4 定位有效及 DDS 反馈。");
      }
      return;
    }

    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard = status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (!target_initialized_) {
      if (armed) {
        stop_control("请从未解锁的仿真无人机启动本节点。");
        return;
      }
      start_z_ = position_.z;
      target_ = {position_.x, position_.y, position_.z - static_cast<float>(altitude_)};
      target_yaw_ = position_.heading;
      target_since_ = time;
      last_request_ = time;
      target_initialized_ = true;
      RCLCPP_INFO(
        get_logger(), "锁定 NED 目标：(%.2f, %.2f, %.2f)，先发送2秒心跳。",
        target_[0], target_[1], target_[2]);
    }

    // 操作者切去降落等模式后不再发布目标，也绝不自动重新解锁。
    if (active_ && (!armed || !offboard)) {
      stop_control("PX4 已退出 Offboard 或解除锁定，交还飞控。");
      return;
    }
    if (!active_ && armed && offboard) {
      active_ = true;
      active_since_ = time;
      RCLCPP_INFO(get_logger(), "PX4 已确认解锁且进入 Offboard，开始起飞。");
    }
    if (!active_ && (time - *target_since_).seconds() > kRequestTimeout) {
      stop_control("请求 Offboard/解锁超时，请检查 PX4 的健康检查日志。", true);
      return;
    }
    if (active_ && !arrived_ && (time - *active_since_).seconds() > kTakeoffTimeout) {
      stop_control("起飞到达超时。", true);
      return;
    }

    publish_target();
    if (auto_arm_ && !active_ && (time - *target_since_).seconds() >= kWarmup &&
      (time - *last_request_).seconds() >= kRequestInterval)
    {
      // 先请求模式；状态反馈确认 Offboard 后才请求解锁。失败时每秒重试，有总超时。
      if (!offboard) {
        send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      } else if (!armed) {
        send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      }
      last_request_ = time;
    }

    const double xy_error = std::hypot(position_.x - target_[0], position_.y - target_[1]);
    const double z_error = std::abs(position_.z - target_[2]);
    const double speed = std::hypot(std::hypot(position_.vx, position_.vy), position_.vz);
    if (active_ && xy_error < 0.35 && z_error < 0.25 && speed < 0.25) {
      if (!arrival_since_) {
        arrival_since_ = time;
      }
      if (!arrived_ && (time - *arrival_since_).seconds() >= 2.0) {
        arrived_ = true;
        RCLCPP_INFO(get_logger(), "已到达目标并稳定2秒，继续定点保持。");
      }
    } else {
      arrival_since_.reset();
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s | 相对高度 %.2f / %.2f m，水平误差 %.2f m。",
      active_ ? (arrived_ ? "悬停" : "起飞") : "等待模式/解锁",
      start_z_ - position_.z, altitude_, xy_error);
  }

  double altitude_, rate_, feedback_timeout_;
  bool auto_arm_;
  bool have_position_{false}, have_status_{false}, target_initialized_{false};
  bool active_{false}, arrived_{false}, stopped_{false};
  float start_z_{0.0f}, target_yaw_{0.0f};
  std::array<float, 3> target_{};
  VehicleLocalPosition position_{};
  VehicleStatus status_{};
  rclcpp::Time position_received_{0, 0, RCL_ROS_TIME}, status_received_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> startup_since_, target_since_, last_request_, active_since_,
    arrival_since_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr target_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<TakeoffHoldNode>());
  } catch (const std::exception & error) {
    std::cerr << "takeoff_hold_node: " << error.what() << '\n';
    result = 1;
  }
  // 退出后停止 Offboard 心跳，由 PX4 执行本 SITL 配置的失联降落。
  // 一键 launch 退出会一起结束仿真，不代表执行完物理降落。
  rclcpp::shutdown();
  return result;
}
