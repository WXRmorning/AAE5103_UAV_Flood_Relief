// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

/*
 * 航点飞行：PX4反馈 -> 地面锁定参考 -> 接收map路径 -> 起飞 -> 逐点移动 -> 保持/降落。
 * 路径由外部提供，本节点只做分段直线执行，不做A*或建筑碰撞检测。
 * 手动路径仍为单次演示；派发架次由可执行文件选择返航降落或悬停。
 * 异常退出Offboard后不抢回控制。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "goto_point_runner.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uav_planning/frame_alignment.hpp"
#include "uav_planning/msg/sortie.hpp"
#include "uav_planning/msg/sortie_state.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using uav_planning::FrameAlignment;
using uav_planning::msg::Sortie;
using uav_planning::msg::SortieState;
using Position = FrameAlignment::Position;

namespace
{
std::string versioned_topic(const std::string & name, uint32_t version)
{
  return version == 0 ? name : name + "_v" + std::to_string(version);
}

double distance(const Position & first, const Position & second)
{
  return std::hypot(first[0] - second[0], first[1] - second[1], first[2] - second[2]);
}
}  // namespace

class GotoPointNode : public rclcpp::Node
{
public:
  explicit GotoPointNode(bool return_hover)
  : Node("goto_point_node"), return_hover_(return_hover)
  {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    // 三个三维参数共用一次入口检查；不引入通用配置管理器。
    const auto read_position = [this, &read_only](const std::string & name) {
        const auto values = declare_parameter<std::vector<double>>(
          name, std::vector<double>{}, read_only);
        if (values.size() != 3 || !std::isfinite(values[0]) ||
          !std::isfinite(values[1]) || !std::isfinite(values[2]))
        {
          throw std::invalid_argument(name + " 必须在YAML中提供3个有限数值。");
        }
        return Position{values[0], values[1], values[2]};
      };
    map_reference_ = read_position("map_reference_position");
    area_min_ = read_position("flight_area_min");
    area_max_ = read_position("flight_area_max");
    const double rate = declare_parameter<double>("control_rate", 20.0, read_only);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 1.0, read_only);
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 2.0, read_only);
    max_speed_ = declare_parameter<double>("max_speed", 0.5, read_only);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 0.5, read_only);
    auto_land_ = declare_parameter<bool>("auto_land", false, read_only);
    // PX4 的 MAV_SYS_ID：S1 的 instance 0 为 1，S2 的 instance 1 为 2。
    const auto system_id = declare_parameter<int>("target_system", 1, read_only);
    if (system_id < 1 || system_id > 255) {
      throw std::invalid_argument("target_system 必须在 1 到 255 之间。");
    }
    target_system_ = static_cast<uint8_t>(system_id);
    for (double value :
      {rate, feedback_timeout_, takeoff_altitude_, max_speed_, max_acceleration_})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("频率、超时、起飞高度、参考速度和加速度必须为正有限数值。");
      }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (area_min_[axis] >= area_max_[axis]) {
        throw std::invalid_argument("flight_area_min 必须逐轴小于 flight_area_max。");
      }
    }
    Position takeoff = map_reference_;
    takeoff[2] += takeoff_altitude_;
    if (!inside_area(takeoff)) {
      throw std::invalid_argument("配置的起飞目标不在测试区域内，请检查参考点和起飞高度。");
    }

    pose_pub_ = create_publisher<PoseStamped>("/uav/estimated_pose", 10);
    if (return_hover_) {
      planning_start_pub_ = create_publisher<PoseStamped>("/uav/planning_start", 10);
    }
    execution_pub_ = create_publisher<SortieState>("/uav/execution_state", 10);
    mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    target_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    goal_sub_ = create_subscription<PoseStamped>(
      "/uav/goal", 10, std::bind(&GotoPointNode::on_goal, this, std::placeholders::_1));
    path_sub_ = create_subscription<Path>(
      "/uav/path", 10, [this](Path::ConstSharedPtr message) {on_path(*message);});
    sortie_sub_ = create_subscription<Sortie>(
      "/uav/sortie_path", 10, [this](Sortie::ConstSharedPtr message) {on_sortie(*message);});
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
    land_sub_ = create_subscription<VehicleLandDetected>(
      versioned_topic("/fmu/out/vehicle_land_detected", VehicleLandDetected::MESSAGE_VERSION),
      rclcpp::SensorDataQoS(),
      [this](VehicleLandDetected::SharedPtr message) {
        landed_ = message->landed;
        land_received_ = now();
        have_land_ = true;
      });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      versioned_topic("/fmu/out/vehicle_command_ack", VehicleCommandAck::MESSAGE_VERSION),
      rclcpp::SensorDataQoS(),
      [this](VehicleCommandAck::SharedPtr message) {
        if (message->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
        message->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM ||
        message->command == VehicleCommand::VEHICLE_CMD_NAV_LAND)
        {
          RCLCPP_INFO(
            get_logger(), "命令回执：command=%u, result=%u；最终以状态反馈为准。",
            message->command, message->result);
        }
      });
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      std::bind(&GotoPointNode::update, this));
    RCLCPP_INFO(
      get_logger(), "等待地面定位与map路径；派发架次返航方式：%s。",
      return_hover_ ? "悬停" : "降落");
  }

private:
  enum class Stage {Waiting, Takeoff, Moving, Hold, Landing};
  // 只保留当前演示必需的常量，不把所有到达阈值/阶段时间都做成参数。
  static constexpr double kWarmup = 2.0, kRequestInterval = 1.0, kRequestTimeout = 60.0;
  static constexpr double kTakeoffTimeout = 45.0, kTakeoffStableSeconds = 2.0;
  static constexpr double kWaypointStableSeconds = 0.5;
  static constexpr double kHorizontalTolerance = 0.35, kVerticalTolerance = 0.25;
  static constexpr double kArrivalSpeed = 0.25, kMaxTrackingError = 0.75;
  static constexpr double kLiftoffStep = 0.5;  // m；先给PX4起转/离地留出跟随时间。
  static constexpr double kLandModeTimeout = 5.0, kLandingTimeout = 60.0;
  static constexpr double kHomeTolerance = 0.5;  // m；停机位验收，中心间距为2米。
  static constexpr double kStatePeriod = 0.25;    // 仿真秒；持续报告可派单/执行状态。
  static constexpr double kHoverLandFeedbackMargin = 0.25;  // 落地检测为1 Hz，留传输余量。
  static constexpr const char * kHoverSuccess = "已返航并稳定悬停，保持Offboard。";

  bool inside_area(const Position & point) const
  {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (point[axis] < area_min_[axis] || point[axis] > area_max_[axis]) {
        return false;
      }
    }
    return true;
  }

  void on_goal(PoseStamped::SharedPtr message)
  {
    // 保留P1单点命令，但不再单独维护一套飞行逻辑；先接收到的任务生效。
    Path path;
    path.header = message->header;
    path.poses.push_back(*message);
    on_path(path);
  }

  void on_path(const Path & message)
  {
    if (stopped_ || !waypoints_.empty() || (return_hover_ && active_)) {
      RCLCPP_WARN(get_logger(), "飞行节点不可接收手动路径；返航悬停时只接受派发架次。");
      return;
    }
    if (accept_path(message)) {
      publish_state(SortieState::BUSY, 0);
    }
  }

  void on_sortie(const Sortie & message)
  {
    // 同一架次的重复消息不能覆盖当前路径或再次触发起飞。
    if (message.task_id <= last_completed_task_ || message.task_id == task_id_) {
      return;
    }
    const auto time = now();
    if (stopped_ || !waypoints_.empty() || !(auto_land_ || return_hover_) ||
      !alignment_.is_initialized() || !valid_position(time) || position_reset() ||
      !(active_ ? hover_ready(time, map_pose()) :
      landed_disarmed(time) && home_xy(map_pose())))
    {
      publish_state(SortieState::FAILED, message.task_id, "执行节点未在停机位就绪。");
      return;
    }
    if (message.path.poses.empty()) {
      publish_state(SortieState::FAILED, message.task_id, "执行路径没有航点。");
      return;
    }
    const auto & end = message.path.poses.back().pose.position;
    if (!home_xy({end.x, end.y, end.z}) || !accept_path(message.path)) {
      publish_state(SortieState::FAILED, message.task_id, "执行路径无效或末点不是本机停机位。");
      return;
    }
    task_id_ = message.task_id;
    publish_state(SortieState::BUSY, task_id_);
  }

  bool accept_path(const Path & message)
  {
    if (message.header.frame_id != "map" || message.poses.empty()) {
      RCLCPP_WARN(get_logger(), "拒绝路径：header.frame_id必须为map，且至少包含一个航点。");
      return false;
    }
    // 整条路径检查通过后才接收，避免后面的坏点让前半条路径先触发起飞。
    std::vector<Position> points;
    for (const auto & pose : message.poses) {
      const auto & point = pose.pose.position;
      const Position position{point.x, point.y, point.z};
      if ((!pose.header.frame_id.empty() && pose.header.frame_id != "map") ||
        !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !inside_area(position))
      {
        RCLCPP_WARN(get_logger(), "拒绝整条路径：航点需要map坐标、有限数值，且位于YAML区域内。");
        return false;
      }
      points.push_back(position);  // 子header省略时，继承整条Path的map坐标系。
    }
    waypoints_ = std::move(points);
    RCLCPP_INFO(
      get_logger(), "接收map路径：%zu个航点，准备自动起飞；只在最后一个点保持或降落。",
      waypoints_.size());
    return true;
  }

  void publish_state(uint8_t state, uint64_t task_id, const char * detail = "")
  {
    SortieState message;
    message.task_id = task_id;
    message.state = state;
    message.detail = detail;
    execution_pub_->publish(message);
    last_state_sent_ = now();
  }

  bool landed_disarmed(const rclcpp::Time & time) const
  {
    return have_status_ && have_land_ && landed_ &&
           status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED &&
           (time - status_received_).seconds() <= feedback_timeout_ &&
           (time - land_received_).seconds() <= feedback_timeout_;
  }

  bool valid_position(const rclcpp::Time & time) const
  {
    return have_position_ && have_status_ &&
           (time - position_received_).seconds() <= feedback_timeout_ &&
           (time - status_received_).seconds() <= feedback_timeout_ &&
           position_.xy_valid && position_.z_valid &&
           position_.v_xy_valid && position_.v_z_valid &&
           std::isfinite(position_.x) && std::isfinite(position_.y) &&
           std::isfinite(position_.z) && std::isfinite(position_.heading) &&
           std::isfinite(position_.vx) && std::isfinite(position_.vy) &&
           std::isfinite(position_.vz);
  }

  bool position_reset() const
  {
    return position_.xy_reset_counter != reset_counts_[0] ||
           position_.z_reset_counter != reset_counts_[1];
  }

  Position map_pose() const
  {
    return alignment_.ned_to_map({position_.x, position_.y, position_.z});
  }

  // 落地由 PX4 确认；停机位只比较水平坐标，避免气压高度偏差把已返航误判为失败。
  bool home_xy(const Position & map) const
  {
    return std::hypot(map[0] - map_reference_[0], map[1] - map_reference_[1]) <= kHomeTolerance;
  }

  bool hover_feedback(const rclcpp::Time & time) const
  {
    return !landed_ &&
           (time - land_received_).seconds() <= feedback_timeout_ + kHoverLandFeedbackMargin;
  }

  bool hover_ready(const rclcpp::Time & time, const Position & map) const
  {
    return status_.arming_state == VehicleStatus::ARMING_STATE_ARMED &&
           status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
           hover_feedback(time) && home_xy(map) &&
           distance(map, reference_map_) <= kMaxTrackingError;
  }

  Position takeoff_from(Position map) const
  {
    map[2] += takeoff_altitude_;
    return map;
  }

  void send_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand message{};
    message.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    message.command = command;
    message.param1 = param1;
    message.param2 = param2;
    message.param5 = message.param6 = message.param7 = std::numeric_limits<float>::quiet_NaN();
    message.target_system = target_system_;
    message.target_component = 1;
    message.source_system = message.source_component = 1;
    message.from_external = true;
    command_pub_->publish(message);
  }

  void stop(const char * reason, bool request_land = false)
  {
    stopped_ = true;
    const auto failed_task = return_hover_ && active_ && task_id_ == 0 ?
      last_completed_task_ : task_id_;
    publish_state(SortieState::FAILED, failed_task, reason);
    RCLCPP_ERROR(get_logger(), "%s 停止心跳和目标发布；检查后重启仿真场景和节点。", reason);
    if (request_land && status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
      send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }
    // 不自动恢复、不重新锁定原点；无法送达降落命令时，由PX4失联策略接管。
  }

  void publish_target()
  {
    OffboardControlMode mode{};
    mode.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    mode.position = true;
    mode_pub_->publish(mode);

    const auto ned = alignment_.map_to_ned(reference_map_);
    TrajectorySetpoint target{};
    target.timestamp = mode.timestamp;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      target.position[axis] = static_cast<float>(ned[axis]);
    }
    target.yaw = target_yaw_;
    const float unused = std::numeric_limits<float>::quiet_NaN();
    // PX4位置控制允许同时提供速度前馈，减少1m/s移动时纯位置跟踪的滞后。
    // ENU速度->NED只换轴和Z符号，不叠加位置参考点偏移。
    target.velocity = {
      static_cast<float>(reference_velocity_map_[1]),
      static_cast<float>(reference_velocity_map_[0]),
      static_cast<float>(-reference_velocity_map_[2])};
    target.acceleration.fill(unused);
    target.jerk.fill(unused);
    target.yawspeed = unused;  // 首版只发位置，不叠加不一致的速度/加速度前馈。
    target_pub_->publish(target);
  }

  void publish_pose(rclcpp::Publisher<PoseStamped> & publisher, const Position & map)
  {
    PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";
    pose.pose.position.x = map[0];
    pose.pose.position.y = map[1];
    pose.pose.position.z = map[2];
    // 只表示ENU航向，roll/pitch按0；不是完整的飞行姿态反馈。
    constexpr double kHalfPi = 1.5707963267948966;
    const double yaw = kHalfPi - position_.heading;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    publisher.publish(pose);
  }

  void publish_available(const Position & planning_start)
  {
    if (planning_start_pub_) {
      publish_pose(*planning_start_pub_, planning_start);
    }
    // 保留最后一单的完成状态作为就绪心跳，直到接收新单。
    publish_state(
      last_completed_task_ == 0 ? SortieState::READY : SortieState::SUCCEEDED,
      last_completed_task_, return_hover_ && active_ ? kHoverSuccess : "");
  }

  void finish_sortie()
  {
    last_completed_task_ = task_id_;
    task_id_ = 0;
    waypoints_.clear();
    waypoint_index_ = 0;
    active_ = return_hover_;
    takeoff_final_segment_ = false;
    landing_confirmed_ = false;
    arrival_since_.reset();
    stage_ = Stage::Waiting;
    // 悬停继续沿用返航航点作为控制参考，不从含噪声的当前位置跳转。
    reference_velocity_map_.fill(0.0);
    publish_available(reference_map_);
  }

  void update_landing(const rclcpp::Time & time, bool valid)
  {
    // 已发送降落指令或已切换降落模式不代表已完成。
    if ((time - status_received_).seconds() > feedback_timeout_ ||
      (time - landing_since_).seconds() > kLandingTimeout)
    {
      stop("降落状态反馈超时或60秒内未完成，请检查PX4；不重新接管飞行。");
      return;
    }
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    if (landed_disarmed(time)) {
      if (task_id_ == 0) {
        stopped_ = true;  // 手动目标/路径沿用原来的单次演示。
      } else {
        if (position_reset()) {
          stop("降落期间PX4位置原点重置，不能确认返航停机位。");
          return;
        }
        if (!valid) {
          return;  // 等待新鲜位置反馈，超时由上面的降落计时处理。
        }
        if (!home_xy(map_pose())) {
          stop("PX4已落地上锁，但实际位置不在本机停机位。");
          return;
        }
        finish_sortie();
      }
      RCLCPP_INFO(get_logger(), "自主降落完成：PX4已确认落地并上锁，任务结束。");
      return;
    }
    // 上锁和落地反馈分开到达，留在此阶段等待二者均确认。
    if (status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
      return;
    }
    if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
      if (!landing_confirmed_) {
        landing_confirmed_ = true;
        RCLCPP_INFO(get_logger(), "PX4已确认降落模式，停止Offboard目标/心跳，由飞控落地。");
      }
      return;
    }
    if (landing_confirmed_ || !armed ||
      status_.nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      stop("降落已被其他模式接管，交还飞控，不抢回Offboard。");
      return;
    }
    if ((time - landing_since_).seconds() > kLandModeTimeout) {
      stop("PX4未在5秒内确认降落模式，停止外部控制，请检查飞控。");
      return;
    }
    // 只在等待降落模式确认的短窗口保持末端目标；不会重发解锁/模式请求。
    publish_target();
  }

  void begin_segment(const Position & goal, Stage stage, const rclcpp::Time & time)
  {
    segment_start_ = reference_map_;
    reference_velocity_map_.fill(0.0);
    segment_goal_ = goal;
    segment_length_ = distance(segment_start_, segment_goal_);
    // 距离短时为三角速度曲线；距离长时为“加速-匀速-减速”的梯形曲线。
    peak_speed_ = std::min(max_speed_, std::sqrt(max_acceleration_ * segment_length_));
    acceleration_time_ = peak_speed_ / max_acceleration_;
    cruise_time_ = peak_speed_ > 0.0 ?
      std::max(0.0, (segment_length_ - peak_speed_ * acceleration_time_) / peak_speed_) : 0.0;
    segment_duration_ = 2.0 * acceleration_time_ + cruise_time_;
    segment_since_ = time;
    stage_ = stage;
    arrival_since_.reset();
  }

  void begin_waypoint(const rclcpp::Time & time)
  {
    const auto & point = waypoints_[waypoint_index_];
    begin_segment(point, Stage::Moving, time);
    RCLCPP_INFO(
      get_logger(), "开始航点%zu/%zu：(%.3f, %.3f, %.3f) m。",
      waypoint_index_ + 1, waypoints_.size(), point[0], point[1], point[2]);
  }

  void advance_reference(double elapsed)
  {
    if (elapsed >= segment_duration_ || segment_length_ < 1e-9) {
      reference_map_ = segment_goal_;
      reference_velocity_map_.fill(0.0);
      return;
    }
    double progress, speed;
    if (elapsed < acceleration_time_) {
      progress = 0.5 * max_acceleration_ * elapsed * elapsed;
      speed = max_acceleration_ * elapsed;
    } else if (elapsed < acceleration_time_ + cruise_time_) {
      progress = 0.5 * peak_speed_ * acceleration_time_ +
        peak_speed_ * (elapsed - acceleration_time_);
      speed = peak_speed_;
    } else {
      const double remaining_time = segment_duration_ - elapsed;
      progress = segment_length_ - 0.5 * max_acceleration_ * remaining_time * remaining_time;
      speed = max_acceleration_ * remaining_time;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      reference_map_[axis] = segment_start_[axis] +
        (segment_goal_[axis] - segment_start_[axis]) * progress / segment_length_;
      reference_velocity_map_[axis] =
        (segment_goal_[axis] - segment_start_[axis]) * speed / segment_length_;
    }
  }

  void update()
  {
    if (stopped_ || now().nanoseconds() <= 0 ||
      !have_position_ || !have_status_ || !have_land_)
    {
      return;
    }
    const auto time = now();
    const bool valid = valid_position(time);
    const bool send_state = (time - last_state_sent_).seconds() >= kStatePeriod;
    if (send_state && !waypoints_.empty()) {
      publish_state(SortieState::BUSY, task_id_);
    }
    if (stage_ == Stage::Landing) {
      // PX4接管降落后，位置失效/原点变化不能使节点重新发起飞行；仍可监测落地状态。
      if (valid && !position_reset()) {
        publish_pose(*pose_pub_, map_pose());
      }
      update_landing(time, valid);
      return;
    }
    if (!valid) {
      if (alignment_.is_initialized()) {
        stop("PX4定位/速度无效或反馈过期。", true);
      }
      return;
    }
    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard = status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (!alignment_.is_initialized()) {
      if ((time - land_received_).seconds() > feedback_timeout_) {
        return;
      }
      if (armed || !landed_) {
        stop("只能从未解锁且已落地的固定起飞点建立参考。");
        return;
      }
      alignment_.initialize(map_reference_, {position_.x, position_.y, position_.z});
      reset_counts_ = {position_.xy_reset_counter, position_.z_reset_counter};
      RCLCPP_INFO(
        get_logger(), "锁定NED参考：(%.3f, %.3f, %.3f) m。",
        position_.x, position_.y, position_.z);
    }
    if (position_reset()) {
      stop("PX4位置发生重置，原坐标对应关系已失效。", true);
      return;
    }

    const Position map = map_pose();
    publish_pose(*pose_pub_, map);
    if (active_ && (!armed || !offboard)) {
      stop("PX4已退出Offboard或解除锁定，交还飞控。");
      return;
    }
    if (stage_ == Stage::Waiting) {
      if (active_) {
        if (!hover_feedback(time) || distance(map, reference_map_) > kMaxTrackingError) {
          stop("返航悬停反馈失效或跟踪误差超过0.75米，终止控制并请求降落。", true);
          return;
        }
        if (waypoints_.empty()) {
          if (send_state && home_xy(map)) {
            publish_available(reference_map_);
          }
          publish_target();
          return;
        }
        // 已在空中，沿上一个架次的悬停参考直接出发，不重复起飞或请求Offboard。
        begin_waypoint(time);
      } else {
        if (waypoints_.empty()) {
          if (send_state && (auto_land_ || return_hover_) && landed_disarmed(time) &&
            home_xy(map))
          {
            publish_available(takeoff_from(map));
          }
          return;  // 地面未接单时不发送Offboard心跳或解锁命令。
        }
        if (!landed_disarmed(time) || (task_id_ != 0 && !home_xy(map))) {
          stop("任务开始时必须仍处于未解锁的地面状态。");
          return;
        }
        reference_map_ = map;
        takeoff_goal_ = takeoff_from(map);
        if (!inside_area(takeoff_goal_)) {
          stop("实际起飞目标超出测试区域，拒绝执行。");
          return;
        }
        target_yaw_ = position_.heading;
        handshake_since_ = last_request_ = time;
        begin_segment(takeoff_goal_, Stage::Takeoff, time);
        RCLCPP_INFO(
          get_logger(), "先预发2秒心跳，再请求Offboard/解锁；相对起飞 %.2f m。",
          takeoff_altitude_);
      }
    }
    if (!active_) {
      if (armed && !offboard) {
        stop("无人机已被其他模式解锁，拒绝接管。");
        return;
      }
      if (armed && offboard) {
        active_ = true;
        // PX4起转和升力建立期间先平滑引导0.5米并等待，避免参考在地面越走越远。
        Position liftoff_goal = reference_map_;
        liftoff_goal[2] += std::min(kLiftoffStep, takeoff_altitude_);
        begin_segment(liftoff_goal, Stage::Takeoff, time);  // 从实际解锁后计时。
        RCLCPP_INFO(get_logger(), "PX4已确认Offboard和解锁，开始垂直起飞。");
      } else {
        if ((time - handshake_since_).seconds() > kRequestTimeout) {
          stop("请求Offboard/解锁超时，请检查健康检查日志。");
          return;
        }
        publish_target();  // 握手时保持地面参考，不提前推进起飞轨迹。
        if ((time - handshake_since_).seconds() >= kWarmup &&
          (time - last_request_).seconds() >= kRequestInterval)
        {
          if (!offboard) {
            send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
          } else {
            send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
          }
          last_request_ = time;
        }
        return;
      }
    }

    // 名义参考受速度/加速度限制；实际机体超差时中止，不盲目按时间向前推进。
    if (distance(map, reference_map_) > kMaxTrackingError) {
      stop("跟踪误差超过0.75米，终止任务并请求降落。", true);
      return;
    }
    if (stage_ != Stage::Hold) {
      const double elapsed = (time - segment_since_).seconds();
      const double timeout = stage_ == Stage::Takeoff ?
        kTakeoffTimeout : 2.0 * segment_duration_ + 15.0;
      if (elapsed > timeout) {
        stop("当前飞行阶段到达超时。", true);
        return;
      }
      advance_reference(elapsed);
      const bool arrived = elapsed >= segment_duration_ &&
        (stage_ != Stage::Takeoff || !landed_) &&
        std::hypot(map[0] - segment_goal_[0], map[1] - segment_goal_[1]) < kHorizontalTolerance &&
        std::abs(map[2] - segment_goal_[2]) < kVerticalTolerance &&
        std::hypot(position_.vx, position_.vy, position_.vz) < kArrivalSpeed;
      if (arrived) {
        if (!arrival_since_) {
          arrival_since_ = time;
        }
        const double stable_seconds = stage_ == Stage::Takeoff ?
          kTakeoffStableSeconds : kWaypointStableSeconds;
        if ((time - *arrival_since_).seconds() >= stable_seconds) {
          if (stage_ == Stage::Takeoff) {
            if (!takeoff_final_segment_) {
              takeoff_final_segment_ = true;
              begin_segment(takeoff_goal_, Stage::Takeoff, time);
              RCLCPP_INFO(get_logger(), "离地引导已稳定，继续升至指定起飞高度。");
            } else {
              begin_waypoint(time);
            }
          } else {
            RCLCPP_INFO(
              get_logger(), "航点%zu/%zu已到达并稳定%.1f秒。",
              waypoint_index_ + 1, waypoints_.size(), stable_seconds);
            arrival_since_.reset();
            if (waypoint_index_ + 1 < waypoints_.size()) {
              ++waypoint_index_;
              begin_waypoint(time);  // 中间点不降落，保持同一次Offboard任务。
            } else if (return_hover_ && task_id_ != 0) {
              if (!hover_feedback(time) || !home_xy(map)) {
                stop("无法确认已在本机停机位上方悬停，终止任务并请求降落。", true);
                return;
              }
              finish_sortie();
              RCLCPP_INFO(get_logger(), "%s", kHoverSuccess);
            } else if (auto_land_) {
              stage_ = Stage::Landing;
              landing_since_ = time;
              send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
              RCLCPP_INFO(get_logger(), "整条路径已完成，请求PX4在最后航点自主降落。");
            } else {
              stage_ = Stage::Hold;
              RCLCPP_INFO(get_logger(), "整条路径已完成，持续悬停；等待操作者降落。");
            }
          }
        }
      } else {
        arrival_since_.reset();
      }
    }
    publish_target();
  }

  FrameAlignment alignment_;
  Position map_reference_{}, area_min_{}, area_max_{};
  Position reference_map_{}, takeoff_goal_{}, segment_start_{}, segment_goal_{};
  Position reference_velocity_map_{};  // map/ENU速度前馈，单位m/s；保持时为0。
  std::vector<Position> waypoints_;
  std::size_t waypoint_index_{0};
  uint64_t task_id_{0}, last_completed_task_{0};  // 0 表示没有派发任务。
  Stage stage_{Stage::Waiting};
  double feedback_timeout_, takeoff_altitude_, max_speed_, max_acceleration_;
  double segment_length_{0.0}, peak_speed_{0.0}, acceleration_time_{0.0};
  double cruise_time_{0.0}, segment_duration_{0.0};
  float target_yaw_{0.0f};
  uint8_t target_system_{1};
  bool have_position_{false}, have_status_{false}, have_land_{false};
  bool landed_{false}, stopped_{false}, active_{false};
  bool takeoff_final_segment_{false};
  bool auto_land_{false}, landing_confirmed_{false};
  const bool return_hover_;
  std::array<uint8_t, 2> reset_counts_{};
  VehicleLocalPosition position_{};
  VehicleStatus status_{};
  rclcpp::Time position_received_{0, 0, RCL_ROS_TIME}, status_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time land_received_{0, 0, RCL_ROS_TIME}, handshake_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_request_{0, 0, RCL_ROS_TIME}, segment_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time landing_since_{0, 0, RCL_ROS_TIME}, last_state_sent_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> arrival_since_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_, planning_start_pub_;
  rclcpp::Publisher<SortieState>::SharedPtr execution_pub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr target_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<Sortie>::SharedPtr sortie_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int run_goto_point(int argc, char ** argv, bool return_hover)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<GotoPointNode> node;
  try {
    node = std::make_shared<GotoPointNode>(return_hover);
  } catch (const std::invalid_argument & error) {
    std::cerr << "goto_point_node: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
