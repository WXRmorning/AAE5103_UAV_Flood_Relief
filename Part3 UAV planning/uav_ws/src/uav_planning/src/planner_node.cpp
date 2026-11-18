// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uav_planning/astar_3d.hpp"
#include "uav_planning/msg/sortie.hpp"
#include "uav_planning/msg/sortie_state.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using uav_planning::AStar3D;
using uav_planning::Point3D;
using uav_planning::msg::Sortie;
using uav_planning::msg::SortieState;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("planner_node")
  {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    const auto read_position = [this, &read_only](const std::string & name) {
        const auto value = declare_parameter<std::vector<double>>(
          name, std::vector<double>{}, read_only);
        if (value.size() != 3) {
          throw std::invalid_argument(name + "需要3个数值。");
        }
        return Point3D{value[0], value[1], value[2]};
      };
    const auto lower = read_position("flight_area_min");
    const auto upper = read_position("flight_area_max");
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 2.0, read_only);
    const bool use_planning_start = declare_parameter<bool>("use_planning_start", false, read_only);
    const double resolution = declare_parameter<double>("resolution", 0.5, read_only);
    const double clearance = declare_parameter<double>("clearance", 1.5, read_only);
    const auto file = declare_parameter<std::string>(
      "world_file", ament_index_cpp::get_package_share_directory("uav_planning") +
      "/worlds/flood_demo.sdf", read_only);
    if (!std::isfinite(takeoff_altitude_) || takeoff_altitude_ <= 0.0) {
      throw std::invalid_argument("takeoff_altitude必须为正有限数值。");
    }
    const auto buildings = uav_planning::load_buildings(file);
    planner_ = std::make_unique<AStar3D>(lower, upper, resolution, clearance, buildings);
    path_pub_ = create_publisher<Path>("/uav/path", 10);
    planned_path_pub_ =
      create_publisher<Path>("/uav/planned_path", rclcpp::QoS(1).transient_local());
    sortie_path_pub_ = create_publisher<Sortie>("/uav/sortie_path", 10);
    planning_state_pub_ = create_publisher<SortieState>("/uav/planning_state", 10);
    const auto start_topic = use_planning_start ? "/uav/planning_start" : "/uav/estimated_pose";
    pose_sub_ = create_subscription<PoseStamped>(
      start_topic, 10, [this, use_planning_start](PoseStamped::ConstSharedPtr pose) {
        const auto & p = pose->pose.position;
        if (pose->header.frame_id != "map" || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.z))
        {
          return;
        }
        // 悬停模式由控制节点提供空中参考点；地面模式从预计起飞高度开始规划。
        start_ = {p.x, p.y, p.z + (use_planning_start ? 0.0 : takeoff_altitude_)};
        pose_received_ = now();
        try_plan_manual();
        try_plan_sortie();
      });
    mission_sub_ = create_subscription<Path>(
      "/uav/mission", 10, [this](Path::ConstSharedPtr message) {on_mission(*message);});
    sortie_sub_ = create_subscription<Sortie>(
      "/uav/sortie", 10, [this](Sortie::ConstSharedPtr message) {on_sortie(*message);});
    pose_wait_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {try_plan_sortie();});
    RCLCPP_INFO(
      get_logger(), "读取%zu个建筑碰撞盒，%zu个栅格；resolution=%.2f m，clearance=%.2f m。",
      buildings.size(), planner_->cell_count(), resolution, clearance);
    RCLCPP_INFO(
      get_logger(), "等待%s的手动路线或%s的分发架次，逐段执行A*规划。",
      mission_sub_->get_topic_name(), sortie_sub_->get_topic_name());
  }

private:
  // 用仿真时间等待派单后的地图位置样本，与控制节点反馈超时量级一致。
  static constexpr double kPoseTimeout = 1.0;

  struct PendingSortie
  {
    uint64_t task_id;
    std::vector<Point3D> anchors;
    rclcpp::Time received;
  };

  std::vector<Point3D> read_anchors(const Path & message) const
  {
    if (message.header.frame_id != "map" || message.poses.empty()) {
      return {};
    }
    std::vector<Point3D> points;
    for (const auto & pose : message.poses) {
      const auto & p = pose.pose.position;
      const Point3D point{p.x, p.y, p.z};
      if ((!pose.header.frame_id.empty() && pose.header.frame_id != "map") ||
        !planner_->segment_free(point, point))
      {
        return {};
      }
      points.push_back(point);
    }
    return points;
  }

  void on_mission(const Path & message)
  {
    if (attempted_ || !manual_anchors_.empty() || last_sortie_id_ != 0) {
      RCLCPP_WARN(get_logger(), "当前已收到任务；手动路线每轮只执行一次。");
      return;
    }
    manual_anchors_ = read_anchors(message);
    if (manual_anchors_.empty()) {
      RCLCPP_WARN(get_logger(), "拒绝路线：需要有效、非空且可通行的map关键航点。");
      return;
    }
    try_plan_manual();
  }

  void try_plan_manual()
  {
    if (!pose_received_ || manual_anchors_.empty() || attempted_) {
      return;
    }
    attempted_ = true;
    const auto path = plan_path(manual_anchors_);
    if (!path.poses.empty()) {
      path_pub_->publish(path);
    }
  }

  void on_sortie(const Sortie & message)
  {
    // 一次启动中架次编号递增；重复消息不重新规划，也不覆盖原任务结果。
    if (message.task_id <= last_sortie_id_) {
      return;
    }
    last_sortie_id_ = message.task_id;
    if (pending_sortie_ || !manual_anchors_.empty()) {
      fail_sortie(message.task_id, "规划节点已有待处理架次或手动任务。");
      return;
    }
    auto anchors = read_anchors(message.path);
    if (anchors.empty()) {
      fail_sortie(message.task_id, "需要有效、非空且可通行的map关键航点。");
      return;
    }
    pending_sortie_ = PendingSortie{message.task_id, std::move(anchors), now()};
  }

  void try_plan_sortie()
  {
    if (!pending_sortie_) {
      return;
    }
    const auto time = now();
    const auto id = pending_sortie_->task_id;
    if ((time - pending_sortie_->received).seconds() > kPoseTimeout) {
      fail_sortie(id, "等待派单后的map位置超时。");
      pending_sortie_.reset();
      return;
    }
    if (!pose_received_ || *pose_received_ <= pending_sortie_->received ||
      (time - *pose_received_).seconds() > kPoseTimeout)
    {
      return;
    }

    auto path = plan_path(pending_sortie_->anchors);
    pending_sortie_.reset();
    if (path.poses.empty()) {
      fail_sortie(id, "A*未找到可通行的往返路径。");
      return;
    }
    Sortie result;
    result.task_id = id;
    result.path = std::move(path);
    sortie_path_pub_->publish(result);
  }

  void fail_sortie(uint64_t task_id, const std::string & detail)
  {
    SortieState result;
    result.task_id = task_id;
    result.state = SortieState::FAILED;
    result.detail = detail;
    planning_state_pub_->publish(result);
  }

  Path plan_path(const std::vector<Point3D> & anchors)
  {
    const auto began = std::chrono::steady_clock::now();
    Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    Point3D start = start_;
    std::size_t total_expanded = 0;
    for (std::size_t leg = 0; leg < anchors.size(); ++leg) {
      const auto search = planner_->plan(start, anchors[leg]);
      const auto simplified = planner_->simplify(search.points);
      if (simplified.empty()) {
        RCLCPP_ERROR(
          get_logger(), "第%zu段A*无可行路径；没有发布任何飞行路径。", leg + 1);
        return Path{};
      }
      total_expanded += search.expanded;
      // 只简化本段，绝不跨越/删除用户关键点；执行节点不需要再次访问段起点。
      for (std::size_t i = simplified.size() > 1 ? 1 : 0; i < simplified.size(); ++i) {
        PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = simplified[i][0];
        pose.pose.position.y = simplified[i][1];
        pose.pose.position.z = simplified[i][2];
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      RCLCPP_INFO(
        get_logger(), "A*第%zu/%zu段：扩展%zu格，原始%zu点，简化后%zu点。",
        leg + 1, anchors.size(), search.expanded, search.points.size(), simplified.size());
      start = anchors[leg];
    }
    const double milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - began).count();
    RCLCPP_INFO(
      get_logger(), "规划完成：保留%zu个关键点，输出%zu个执行航点，扩展%zu格，耗时%.1f ms。",
      anchors.size(), path.poses.size(), total_expanded, milliseconds);

    // The overlay needs the entire first leg. The controller already knows its start point.
    PoseStamped origin;
    origin.header = path.header;
    origin.pose.position.x = start_[0];
    origin.pose.position.y = start_[1];
    origin.pose.position.z = start_[2];
    origin.pose.orientation.w = 1.0;
    auto display_path = path;
    display_path.poses.insert(display_path.poses.begin(), origin);
    planned_path_pub_->publish(display_path);
    return path;
  }

  std::unique_ptr<AStar3D> planner_;
  Point3D start_{};
  std::vector<Point3D> manual_anchors_;
  std::optional<rclcpp::Time> pose_received_;
  std::optional<PendingSortie> pending_sortie_;
  uint64_t last_sortie_id_{0};
  double takeoff_altitude_;
  bool attempted_{false};
  rclcpp::Publisher<Path>::SharedPtr path_pub_, planned_path_pub_;
  rclcpp::Publisher<Sortie>::SharedPtr sortie_path_pub_;
  rclcpp::Publisher<SortieState>::SharedPtr planning_state_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<Path>::SharedPtr mission_sub_;
  rclcpp::Subscription<Sortie>::SharedPtr sortie_sub_;
  rclcpp::TimerBase::SharedPtr pose_wait_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PlannerNode> node;
  try {
    node = std::make_shared<PlannerNode>();
  } catch (const std::exception & error) {
    std::cerr << "planner_node: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
