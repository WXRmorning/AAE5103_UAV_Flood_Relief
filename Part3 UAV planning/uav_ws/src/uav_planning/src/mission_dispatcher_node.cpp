// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uav_planning/msg/sortie.hpp"
#include "uav_planning/msg/sortie_state.hpp"

using std_msgs::msg::String;
using uav_planning::msg::Sortie;
using uav_planning::msg::SortieState;

class MissionDispatcherNode : public rclcpp::Node
{
public:
  MissionDispatcherNode()
  : Node("mission_dispatcher_node")
  {
    const double height = declare_parameter<double>("visit_height", 2.0);
    if (!std::isfinite(height) || height <= 0.0) {
      throw std::invalid_argument("visit_height 必须是正的有限数值。");
    }
    const auto visit_point = [this, height](const std::string & label) {
        const auto p = declare_parameter<std::vector<double>>(
          "points." + label, std::vector<double>{});
        if (p.size() != 3 || !std::isfinite(p[0]) || !std::isfinite(p[1]) ||
          !std::isfinite(p[2]))
        {
          throw std::invalid_argument("请在 mission_points.yaml 中配置 " + label + " 的坐标。");
        }
        // 点表保存表面高度，任务点在表面上方，最后由 PX4 下降到停机位。
        return Point{p[0], p[1], p[2] + height};
      };
    for (std::size_t i = 0; i < destinations_.size(); ++i) {
      destinations_[i] = visit_point(std::string(1, static_cast<char>('A' + i)));
    }
    for (std::size_t i = 0; i < aircraft_.size(); ++i) {
      auto & aircraft = aircraft_[i];
      const auto number = std::to_string(i + 1);
      const auto prefix = "/uav" + number;
      aircraft.home = visit_point("S" + number);
      aircraft.sortie_pub = create_publisher<Sortie>(prefix + "/sortie", 10);
      aircraft.execution_sub = create_subscription<SortieState>(
        prefix + "/execution_state", 10,
        [this, i](SortieState::ConstSharedPtr state) {on_execution(i, *state);});
      aircraft.planning_sub = create_subscription<SortieState>(
        prefix + "/planning_state", 10, [this, i](SortieState::ConstSharedPtr state) {
          auto & vehicle = aircraft_[i];
          if (vehicle.active && vehicle.active->id == state->task_id &&
          state->state == SortieState::FAILED && !vehicle.executing)
          {
            // 规划未交付路径，飞机没有起飞，可以释放该目标；不自动重派失败任务。
            block(i, state->detail);
            vehicle.active.reset();
          }
        });
    }
    missions_sub_ = create_subscription<String>(
      "/missions", 10, [this](String::ConstSharedPtr message) {enqueue(message->data);});
    assigned_sub_ = create_subscription<String>(
      "/assigned_missions", 10,
      [this](String::ConstSharedPtr message) {enqueue_assigned(message->data);});
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {dispatch();});
    RCLCPP_INFO(
      get_logger(), "等待 /missions 的待分配任务，或 /assigned_missions 的指定飞机任务。");
  }

private:
  using Point = std::array<double, 3>;
  struct Task
  {
    uint64_t id;
    char destination;
    std::size_t assigned_uav{0};  // 0 为共享任务，1/2 指定 UAV，禁止改派。
  };

  struct Aircraft
  {
    Point home{};
    std::optional<Task> active;
    uint64_t completed_id{0};
    rclcpp::Time feedback_at{0, 0, RCL_ROS_TIME};
    bool ready{false};
    bool executing{false};
    bool blocked{false};
    rclcpp::Publisher<Sortie>::SharedPtr sortie_pub;
    rclcpp::Subscription<SortieState>::SharedPtr execution_sub;
    rclcpp::Subscription<SortieState>::SharedPtr planning_sub;
  };

  static constexpr double kFeedbackTimeout = 2.0;  // 仿真秒。
  static constexpr double kAcceptanceTimeout = 8.0;

  void enqueue(const std::string & targets)
  {
    if (targets.empty() || !std::all_of(
        targets.begin(), targets.end(), [](char target) {
          return target >= 'A' && target <= 'G';
        }))
    {
      RCLCPP_WARN(get_logger(), "拒绝整串任务：需要非空的 A–G 大写字母串，如 ACACABCCB。");
      return;
    }
    const auto first = next_id_;
    for (char target : targets) {
      pending_.push_back({next_id_++, target});
    }
    RCLCPP_INFO(
      get_logger(), "收到 %zu 单（#%llu–#%llu）：%s。",
      targets.size(), static_cast<unsigned long long>(first),
      static_cast<unsigned long long>(next_id_ - 1), targets.c_str());
  }

  void enqueue_assigned(const std::string & pairs)
  {
    // 每两字符保留分配文件的一行：飞机编号 + 目标标签，例如 1A2C。
    std::size_t offset = 0;
    for (; offset + 1 < pairs.size(); offset += 2) {
      const char uav = pairs[offset];
      const char target = pairs[offset + 1];
      if ((uav != '1' && uav != '2') || target < 'A' || target > 'G') {
        break;
      }
    }
    if (pairs.empty() || offset != pairs.size()) {
      RCLCPP_WARN(get_logger(), "拒绝整批指定任务：需要连续的飞机和目标对，例如 1A2C。");
      return;
    }
    const auto first = next_id_;
    for (std::size_t i = 0; i < pairs.size(); i += 2) {
      pending_.push_back({next_id_++, pairs[i + 1], static_cast<std::size_t>(pairs[i] - '0')});
    }
    RCLCPP_INFO(
      get_logger(), "收到分配结果 %zu 单（#%llu–#%llu）：%s。",
      pairs.size() / 2, static_cast<unsigned long long>(first),
      static_cast<unsigned long long>(next_id_ - 1), pairs.c_str());
  }

  void block(std::size_t i, const std::string & reason)
  {
    auto & vehicle = aircraft_[i];
    if (!vehicle.blocked) {
      vehicle.blocked = true;
      if (vehicle.active) {
        RCLCPP_ERROR(
          get_logger(), "UAV%zu 的任务 #%llu 未完成，暂停给该机派单：%s",
          i + 1, static_cast<unsigned long long>(vehicle.active->id), reason.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "UAV%zu 暂停接单：%s", i + 1, reason.c_str());
      }
    }
  }

  void on_execution(std::size_t i, const SortieState & state)
  {
    auto & vehicle = aircraft_[i];
    if (vehicle.blocked) {
      return;
    }
    // 悬停等待时仍可能失去 Offboard；它没有当前任务，用上一单编号标记故障。
    if (state.state == SortieState::FAILED && vehicle.completed_id != 0 &&
      state.task_id == vehicle.completed_id)
    {
      block(i, state.detail);
      return;
    }
    if (vehicle.active) {
      // 刚派单时可能仍在等待起飞/规划，旧 READY/成功反馈不能释放这架飞机。
      if (state.task_id != vehicle.active->id) {
        return;
      }
      if (state.state == SortieState::FAILED) {
        block(i, state.detail);
      } else if (state.state == SortieState::BUSY) {
        vehicle.executing = true;
        vehicle.feedback_at = now();
      } else if (state.state == SortieState::SUCCEEDED) {
        vehicle.completed_id = state.task_id;
        vehicle.active.reset();
        RCLCPP_INFO(
          get_logger(), "UAV%zu 完成 #%llu，已回到 S%zu。%s",
          i + 1, static_cast<unsigned long long>(state.task_id), i + 1,
          state.detail.empty() ? "已落地上锁。" : state.detail.c_str());
      }
    }
    // 执行节点在成功清理并恢复可用后持续报告上一次成功；首次启动报告 READY(0)。
    if (!vehicle.active && state.task_id == vehicle.completed_id &&
      state.state == (vehicle.completed_id ? SortieState::SUCCEEDED : SortieState::READY))
    {
      vehicle.feedback_at = now();
      vehicle.ready = true;
    }
  }

  void dispatch()
  {
    const auto time = now();
    if (time.nanoseconds() <= 0) {
      return;
    }
    for (std::size_t i = 0; i < aircraft_.size(); ++i) {
      auto & vehicle = aircraft_[i];
      if (vehicle.active && !vehicle.blocked && (time - vehicle.feedback_at).seconds() >
        (vehicle.executing ? kFeedbackTimeout : kAcceptanceTimeout))
      {
        block(i, vehicle.executing ? "执行状态反馈超时。" : "未确认接单。");
      }
      if (pending_.empty() || vehicle.active || vehicle.blocked || !vehicle.ready ||
        (time - vehicle.feedback_at).seconds() > kFeedbackTimeout ||
        vehicle.sortie_pub->get_subscription_count() == 0)
      {
        continue;
      }
      // 有指定飞机时只看本机的下一单；另一架的排队不会挡住本机。
      const auto next = std::find_if(
        pending_.begin(), pending_.end(), [i](const Task & task) {
          return task.assigned_uav == 0 || task.assigned_uav == i + 1;
        });
      if (next == pending_.end()) {
        continue;
      }
      const auto task = *next;
      // 试跑版本允许两机同时去同一目标，暂时注释下面的互斥判断。
      // if (std::any_of(
      //     aircraft_.begin(), aircraft_.end(), [task](const auto & other) {
      //       return other.active && other.active->destination == task.destination;
      //     }))
      // {
      //   continue;
      // }
      const auto & destination = destinations_[static_cast<std::size_t>(task.destination - 'A')];
      Sortie request;
      request.task_id = task.id;
      request.path.header.frame_id = "map";
      request.path.header.stamp = time;
      for (const auto & point : {destination, vehicle.home}) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = request.path.header;
        pose.pose.position.x = point[0];
        pose.pose.position.y = point[1];
        pose.pose.position.z = point[2];
        pose.pose.orientation.w = 1.0;
        request.path.poses.push_back(pose);
      }
      // 先占用飞机，再发布路径。只有匹配本架次的结果才能结束占用。
      vehicle.active = task;
      vehicle.ready = false;
      vehicle.executing = false;
      vehicle.feedback_at = time;
      pending_.erase(next);
      vehicle.sortie_pub->publish(request);
      RCLCPP_INFO(
        get_logger(), "派单 #%llu 给 UAV%zu：S%zu → %c → S%zu；待派 %zu 单。",
        static_cast<unsigned long long>(task.id), i + 1, i + 1, task.destination,
        i + 1, pending_.size());
    }
  }

  std::array<Point, 7> destinations_{};
  std::array<Aircraft, 2> aircraft_;
  std::deque<Task> pending_;
  uint64_t next_id_{1};
  rclcpp::Subscription<String>::SharedPtr missions_sub_, assigned_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MissionDispatcherNode> node;
  try {
    node = std::make_shared<MissionDispatcherNode>();
  } catch (const std::invalid_argument & error) {
    std::cerr << "mission_dispatcher_node: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
