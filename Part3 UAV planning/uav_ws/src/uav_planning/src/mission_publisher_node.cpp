// Copyright 2026 uav_planning contributors
// SPDX-License-Identifier: Apache-2.0

/*
 * 任务路线发布：坐标waypoints或命名route -> nav_msgs/Path -> 当前无人机的mission话题。
 * waypoints每3个数是一个map绝对访问坐标；route从mission_points.yaml查表，
 * 将标记表面Z加visit_height得到访问高度。两种输入只选一种，不写死默认路线。
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

class MissionPublisherNode : public rclcpp::Node
{
public:
  MissionPublisherNode()
  : Node("mission_publisher_node")
  {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    auto values = declare_parameter<std::vector<double>>(
      "waypoints", std::vector<double>{}, read_only);
    const auto route = declare_parameter<std::vector<std::string>>(
      "route", std::vector<std::string>{}, read_only);
    if (values.empty() == route.empty()) {
      throw std::invalid_argument("请选择一种输入：waypoints坐标数组，或route名称数组。");
    }
    if (!route.empty()) {
      values = resolve_route(route, read_only);
    }
    if (values.empty() || values.size() % 3 != 0) {
      throw std::invalid_argument("waypoints需要[x1,y1,z1,x2,y2,z2,...]，至少提供一个点。");
    }
    mission_.header.frame_id = "map";
    for (std::size_t i = 0; i < values.size(); i += 3) {
      if (!std::isfinite(values[i]) || !std::isfinite(values[i + 1]) ||
        !std::isfinite(values[i + 2]))
      {
        throw std::invalid_argument("waypoints中的坐标必须为有限数值。");
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = values[i];
      pose.pose.position.y = values[i + 1];
      pose.pose.position.z = values[i + 2];
      pose.pose.orientation.w = 1.0;  // 本阶段执行节点仍保持任务开始时的航向。
      mission_.poses.push_back(pose);
    }
    // 单机使用原话题；多机通过ROS重映射把任务发给指定无人机。
    publisher_ = create_publisher<nav_msgs::msg::Path>("/uav/mission", 10);
    // 等待ROS发现订阅者；连接后只发一次，不循环重复下发飞行任务。
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {
        if (publisher_->get_subscription_count() == 0) {
          return;
        }
        mission_.header.stamp = now();
        for (auto & pose : mission_.poses) {
          pose.header.stamp = mission_.header.stamp;
        }
        publisher_->publish(mission_);
        timer_->cancel();
        RCLCPP_INFO(
          get_logger(), "已向%s发布%zu个任务点，仅发送一次；可按Ctrl+C退出。",
          publisher_->get_topic_name(), mission_.poses.size());
      });
    RCLCPP_INFO(
      get_logger(), "任务含%zu个点，等待%s订阅者。",
      mission_.poses.size(), publisher_->get_topic_name());
  }

private:
  std::vector<double> resolve_route(
    const std::vector<std::string> & route,
    const rcl_interfaces::msg::ParameterDescriptor & read_only)
  {
    // 首个名称仅标明预期起点，实际搜索起点仍由定位节点提供。
    const auto start_name = declare_parameter<std::string>("start_name", "S1", read_only);
    if (route.size() < 2 || route.front() != start_name) {
      throw std::invalid_argument(
              "route需以" + start_name + "开头并包含目的点；start_name不会改变实际出生位置。");
    }
    const double height = declare_parameter<double>("visit_height", 2.0, read_only);
    if (!std::isfinite(height) || height <= 0.0) {
      throw std::invalid_argument("visit_height必须为正有限数值。");
    }
    std::vector<double> values;
    for (std::size_t i = 0; i < route.size(); ++i) {
      const std::string key = "points." + route[i];
      if (!has_parameter(key)) {
        declare_parameter<std::vector<double>>(key, std::vector<double>{}, read_only);
      }
      const auto point = get_parameter(key).as_double_array();
      if (point.size() != 3 || !std::isfinite(point[0]) ||
        !std::isfinite(point[1]) || !std::isfinite(point[2]))
      {
        throw std::invalid_argument("名称" + route[i] + "未配置有效坐标，请检查mission_points.yaml。");
      }
      if (i > 0) {
        // 保留中途和末尾的返航点，只省略首个起点标识。
        values.insert(values.end(), {point[0], point[1], point[2] + height});
      }
    }
    RCLCPP_INFO(
      get_logger(), "命名路线包含%zu个名称；首个%s表示起点，转换为%zu个目的点。",
      route.size(), start_name.c_str(), values.size() / 3);
    return values;
  }

  nav_msgs::msg::Path mission_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MissionPublisherNode> node;
  try {
    node = std::make_shared<MissionPublisherNode>();
  } catch (const std::exception & error) {
    std::cerr << "mission_publisher_node: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
