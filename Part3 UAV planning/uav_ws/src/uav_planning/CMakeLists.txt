cmake_minimum_required(VERSION 3.8)
project(uav_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# 架次编号贯穿派单、A*路径和执行结果；坐标仍使用标准Path。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sortie.msg"
  "msg/SortieState.msg"
  DEPENDENCIES nav_msgs)
rosidl_get_typesupport_target(sortie_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)

# 纯数学工具库：不依赖 ROS 消息，供单点飞行节点和坐标测试复用。
add_library(frame_alignment STATIC src/frame_alignment.cpp)
target_include_directories(frame_alignment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(frame_alignment PUBLIC cxx_std_17)

add_library(astar_3d STATIC src/astar_3d.cpp)
target_include_directories(astar_3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(astar_3d PUBLIC cxx_std_17)
target_link_libraries(astar_3d ${TinyXML2_LIBRARIES})

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node astar_3d "${sortie_typesupport}")
target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planner_node PUBLIC cxx_std_17)
ament_target_dependencies(planner_node
  rclcpp rcl_interfaces geometry_msgs nav_msgs ament_index_cpp)

add_executable(mission_publisher_node src/mission_publisher_node.cpp)
target_compile_features(mission_publisher_node PUBLIC cxx_std_17)
ament_target_dependencies(mission_publisher_node
  rclcpp rcl_interfaces geometry_msgs nav_msgs)

add_executable(takeoff_hold_node src/takeoff_hold_node.cpp)
target_compile_features(takeoff_hold_node PUBLIC cxx_std_17)
ament_target_dependencies(takeoff_hold_node
  rclcpp rcl_interfaces px4_msgs)

# 两个入口复用同一套飞行控制，只切换任务完成时的行为。
add_library(goto_point_controller STATIC src/goto_point_node.cpp)
target_compile_features(goto_point_controller PUBLIC cxx_std_17)
target_link_libraries(goto_point_controller frame_alignment "${sortie_typesupport}")
ament_target_dependencies(goto_point_controller
  rclcpp rcl_interfaces px4_msgs geometry_msgs nav_msgs)

add_executable(goto_point_node src/goto_point_main.cpp)
target_link_libraries(goto_point_node goto_point_controller)
add_executable(goto_point_hover_node src/goto_point_hover_node.cpp)
target_link_libraries(goto_point_hover_node goto_point_controller)

add_executable(mission_dispatcher_node src/mission_dispatcher_node.cpp)
target_compile_features(mission_dispatcher_node PUBLIC cxx_std_17)
target_link_libraries(mission_dispatcher_node "${sortie_typesupport}")
ament_target_dependencies(mission_dispatcher_node rclcpp std_msgs geometry_msgs nav_msgs)

install(TARGETS planner_node mission_publisher_node mission_dispatcher_node
  takeoff_hold_node goto_point_node goto_point_hover_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/allocation_publisher scripts/flight_path_overlay
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS frame_alignment astar_3d ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# ros2 launch 从包的 share 目录查找资源，而不是从源码目录查找。
# 使用 colcon --symlink-install 时，这些资源会建立到源码的链接。
install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME})
install(FILES TAKEOFF_HOLD.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_alignment test/test_frame_alignment.cpp)
  target_link_libraries(test_frame_alignment frame_alignment)
  ament_add_gtest(test_astar_3d test/test_astar_3d.cpp)
  target_link_libraries(test_astar_3d astar_3d)
  target_compile_definitions(test_astar_3d PRIVATE
    TEST_WORLD_FILE="${CMAKE_CURRENT_SOURCE_DIR}/worlds/flood_demo.sdf")

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # 独立分配脚本不属于 ROS 代码；适配器仍按本包的 Python 规则检查。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(. scripts/allocation_publisher scripts/flight_path_overlay
    --exclude distribute/module04_train.py)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
