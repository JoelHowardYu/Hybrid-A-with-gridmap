cmake_minimum_required(VERSION 3.8)
project(create_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find Qt5 package
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Find OMPL package
find_package(ompl REQUIRED)

# Declare executables
add_executable(hybrid_astar_test src/hybrid_astar_test.cpp)
add_executable(ompl_test src/ompl_test.cpp)
add_executable(simple_demo_node src/simple_demo_node.cpp)

# Include directories
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/motion_control/decompUtil
  ${OMPL_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${DECOMP_ROS_UTILS_INCLUDE_DIRS}
)

# Target dependencies
ament_target_dependencies(
  hybrid_astar_test
  "rclcpp"
  "hybrid_astar_searcher"
  "ompl"
  "geometry_msgs"
  "grid_map_core"
  "grid_map_ros"
  "grid_map_filters"
  "grid_map_visualization"
  "grid_map_msgs"
  "nav_msgs"
  "visualization_msgs"
  "decomp_ros_msgs"
  "decomp_ros_utils"
)

ament_target_dependencies(
  ompl_test
  "rclcpp"
  "hybrid_astar_searcher"
  "ompl"
  "geometry_msgs"
  "grid_map_core"
  "grid_map_ros"
  "grid_map_filters"
  "grid_map_visualization"
  "grid_map_msgs"
  "nav_msgs"
  "visualization_msgs"
)

ament_target_dependencies(
  simple_demo_node
  "rclcpp"
  "hybrid_astar_searcher"
  "ompl"
  "geometry_msgs"
  "grid_map_core"
  "grid_map_ros"
  "grid_map_filters"
  "grid_map_visualization"
  "grid_map_msgs"
  "nav_msgs"
  "visualization_msgs"
)

# Link OMPL library
target_link_libraries(hybrid_astar_test ${OMPL_LIBRARIES})
target_link_libraries(ompl_test ${OMPL_LIBRARIES})
target_link_libraries(simple_demo_node ${OMPL_LIBRARIES})

# Install targets
install(TARGETS 
  hybrid_astar_test
  ompl_test
  simple_demo_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()