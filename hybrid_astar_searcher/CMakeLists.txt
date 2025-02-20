cmake_minimum_required(VERSION 3.8)
project(hybrid_astar_searcher)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find ompl package
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/hybrid_astar_searcher)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/calculate_heuristic.cpp
  src/hybrid_astar.cpp
  src/dynamicvoronoi.cpp
  src/bucketedqueue.cpp
  src/smooth.cpp
  src/ReedsSheppPath.cpp
  src/example.cpp
)

# Link ompl library
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()