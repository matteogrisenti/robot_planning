cmake_minimum_required(VERSION 3.0.2)
project(dubins_planner)

add_compile_options(-std=c++17)

# ==============================================================================
# 1. Dependencies
# ==============================================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  nav_msgs
  actionlib           # allow to create action servers and clients
  actionlib_msgs      # allow to use standard action messages
  # Custom Package
  map_library
)


# ==============================================================================
# 2. Action Files 
# ==============================================================================
add_action_files(
  DIRECTORY action
  FILES FollowDubins.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs 
)


# ==============================================================================
# 3. Catkin Package Declaration
# ==============================================================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    dubins_library
  CATKIN_DEPENDS 
    roscpp 
    nav_msgs 
    actionlib 
    actionlib_msgs
    message_runtime
)


# ==============================================================================
# 4. Include Directories
# ==============================================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# ==============================================================================
# 5. Internal Libraries
# ==============================================================================

# --- Dubins Planner Library ---
add_library(dubins_library 
  src/library/dubins_planner.cpp
  src/library/dubins_trajectory.cpp
  src/library/collision_checker.cpp
)
target_link_libraries(dubins_library
  ${catkin_LIBRARIES}
  map_library
)


# ==============================================================================
# 6. Executables
# ==============================================================================

# --- Dubins Test Node ---
add_executable(dubins_planner_test_node src/library/test/dubins_planner_test.cpp)
target_link_libraries(dubins_planner_test_node 
  map_library
  dubins_library
  ${catkin_LIBRARIES} 
)
add_dependencies(dubins_planner_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Dubins Planner Server Node ---
add_executable(dubins_planner_server_node src/dubins_planner_server.cpp)
target_link_libraries(dubins_planner_server_node 
  ${catkin_LIBRARIES} 
  dubins_library
)
add_dependencies(dubins_planner_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ==============================================================================