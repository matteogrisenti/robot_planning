cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

add_compile_options(-std=c++17)

# ==============================================================================
# 1. Dependencies
# ==============================================================================
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2_ros
  message_generation
  actionlib
  actionlib_msgs
  # Custom Packages
  dubins_planner
  map_library
)

# ==============================================================================
# 2. Message Files
# ==============================================================================
add_message_files(
  DIRECTORY msg
  FILES Reference.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ==============================================================================
# 3. Catkin Package Declaration
# ==============================================================================
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    geometry_msgs 
    nav_msgs 
    roscpp 
    std_msgs 
    tf2_ros 
    message_runtime 
    actionlib        
    actionlib_msgs     
  DEPENDS Eigen3
)

# ==============================================================================
# 4. Include Directories
# ==============================================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# ==============================================================================
# 5. Internal Libraries
# ==============================================================================
add_library(ros_controller_lib 
  src/lyapunov_controller.cpp
  src/ros_controller_node.cpp
)
target_link_libraries(ros_controller_lib
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
add_dependencies(ros_controller_lib ${catkin_EXPORTED_TARGETS})

# ==============================================================================
# 6. Executables
# ==============================================================================

# --- Main Node ---
add_executable(main src/main.cpp)
target_link_libraries(main
  ros_controller_lib
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
add_dependencies(main ${catkin_EXPORTED_TARGETS})


# --- Test Node ---
add_executable(controller_test_node src/test/controller_test_node.cpp)
target_link_libraries(controller_test_node 
  ${catkin_LIBRARIES} 
  dubins_library
  map_library
)
add_dependencies(controller_test_node ${catkin_EXPORTED_TARGETS})