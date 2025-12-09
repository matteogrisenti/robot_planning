cmake_minimum_required(VERSION 3.0.2)
project(robot_planning)

add_compile_options(-std=c++17)

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs   
  obstacles_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs obstacles_msgs
  DEPENDS OpenCV
)

# ==============================================================================
# Include Directories
# ==============================================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ==============================================================================
# Libraries
# ==============================================================================
add_library(dubins_library src/dubins_trajectory.cpp)
add_library(collision_checker_lib src/collision_checker.cpp)
add_library(data_structures src/data_structures.cpp)

target_link_libraries(data_structures
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  dubins_library
  collision_checker_lib
)

# ==============================================================================
# Executables
# ==============================================================================

# --- Roadmap Node (COMMENTED OUT: Missing src/roadmap_node.cpp) ---
# add_executable(roadmap_node src/roadmap_node.cpp)
# target_link_libraries(roadmap_node
#   ${catkin_LIBRARIES} 
#   ${OpenCV_LIBRARIES} 
#   data_structures
#   dubins_library
#   collision_checker_lib
# )
# add_dependencies(roadmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Data Verifier Node ---
add_executable(data_verifier_node src/data_verifier_node.cpp)
target_link_libraries(data_verifier_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  data_structures
  dubins_library
  collision_checker_lib
)
add_dependencies(data_verifier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

