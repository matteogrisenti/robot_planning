cmake_minimum_required(VERSION 3.0.2)
project(robot_planning)

add_compile_options(-std=c++17)

# ==============================================================================
# 1. Dependencies
# ==============================================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs    
  obstacles_msgs
  visualization_msgs
  nav_msgs
  # Custom Packages
  dubins_planner
  map_library
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    obstacles_msgs 
    nav_msgs
    map_library   
  DEPENDS OpenCV
)

# ==============================================================================
# 2. Include Directories
# ==============================================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ==============================================================================
# 3. Internal Libraries
# ==============================================================================


# --- Roadmap Libraries ---
add_library(roadmap_visualization src/roadmap/roadmap_visualization.cpp)
target_link_libraries(roadmap_visualization
  ${OpenCV_LIBS}         
  ${catkin_LIBRARIES}
)

add_library(roadmap_data_structures src/roadmap/roadmap_data_structures.cpp)
target_link_libraries(roadmap_data_structures 
  roadmap_visualization
)

# --- Planning Utilities Library ---
add_library(planning_utils_lib src/combinatorial_planning/planning_utils.cpp)
target_link_libraries(planning_utils_lib
  roadmap_data_structures
  ${catkin_LIBRARIES}
)

# --- Combinatorial Planning Library ---
add_library(combinatorial_planning_lib
  src/combinatorial_planning/exact_cell_decomposition.cpp
  src/combinatorial_planning/approximate_cell_decomposition.cpp
  src/combinatorial_planning/maximum_clearance_roadmap.cpp
  src/combinatorial_planning/shortest_path_roadmap.cpp
)
target_link_libraries(combinatorial_planning_lib
  roadmap_data_structures
  planning_utils_lib
  ${catkin_LIBRARIES}
)

# --- Sample Based Planning Library ---
add_library(sample_planning_lib
  src/sample_based_planning/prm.cpp
  src/sample_based_planning/rrt.cpp
  src/sample_based_planning/rrt_star.cpp
)
target_link_libraries(sample_planning_lib
  roadmap_data_structures
  planning_utils_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# --- Graph Search Library (A*) ---
add_library(graph_search_lib src/graph_search/astar.cpp)
target_link_libraries(graph_search_lib
  roadmap_data_structures
  ${catkin_LIBRARIES}
)

# ==============================================================================
# 4. Executables
# ==============================================================================

# --- Combinatorial Test Node ---
add_executable(combinatorial_test_node src/combinatorial_planning/combinatorial_test_node.cpp)
target_link_libraries(combinatorial_test_node 
  combinatorial_planning_lib
  roadmap_data_structures
  ${catkin_LIBRARIES}
)
add_dependencies(combinatorial_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Sampling Planning Test Node ---
add_executable(sampling_planning_test_node src/sample_based_planning/sampling_planning_test_node.cpp)
target_link_libraries(sampling_planning_test_node
  sample_planning_lib
  roadmap_data_structures   
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(sampling_planning_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Roadmap Test Node ---
add_executable(test_roadmap src/test_roadmap.cpp)
target_link_libraries(test_roadmap
  roadmap_data_structures
  combinatorial_planning_lib 
  sample_planning_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(test_roadmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- A* Planning Test Node ---
add_executable(astar_planning_test_node src/graph_search/astar_planning_test_node.cpp)
target_link_libraries(astar_planning_test_node
  graph_search_lib
  sample_planning_lib
  combinatorial_planning_lib 
  roadmap_data_structures
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(astar_planning_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})