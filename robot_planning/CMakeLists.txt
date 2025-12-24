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
  gazebo_msgs
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
    visualization_msgs
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
add_library(roadmap_lib
  src/libraries/roadmap/roadmap_visualization.cpp
  src/libraries/roadmap/roadmap_data_structures.cpp
)
target_link_libraries(roadmap_lib
  ${OpenCV_LIBS}         
  ${catkin_LIBRARIES}
)


# --- Planning Utilities Library ---
add_library(planning_utils_lib src/libraries/planning_utils.cpp)
target_link_libraries(planning_utils_lib
  roadmap_lib
  ${catkin_LIBRARIES}
)

# --- Combinatorial Planning Library ---
add_library(combinatorial_planning_lib
  src/libraries/combinatorial_planning/exact_cell_decomposition.cpp
  src/libraries/combinatorial_planning/approximate_cell_decomposition.cpp
  src/libraries/combinatorial_planning/maximum_clearance_roadmap.cpp
  src/libraries/combinatorial_planning/shortest_path_roadmap.cpp
)
target_link_libraries(combinatorial_planning_lib
  roadmap_lib
  planning_utils_lib
  ${catkin_LIBRARIES}
)

# --- Sample Based Planning Library ---
add_library(sample_planning_lib
  src/libraries/sample_based_planning/prm.cpp
  src/libraries/sample_based_planning/rrt.cpp
  src/libraries/sample_based_planning/rrt_star.cpp
)
target_link_libraries(sample_planning_lib
  roadmap_lib
  planning_utils_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# --- Roadmap Factory Library ---
add_library(roadmap_factory_lib
  src/libraries/roadmap_factory.cpp
)
target_link_libraries(roadmap_factory_lib
  roadmap_lib
  combinatorial_planning_lib
  sample_planning_lib
  ${catkin_LIBRARIES}
)

# --- Graph Search Library (A*) ---
add_library(graph_search_lib 
  src/libraries/graph_search/astar.cpp
  src/libraries/graph_search/rviz_plot_plan.cpp)
target_link_libraries(graph_search_lib
  roadmap_lib
  ${catkin_LIBRARIES}
)

# ==============================================================================
# 4. Executables
# ==============================================================================

# --- Combinatorial Test Node ---
add_executable(combinatorial_test_node src/libraries/combinatorial_planning/test/combinatorial_test_node.cpp)
target_link_libraries(combinatorial_test_node 
  combinatorial_planning_lib
  roadmap_lib
  ${catkin_LIBRARIES}
)
add_dependencies(combinatorial_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Sampling Planning Test Node ---
add_executable(sampling_planning_test_node src/libraries/sample_based_planning/sampling_planning_test_node.cpp)
target_link_libraries(sampling_planning_test_node
  sample_planning_lib
  roadmap_lib   
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(sampling_planning_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Roadmap Test Node ---
add_executable(test_roadmap src/test_roadmap.cpp)
target_link_libraries(test_roadmap
  roadmap_lib
  combinatorial_planning_lib 
  sample_planning_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(test_roadmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- A* Planning Test Node ---
add_executable(astar_planning_test_node src/libraries/graph_search/astar_planning_test_node.cpp)
target_link_libraries(astar_planning_test_node
  graph_search_lib
  sample_planning_lib
  combinatorial_planning_lib 
  roadmap_lib
  roadmap_factory_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(astar_planning_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Complete Planning Pipeline Node ---
add_executable(complete_planning_pipeline_node src/complete_planning_pipeline.cpp)
target_link_libraries(complete_planning_pipeline_node
  graph_search_lib
  sample_planning_lib
  combinatorial_planning_lib 
  roadmap_lib
  roadmap_factory_lib
  dubins_client_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(complete_planning_pipeline_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(individual_benchmark_node src/individual_benchmark_pipeline.cpp)
target_link_libraries(individual_benchmark_node
  ${catkin_LIBRARIES}
  planning_utils_lib         # Per PlanningUtils
  roadmap_lib                # Per Roadmap
  roadmap_factory_lib        # Per generateRoadmap
  graph_search_lib           # Per AStar e TaskPlanner
  map_library                # Per MapBuilder
  dubins_client_library      # Per DubinsClient
  sample_planning_lib        # Necessarie se incluse indirettamente
  combinatorial_planning_lib # Necessarie se incluse indirettamente
)


add_executable(benchmark_node src/benchmark_pipeline.cpp)
target_link_libraries(benchmark_node
  ${catkin_LIBRARIES}
  planning_utils_lib         # Per PlanningUtils
  roadmap_lib                # Per Roadmap
  roadmap_factory_lib        # Per generateRoadmap
  graph_search_lib           # Per AStar e TaskPlanner
  map_library                # Per MapBuilder
  dubins_client_library      # Per DubinsClient
  sample_planning_lib        # Necessarie se incluse indirettamente
  combinatorial_planning_lib # Necessarie se incluse indirettamente
)