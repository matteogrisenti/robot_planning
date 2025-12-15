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
  nav_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_planning
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs obstacles_msgs nav_msgs
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
# Library to menage the Map
add_library(map_data_structures src/map/map_data_structures.cpp)
add_library(map_visualization src/map/map_visualization.cpp)
add_library(map_builder src/map/map_builder.cpp)
target_link_libraries(map_data_structures 
    ${OpenCV_LIBS}               
    map_visualization            # map_data_structures uses map_visualization
    ${catkin_LIBRARIES}
)
target_link_libraries(map_visualization
    ${OpenCV_LIBS}              # map_visualization uses OpenCV
    ${catkin_LIBRARIES}         # map_visualization uses MapVisualizer
)
target_link_libraries(map_builder 
    map_data_structures            # map_builder uses map_data_structures
    ${catkin_LIBRARIES}
)


# Library to menage the Roadmap
add_library(roadmap_data_structures src/roadmap/roadmap_data_structures.cpp)
add_library(roadmap_visualization src/roadmap/roadmap_visualization.cpp)
target_link_libraries(roadmap_data_structures 
    map_data_structures
    roadmap_visualization
)
target_link_libraries(roadmap_visualization
    ${OpenCV_LIBS}          
    ${catkin_LIBRARIES}
)


# Library to menage the DUblins Trajectory
add_library(dubins_library src/dubins_trajectory.cpp)
add_library(collision_checker_lib src/collision_checker.cpp)


# Library for combinatorial planning 
add_library(planning_utils src/combinatorial_planning/planning_utils.cpp)
add_library(exact_cell_decomposition src/combinatorial_planning/exact_cell_decomposition.cpp)
add_library(approximate_cell_decomposition src/combinatorial_planning/approximate_cell_decomposition.cpp)
add_library(maximum_clearance_roadmap src/combinatorial_planning/maximum_clearance_roadmap.cpp)
target_link_libraries(exact_cell_decomposition 
  planning_utils
  roadmap_data_structures  # <--- ADD THIS
)

target_link_libraries(approximate_cell_decomposition 
  planning_utils
  roadmap_data_structures  # <--- ADD THIS
)

target_link_libraries(maximum_clearance_roadmap 
  planning_utils
  roadmap_data_structures  # <--- ADD THIS
)

# ==============================================================================
# Executables
# ==============================================================================

# --- Data Verifier Node ---
add_executable(data_verifier_node src/data_verifier_node.cpp)
target_link_libraries(data_verifier_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  map_builder
  dubins_library
  collision_checker_lib
)
add_dependencies(data_verifier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Test node
add_executable(roadmap_test_node src/combinatorial_planning/roadmap_test_node.cpp )
target_link_libraries(roadmap_test_node
  map_builder 
  exact_cell_decomposition
  approximate_cell_decomposition
  maximum_clearance_roadmap
  roadmap_data_structures  # <--- ADD THIS
  ${catkin_LIBRARIES}
)
add_dependencies(roadmap_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Sampling Planning Test Node ---
add_executable(sampling_planning_test_node 
    src/sample_based_planning/sampling_planning_test_node.cpp
    src/sample_based_planning/prm.cpp
    src/sample_based_planning/rrt.cpp
    src/sample_based_planning/rrt_star.cpp
    src/map/map_builder.cpp
    src/map/map_data_structures.cpp
    src/map/map_visualization.cpp
    src/roadmap/roadmap_data_structures.cpp
    src/roadmap/roadmap_visualization.cpp
    src/combinatorial_planning/planning_utils.cpp
)

target_link_libraries(sampling_planning_test_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(sampling_planning_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# =========================================================
# 1. Crea una libreria per gli algoritmi Sample Based
# (Mancava dai tuoi log, quindi la definiamo qui per pulizia)
# =========================================================
add_library(sample_planning_lib
    src/sample_based_planning/prm.cpp
    src/sample_based_planning/rrt.cpp
    src/sample_based_planning/rrt_star.cpp
    # Se planning_utils non è già una lib a parte, scommenta la riga sotto:
    # src/combinatorial_planning/planning_utils.cpp 
)
# Linka le dipendenze base alla nuova libreria
target_link_libraries(sample_planning_lib
    map_data_structures
    roadmap_data_structures
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# =========================================================
# 2. Configura l'eseguibile test_roadmap
# =========================================================
add_executable(test_roadmap src/test_roadmap.cpp)

target_link_libraries(test_roadmap
    map_builder
    map_data_structures
    roadmap_data_structures
    exact_cell_decomposition
    approximate_cell_decomposition
    maximum_clearance_roadmap
    sample_planning_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(test_roadmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})