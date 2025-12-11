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


find_package(Boost REQUIRED)
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
  ${Boost_INCLUDE_DIRS}
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
add_library(shortest_path_roadmap src/combinatorial_planning/shortest_path_roadmap.cpp)
target_link_libraries(exact_cell_decomposition 
  planning_utils
  roadmap_data_structures  # <--- ADD THIS
)

target_link_libraries(approximate_cell_decomposition 
  planning_utils
  roadmap_data_structures  # <--- ADD THIS
)

target_link_libraries(maximum_clearance_roadmap 
  map_data_structures
  roadmap_data_structures
)

target_link_libraries(shortest_path_roadmap 
  map_data_structures
  roadmap_data_structures
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
add_executable(combinatorial_test_node src/combinatorial_planning/combinatorial_test_node.cpp )
target_link_libraries(combinatorial_test_node
  map_builder 
  exact_cell_decomposition
  approximate_cell_decomposition
  maximum_clearance_roadmap
  shortest_path_roadmap
  roadmap_data_structures  # <--- ADD THIS
  ${catkin_LIBRARIES}
)
add_dependencies(combinatorial_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)