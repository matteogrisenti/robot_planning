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
<<<<<<< HEAD
#  INCLUDE_DIRS include
#  LIBRARIES robot_planning
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs obstacles_msgs nav_msgs
=======
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs obstacles_msgs
>>>>>>> 310de8918d86c3fa745998be87e103a96f3def9f
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
<<<<<<< HEAD
# Library to menage the Map
add_library(map_data_structures src/map/map_data_structures.cpp)
add_library(map_visualization src/map/map_visualization.cpp)
add_library(map_builder src/map/map_builder.cpp)
# Library to menage the Roadmap
add_library(roadmap_data_structures src/roadmap/roadmap_data_structures.cpp)
add_library(roadmap_visualization src/roadmap/roadmap_visualization.cpp)
# Library for combinatorial planning 
add_library(planning_utils src/combinatorial_planning/planning_utils.cpp)
add_library(exact_cell_decomposition src/combinatorial_planning/exact_cell_decomposition.cpp)
add_library(approximate_cell_decomposition src/combinatorial_planning/approximate_cell_decomposition.cpp)
add_library(maximum_clearance_roadmap src/combinatorial_planning/maximum_clearance_roadmap.cpp)


target_link_libraries(map_data_structures 
    ${OpenCV_LIBS}               # ADD THIS - data_structures uses OpenCV
    map_visualization            # ADD THIS - data_structures uses MapVisualizer
    ${catkin_LIBRARIES}
)

target_link_libraries(map_visualization
    ${OpenCV_LIBS}           # ADD THIS if visualization uses OpenCV
    ${catkin_LIBRARIES}
)

target_link_libraries(map_builder 
    map_data_structures            # ADD THIS - data_structures uses MapVisualizer
    ${catkin_LIBRARIES}
)

target_link_libraries(roadmap_data_structures 
    map_data_structures
    roadmap_visualization
)

target_link_libraries(roadmap_visualization
    ${OpenCV_LIBS}           # ADD THIS if visualization uses OpenCV
    ${catkin_LIBRARIES}
)

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
=======
add_library(collision_checker_lib src/collision_checker.cpp)
add_library(data_structures src/data_structures.cpp)
>>>>>>> 310de8918d86c3fa745998be87e103a96f3def9f

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

<<<<<<< HEAD
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
=======
>>>>>>> 310de8918d86c3fa745998be87e103a96f3def9f
