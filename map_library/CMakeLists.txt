cmake_minimum_required(VERSION 3.0.2)
project(map_library)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  obstacles_msgs
)

find_package(OpenCV REQUIRED)  # Necessary for map visualization

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES map_library      
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    nav_msgs 
    geometry_msgs 
    visualization_msgs 
    obstacles_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Aggiungi qui i tuoi file sorgente della mappa (spostali da robot_planning a qui)
add_library(map_library
  src/map_builder.cpp
  src/map_visualization.cpp
  src/map_data_structures.cpp
)

# --- FIX: Aggiungi questa dipendenza ---
# Dice a CMake: "Non compilare map_library finché tutti i messaggi 
# dei pacchetti dipendenti (come obstacles_msgs) non sono stati generati".
add_dependencies(map_library ${catkin_EXPORTED_TARGETS})

target_link_libraries(map_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)