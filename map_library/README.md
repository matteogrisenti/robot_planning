# map_library
map_library is a core ROS package that defines the fundamental data structures and utilities for map manipulation and representation.

It is designed as a pure library package (no nodes) to act as the base dependency for the entire planning stack.

## 🏗 Architecture & Motivation
Originally, the map logic was implemented as an internal library within the robot_planning package. It was extracted into this standalone package to resolve a circular dependency architecture:

- Shared Dependency: The map data is required by both the High-Level Planner (robot_planning) and the Low-Level Planner (dubins_planner).

- Dubins Validation: The dubins_planner specifically requires this library to validate generated trajectories, checking them against obstacles and map borders to ensure collision-free paths before execution.

By moving these structures to map_library, both packages can depend on it hierarchy-free: map_library ← dubins_planner ← robot_planning

## 📦 Modules
This package utilizes a single **Umbrella Header** (*map_library.h*) to expose the entire API. By including this file, you automatically gain access to the three core modules:

- **Data Structures** (*map_library/map_data_structures.h*): Standardized C++ structs for Point, Vertex, Map, Obstacle, Gate, and Victim.

- **Map Builder** (*map_library/map_builder.h*): A utility class that loads map data (dimensions, obstacles, POIs) from ROS parameters or static definitions.

- **Visualization** (*map_library/map_visualization.h*): Helper functions to instantly render the Map and Obstacles in Rviz using visualization_msgs. 

