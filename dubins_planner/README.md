# Dubins Planner
This package implements Dubins-path planning, validation, and execution helpers for a differential-drive robot inside a ROS workspace. It provides:
  - A planner that computes shortest-feasible Dubins curves between two poses.
  - Collision checking utilities against polygonal and circular obstacles.
  - An action server/client pair to request following a planned Dubins path.


**Directory Structure**
```graphql
├── action/
│   └── FollowDubins.action       # Messages structures of the action
├── include/
│   ├── dubins_planner_client.h   # Library to export in system thet want to use the action
│   └── dubins_planner/           # Implemnt internal library
│       └── collision_checker.h   #`CollisionChecker` class and helper types.
│       └── dubins_planner.h      #`DubinsPlanner` class: high-level planner, execution and visualization helpers.
│       └── dubins_trajectory.h   # Low-level Dubins geometry, data structures and algorithms.
│
│
└── src/
    ├── dubins_planner_client.cpp   # Client class using the action client.
    ├── dubins_planner_server.cpp   # Server node implementing follow-and-execute behavior.
    └── library/   # implementation of library 


```


**Dubins Server Initialisation:**
1. Initialize a DubinsPlanner as core logic of the server
2. Read the Map and load it inside the DubinsPlanner 
3. Setup the Odom robot subcriber to be able to read the robot position
4. Activate the Action Server


**Dubins Server Work-Pipelin:**
1. It receive an action goal message that contain: goal pose, velocity and turning radio
PLANNING PHASE: 
2. Call the DubinsPlanner to plan a Dubins Curve from the current position ( get through the odometry subscriber ) to the goal pose following the velocity and turning radio constraint
3. Check that the Cuve is valid 
EXECUTION PHASE:
4. Keep active the spin() of the DubinsPlanner which:
  4.1 Extract the Reference pose for the controller
  4.2 Publish on the robot topi /ref the computed reference





