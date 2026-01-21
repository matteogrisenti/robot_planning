# 🚀 Benchmark Execution Steps

## 1. Environment Setup (Terminal 1) 

Initialize the Docker container and launch the simulation environment:

1.1. Activate the docker image
```Bash
lab_planning
```

1.2 Build the workspace
```Bash
cd ros_ws
catkin_make
source devel/setup.bash
```

1.3 Launch the simulation environment
```Bash
roslaunch robot_planning target_rescue.launch
```


## 2. Individual Benchmark Run (Terminal 2)
Open a new terminal to interface with the running container and execute the planners:

2.1 Link to the running docker container
```Bash
dock-other
```

2.2 Build the workspace
```Bash
cd ros_ws
catkin_make
source devel/setup.bash
```

2.3 Run a specific planner 

To test an example with combinatorial planner:
```Bash
rosrun robot_planning individual_benchmark_node _planner_type:=spr _time_limit:=120.0
```
To test an example with sample-based planner:
```Bash
rosrun robot_planning individual_benchmark_node _planner_type:=prm _time_limit:=120.0
```

## 🛠 Benchmark Configuration
Each run uses the individual_benchmark_node with the following parameters:

- **_planner_type**: The algorithm to test *spr, mcr, acd, ecd, prm, rrt, rrt_star*.

- **_time_limit**: Maximum allowed computation time (default set to 120.0 seconds, but tested with also 60 and 30 seconds).

NB: To run a second benchmark you have to reset the enviroment by terminate the current execution in Terminal one and re run the 1.3 script. 