# Robot Planning
This component is responsible for high-level decision-making (Task Planning) and low-level path discovery (Graph Search) within a roadmap-based navigation system. It enables a robot to maximize mission value (collecting victims) while strictly adhering to a time budget.

<br>

## 1. Component Logic

### A* Graph Search (AStarPlanner)
The AStarPlanner provides the foundational navigation logic. It finds the shortest path between two discrete nodes in a roadmap.
- Algorithm: Standard A* search.
- Heuristic: Euclidean distance ($d = \sqrt{\Delta x^2 + \Delta y^2}$). This is "admissible," meaning it never overestimates the distance, ensuring the shortest path is found.
- Cost Function: $f(n) = g(n) + h(n)$, where $g(n)$ is the actual distance traveled from the start and $h(n)$ is the estimated distance to the goal.

### Task Planner (TaskPlanner)
The TaskPlanner acts as the "brain." It solves a variation of the Orienteering Problem.
- Greedy Selection: Instead of checking every possible permutation of tasks, it iteratively picks the "best" next victim based on this score:$$\text{Score} = \frac{\text{Victim Value}}{\text{Time to Reach} + 1.0}$$
- Time Budgeting: Before adding a victim to the sequence, the planner calculates:$$[Current Time] + [Time to Victim] + [Time from Victim to Gate] + [Safety Overhead]$$ If this total exceeds the time_limit, the victim is rejected.

<br>

## 2. Component Workflow
The following schema describes the execution flow from the moment a mission is triggered:

#### 1. The Entry Point
- Caller: External High-Level Controller (e.g., a ROS Mission Node). 
- Function: *planMissionSequence(...)*

#### 2. Internal Initialization
- Node Mapping: Converts coordinates (Start, Gate, Victims) into the nearest Graph Node IDs using *getNearestNodeIdx*.

- Candidate Wrap: Wraps victim data into Candidate structures, using the radius as the reward value.

#### 3. The Greedy Task Planning Loop
The TaskPlanner enters a while loop to build the sequence:
- Search: Iterates through all unvisited victims.
- Path Query: Calls *getGraphDistance* for each candidate.
    
    - Sub-call: *getGraphDistance* calls *AStarPlanner::computePath*.
    - Return: The sum of edge weights of the shortest path.

- Feasibility Check: Calculates if the robot can reach the victim and still return to the gate before time_limit.
- Selection: Picks the victim with the highest Value / Cost ratio.
- Update: Marks victim as visited, updates currentTime, and moves currentNode.

#### 4. Mission Finalization
- Exit Condition: No more victims are reachable within the time budget.
- Gate Addition: The gateNode is appended as the final destination.
- Return: Returns an ordered std::vector<int> of node IDs (the "Mission Blueprint").