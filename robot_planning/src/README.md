# Individual Benchmark Pipeline
The primary role of this script is to run a **benchmarked mission**. It takes a planner type (e.g., RRT, PRM) and a time limit as input, generates a roadmap, plans a sequence to rescue victims, and then commands the robot to move using a *Dubins Path* controller while recording performance metrics (score, time, success rate).

## Component Logic & Key Functions
### A. Infrastructure & Real-Time Monitoring
- Odometry Tracking: The odomCallback continuously updates the robot's current position and speed. This is used to "lock" the start position and detect if the robot has stalled (stopped moving despite having a command).
- Safety & Collision Avoidance:
    - *isSegmentSafe*: This function performs high-resolution collision checking. It doesn't just check if a line hits an obstacle; it "walks" along the path in 5cm steps to ensure a *min_clearance* (Safety Margin) is maintained from all obstacles.
    - *integratePosition*: This is critical for connecting floating coordinates (like the robot's current location or a victim's spot) into the pre-generated roadmap by creating safe edges to the nearest $N$ nodes.

### B. Path Optimization
The optimizePath function performs a "Shortcut" routine. Since roadmap-generated paths (like those from A*) can be jagged or zig-zagged, this function attempts to draw straight lines between distant nodes in the path. If a straight line is "Safe" (doesn't hit obstacles), it deletes the intermediate nodes, resulting in a smoother, faster trajectory.

### C. The Execution Logic (Dubins Control)
The script uses a Dubins Planner, which accounts for the robot's turning radius ($0.4m$).
- Critical vs. Non-Critical: If a node is a "Critical Node" (a victim or the gate), the robot uses higher precision and more retries.
- Angle Calculation: For every segment, the script calculates the optimal goal_theta (heading). If the next turn is sharp (over $60^\circ$), it forces the robot to align with the current segment before turning.