# robot_navigation
The goal of our project is to explore the kinematic and dynamic vehicle models in path planning and tracking. The planning process uses the Rapidly-exploring random tree (RRT)to create a path for the robot to follow, then we take advantage of model differential flatness to track the desired output via feedback linearization.
## RRT planning
We implemented a simulation environment with obstacles and used RRT to find a path between start locationand goal location.  RRT is a sampling based algorithm, a tree of possible paths is constructed incrementallyfrom samples drawn randomly from the configuration space.Usually for a realistic vehicle, the steering angle is limited within a small range.  Therefore, large steeringeffort should be avoided along the path. 

We optimized the path generation by setting bounds of possiblesteering angles and using weighted sum of trajectory length and control effort as a more reasonable distancemetric for choosing the nearest neighbor.  As the result shown in figure2, the trajectory is more realistic forthe robot to follow. The result of the planning process is shown below: 
<p align="center"><img src ="https://github.com/bigdayangyu/robot_navigation/blob/planandtrack/util/planning.gif" width = 60% /></p> 
## Tracking via Differential Flatness 
The figure indicates good tracking result: robot can follow the generated path quite well underrandom noise.  The red line is the target trajectory, while the orange dots indicate the actual path of therobot.  Also, the density of those orange dots can indicate the actual velocity of the robot at this location.
<p align="center"><img src ="https://github.com/bigdayangyu/robot_navigation/blob/planandtrack/util/rrt_tracking.gif" width = 60% /></p> 
