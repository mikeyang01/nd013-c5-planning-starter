## Project Overview
In this project, you will implement two of the main components of a traditional hierarchical planner: The Behavior Planner and the Motion Planner. Both will work in unison to be able to:

Avoid static objects (cars, bicycles and trucks) parked on the side of the road (but still invading the lane). The vehicle must avoid crashing with these vehicles by executing either a “nudge” or a “lane change” maneuver.
Handle any type of intersection (3-way, 4-way intersections and roundabouts) by STOPPING in all of them (by default)
Track the centerline on the traveling lane.
To accomplish this, you will implement:

Behavioral planning logic using Finite State Machines - FSM
Static objects collision checking.
Path and trajectory generation using cubic spirals
Best trajectory selection though a cost function evaluation. This cost function will mainly perform a collision check and a proximity check to bring cost higher as we get closer or collide with objects but maintaining a bias to stay closer to the lane center line.

## Project Requirements
The project files have TODOs for you to be able to:

Define the FSM states and the logic (transitions between them) required to handle static objects and intersections that are required to STOP for 5 seconds. The minimum behavior we are expecting to see should be able to handle the following transitions:

from “lane following” to “deceleration to stop”,
from “deceleration to stop” to “stay stopped”,
and from “stay stopped” back to “lane following” (when is safe to do so). Students should feel free to augment this behavior.
Know the difference between splines, min jerk trajectories (MJTs) and spiral paths. Also, understand the reason why we chose to implement a cubic spiral. The majority of the mathematical code for generating the cubic spirals will be provided. To create an optimal path between 2 points, you will have to:

Decide where to locate the goal point as a result of the desired behavior (Behavior planner). In a perfect world, the goal is located on the center of the lane at a distance “d” ahead of the ego vehicle. Since we don’t know if a path is feasible between these 2 points, you will have to exploit the road structure and choose laterally offset goals from this ideal location and generate other alternative paths that will be evaluated later.

Return a discretized/sampled spiral. Deciding the number of points used to discretize will have a huge impact on checking for collisions and visualization: Too many points, and we’ll pay a high computation penalty and the visualization will be terribly slow. Too few and we might miss detecting collisions and we will teleport large distances making it visually not natural. The velocity profile generator computes a velocity trajectory from a starting speed to a desired speed.

It works in unison with the behavioral planner as it needs to build a velocity profile for each of the states that the vehicle can be in. In the “Follow_lane" state, we need to either speed up or speed down to maintain a speed target. In the "decel_to_stop" state, we need to create a profile that allows us to decelerate smoothly to a stop line. The order of precedence for handling these cases is stop sign handling and then nominal lane traveling. In a real velocity planner, you would need to handle the coupling between these states, but for simplicity this project can be implemented by isolating each case.

Implement a circle-based collision checking algorithm.

Finally, you will evaluate each trajectory generated against an objective function and select the “best” one. The goal of this objective function is to score poor paths that are in collision OR too close to static obstacles and to score high paths that end up closer to the center-line of the global path.

## Results
<img src="img/result.png"/>
