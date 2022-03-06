# AMP Autonomous Software Stack v2

The second iteration of the Autonomous Software Stack (ASS) for the AMP go-kart.

# Running 
* Show available options help
```
. amd64-docker.sh 0
```

To build the docker image, have a second argument behind the number.
* Run intel graphics docker
```
. amd64-docker.sh 1
```
* Run nvidia graphics docker
```
. amd64-docker.sh 2
```

# Design

Our software stack uses the [ROS 2d navigation package](http://wiki.ros.org/navigation) to obtain control information given sensor and odometry sources, a map, a [tf](http://wiki.ros.org/tf) transform tree, a node to publish the desired kart position (i.e. goal setting), and a set of configurations for the naviagtion stack's global and local planners and costmaps.

This repository contains top-level launch files for simulation and physical testing in `src/sim_stage/launch` and `.../launch` (TODO: Issue 34). We highlight some of the main packages below:

TODO(35): Add other packages and refactor current ones.

## sim_stage
This package runs the [`stage_ros`](http://wiki.ros.org/stage_ros) node. See `src/sim_stage/sim_files` for the image file that specifies the shape of the racetrack and the config files that define the simulated laserscanner sensor and kart.

This simulated kart takes /cmd\_vel topic to move. (cmd\_vel contains tf map frame, quaternion, and 3d pose, but we are only using it for forward displacement and yaw displacement)   

Broadcasts /top/scan, which is the current moment laserscan data as a list of each ray's distance in a 180 degree range. 

## slam_mode_goal
Reads top/scan and moves the kart towards open space (path finding). Puts out /amp\_slam\_goal which is a MoveBaseGoal structure that represents displacement. TODO: change from MoveBaseGoal to something simpler.

slam\_mode\_goal is intended to put out high-level, ideal cart commands that are translated by the mover node into hardware commands for the kart's motors and steering.

