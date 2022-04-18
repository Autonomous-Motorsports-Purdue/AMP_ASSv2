# Simulator nodes
This is a gazebo simulation of a racecar in a cone/wall/track environment. Control the racecar through `/racecar_cmd`.
Run with `roslaunch racecar_gazebo cones.launch`.

Get laser scans from `/scan`. Get rgb image from `/camera/rgb/image_rect_color`. Get depth from `/camera/depth/image_raw`

### gazebo
Contains the models for each scene, most importantly cones.world. 
Launching a world will also load all other required features:
- racecar_description
- gazebo
- joint controllers
- image publishers

Execute the cones.launch file to launch the simulator as a whole.

### racecar_description
`xacro` files and meshes for the racecar

### control
Sets up the joint controllers, and state publishers for each joint.

### basement
Some unrelated files I found during cleanup that may be useful later
