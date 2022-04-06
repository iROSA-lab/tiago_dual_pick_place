# Tiago++ Pick & Place pipeline

## Description
This is a fork of the Pal Robotics [tiago_pick_demo](https://github.com/pal-robotics/tiago_tutorials/tree/kinetic-devel/tiago_pick_demo). While the demo was made for the single arm Tiago and ROS Kinect, this package is for the (dual arm) Tiago++ and ROS Melodic!

Additional features:
* No requirement for aruco
* Pick/Place of scene objects (by object name)
* Pick/Place by sending a custom pose

## Installation
Follow the official [tutorial](http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS).

## Usage
### (Optional) Launching simulation
`roslaunch tiago_dual_pick_place pick_place_sim.launch`

### Launching Pick & Place pipeline
`roslaunch tiago_dual_pick_place pick_place.launch`  
This command starts both the Pick & Place server and client.

### Picking
The Pick & Place pipeline requires an object the planning scene to work with.  
There are two ways to pick: Either by specifing the name of the object in the scene or by specifying the grasp pose.

#### Picking object by name
In this case the object to be grasped has to exist in the scene. The object could be a virtual copy of an object in the real world.

To pick an object with the name 'Box_0': `rosservice call /pick_object 'Box_0'`

#### Picking by grasp pose
In this case a grasp pose (position and orientation) is directly given. Since the pipeline works with scene objects, a virtual object is constructed in the scene around the given pose. This object is then picked up.

To grasp by pose, first call the pick service: `rosservice call /pick`  
Then publish message with pose of object:
`rostopic pub /grasp/pose geometry_gs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_footprint'
pose:
  position:
    x: 0.2
    y: 0.2
    z: 0.8
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"`

### Placing
Placing requires a target pose and the object name of the object which is be placed.  
Currently, the target pose is hard-wired to be the original pose of the object before picking up (this will of course change in the future).

To place an object with the name 'Box_0': `rosservice call /place_object 'Box_0'`
