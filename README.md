# Tiago++ Pick & Place pipeline

## Description
This is a fork of the Pal Robotics [tiago_pick_demo](https://github.com/pal-robotics/tiago_tutorials/tree/kinetic-devel/tiago_pick_demo). While the demo was made for the single arm Tiago and ROS Kinect, this package is for the (dual arm) Tiago++ and ROS Melodic!

Additional features:
* No requirement for aruco
* Pick/Place of scene objects (by object name)
* Pick/Place by sending a custom pose

## Installation
Follow the official [tutorial](http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS).

## Configuration
### Changing the arm
This pipeline has been tested to work with the left arm and gripper. You can switch to the right arm by changing parameters in __launch/pick_place.launch__: simply replace all occurences of the word 'left' with 'right. Then, adjust the configuration files for the right arm in the __config__ folder to your needs (see below).

### Configuring motion
__config/pick_motions_left__ / __config/pick_motions_right__ define two positions for the left / right arm: a pre-grasp position and the final go-to position after picking up the object.

__config/pick_place_params_left__ / __config/pick_place_params_right__ define the tool frame, joints, gripper closure and grasp motion: direction of the grasp approach and direction to exiting the grasp (pre-grasp / post-grasp).

### Grasp generation
By default a spherical grasp generator is used. This generator creates possible grasps on a hemisphere around the target pose. [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigurehttp://wiki.ros.org/dynamic_reconfigure) is used to configure the grasp generation. You can also statically change the parameters by modifying __cfg/Grasps.cfg__.

## Usage
### (Optional) Launching simulation
`roslaunch tiago_dual_pick_place pick_place_sim.launch`

### Launching Pick & Place pipeline
`roslaunch tiago_dual_pick_place pick_place.launch`  
This command starts both the Pick & Place server and client.

### Picking
The Pick & Place pipeline requires an object in the planning scene to work with.  
There are two ways to pick: Either by specifing the name of the object in the scene or by specifying the grasp pose.

#### Picking object by name
In this case the object to be grasped has to exist in the scene. The object could be a virtual copy of an object in the real world.

To pick an object with the name 'Box_0': `rosservice call /pick_object 'Box_0'`

#### Picking by grasp pose
In this case a grasp pose (position and orientation) is directly given. Since the pipeline works with scene objects, a virtual object is constructed in the scene around the given pose and then picked up. Since this virtual object represents the grasp and not any actual object you can completely ignore the visualization.

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
Change position and orientation for the desired grasp. These can also be defined in a different reference frame ('frame_id').

### Placing
Placing requires a target pose and the object name of the object which is be placed.  
If no target pose is sent within 10 seconds, the original pose of the object (before being picking up) is used.

(Optional) To specify the pose for placing:
`rostopic pub /place/pose geometry_gs/PoseStamped "header:
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

To place an object with the name 'Box_0' at the specified pose (or alternatively the pickup pose): `rosservice call /place_object 'Box_0'`
