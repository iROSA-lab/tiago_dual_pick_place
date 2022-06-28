rosservice call /pick right &
rostopic pub -1 /grasp/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_footprint'
pose:
  position:
    x: 0.4
    y: -0.6
    z: 0.6
  orientation:
    x: 0.0
    y: 0.707
    z: 0.0
    w: 0.707"
