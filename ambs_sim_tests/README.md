# AMBS SImulation Tests

Currently simulates `test1_braking?` with the Turtlebot3 gazebo world environment.

## Bringup

1. Launch the sim    
`roslaunch ambs_sim_tests gazebo_sim.launch`   
can set arg `gazebo_gui` to `true` if you want the simulation engine, else starts in headless mode.

2. Launch the test   
`roslaunch ambs_sim_tests test1_braking.test`   

3. Start the test    
```
rostopic pub /ambs/event/ui/start_test ambs_msgs/BoolStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
data: true"
```
