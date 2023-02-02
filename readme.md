# Use of drone simulation tools
This tool pack is to simulate drone in Gazebo such to test controllers using Mavros.

## Packages used by simulation tools 
- px4
- mavros for communicating with drones in ROS
- fake_qualisy for producing vicon/qualisys information from Gazebo
- mocap_to_mavros for feeding vision pose information to drone for OFFBOARD mode

## How to use
For simulation px4 in gazebo + VICON + onboard node (mavros + mocap_to_mavros), just run
```shell
    roslaunch drone_simulation_tools drone_px4_onboard.launch
```
Then, we should find
<figure>
    <img src="img/node_grash_px4_onboard.png"
         height="200">
</figure>

show topics and nodes
- gazebo simulates a px4 drone
- vicon_fake subscribe to gazebo and pushes drone pose info to /vicon/drone
- mocap_to_mavros feeds drone pose infor to /mavros/vision_pose
- mavros builds communication between ROS and drone