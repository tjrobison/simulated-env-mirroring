# Simulated Environment Privacy Interface

Repository for work done for a robotic privacy interface involving a navigable 3D world
alongside a real-world video feed, for Cornell College Summer Research Institute 2016.

The environment used for development involved the robotic simulation software Gazebo v7.1
and ROS Kinetic. While the ROS nodes that function independently of Gazebo will likely work
with earlier versions of ROS, Gazebo functionality is restricted to recent Gazebo releases
and has not been tested on versions earlier than 7.0.

## Demo Videos

- [Simulated Mirroring Demos](https://youtu.be/AKyZ55t4OgU)
- [Merged Overlay Demo](https://youtu.be/Wj7Vkkc-Zts)

## Launch Files

- full_launch.launch launches all nodes necessary for the merged overlay, in the correct order.
    refer to the XML comments for brief descriptions on the function of each node.
- gazebo_camera.launch launches a gui-less (by default) instance of the Gazebo simulator,
    loads a world file, and spawns a camera with the same properties as the Kinect camera
    on the iRobot Create
- keyboard_teleop.launch runs a slightly modified version of the teleop script provided by
    ROS/iRobot for manual control. It allows switching between camera feeds in a single window 

## Worlds

- law_hall.world A highly accurate measured model of the fourth floor of the Law Hall 
    building at Cornell College
