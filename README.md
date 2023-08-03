# HoloOcean ROS

A ROS 2 bridge for implementing a [HoloOcean-based underwater robotics simulation](https://holoocean.readthedocs.io) with sonar mapping.

Check out our intro video on [Youtube](https://www.youtube.com/watch?v=kyBcfZYSLzc).

## Features

* ROS2 bridge using rclpy and the HoloOcean Python API
* Thruster control using ROS2 messages
* Basic URDF file for the HoveringAUV vehicle based on the Unreal Engine simulation 
* Converts ImagingSonar data to PointCloud messages
  * Uses PyBind-based C++ module to handle the conversion
  * Runs the conversion on a background thread to reduce simulation delays
* Docker and Virtualenv based development environment (see [holoocean_tools](https://github.com/migeran/holoocean_tools))
* Demo application with Rviz visualization

## Usage

**NOTE:** You have to clone the holoocean_tools repository for using this project. 
Please check the README in [holoocean_tools](https://github.com/migeran/holoocean_tools) for building instructions.

To run the simulation with point cloud visualization:

```
cd /holoocean/holoocean_tools
./run_point_cloud_visualization_test.sh
```

Now the simulation is running. You can control the submarine by selecting the control window and using the following keys:

```
W: move forward
A: strafe left
S: move backwards
D: strafe right

I: move upwards
J: turn left
K: move downwards
L: turn right
```

You can also use HoloOcean's built-in hotkeys which are documented here: https://holoocean.readthedocs.io/en/develop/usage/hotkeys.html


## Future Work

* Implement ROS2 Control Hardware Interface
* Implement more sensors
* Allow handling more agents at the same time

## About Us

This project was implemented as a research project by [Migeran - Software Engineering & Consulting](https://migeran.com).
We are available for new software development projects in Robotics, AR/VR and other domains. If you have any questions, you may [contact us through our website](https://migeran.com/contact).