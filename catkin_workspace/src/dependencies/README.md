# Dependencies

## controller
Contains the vehicle controller, which receives measurements from the simulation, processes them, and provides actuator commands back to the simulation.

## definitions
Some required message definitions and helper functions for other packages

## flatland
Flatland is a performance-optimized 2D simulator for ground robots on a flat surface.
It is a light-weight alternative to the more heavy ROS-native Gazebo simulator.
Flatland uses Box2D for physics simulation and it is built to integrate directly with ROS.
Flatland loads its simulation environment from YAML files and provides a plugin system for extending its functionalities.

## flatland_ika_plugins
Self-written plugins to adapt the simulation environment to our needs.

## misc
This folder consists of a package that distorts ideal rosbags and a tensorflow ROS C++ adapter.

## rviz_plugins
Additional visualization plugins for rviz to display messages from "definitions".