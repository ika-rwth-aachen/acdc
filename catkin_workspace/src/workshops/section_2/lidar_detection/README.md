# Point Cloud Object Detection C++

This package aims to perform object detection on point clouds.

## Overview

The Lidar detection package consists of different components:

* `lidar_detection`: Nodelet for object detection in Lidar point clouds. The nodelet holds the tensorflow session, subscribes and advertises the relevant topics. It subscribes to the point cloud topic and the static transformations and advertises the detected object list.
* `detector` + `detector_utils`: This components holds all functions and helper utilities for the detection, e.g. for bounding box candidates and the conversion to the object list.
* `pillar_utils`: This function contain all necessary utility functions to create the network input from the received point cloud. [see here](https://arxiv.org/abs/1812.05784)
* `definitions.h`: Functions for definitions.
