## Start the Docker Environment

Navigate to the local directory `${REPOSITORY}/docker` and execute `./run.sh`. This will start the Docker container, in which ROS and all required libraries are preinstalled. You can stop the container by pressing <kbd>Ctrl</kbd>+<kbd>C</kbd> in the terminal. If everything is setup correctly you will see the following:

```
Starting new container...
================================================================================

=== CONTAINER INFORMATION ======================================================
Architecture: x86_64
Ubuntu: 22.04.2 LTS (Jammy Jellyfish)
Python: 3.10.6
ROS: humble
CMake: 3.22.1
CUDA: 12.1.105
cuDNN: 8.9.2
TensorRT: 8.6.1
TensorFlow Python: 2.13.0
TensorFlow C/C++: 
PyTorch Python: 
PyTorch C/C++: 
Available GPUs: 1
  name                         driver_version   utilization.gpu [%]   utilization.memory [%]   memory.used [MiB]   memory.total [MiB]
  NVIDIA GeForce RTX 2080 Ti   525.116.04       1 %                   2 %                      539 MiB             11264 MiB
================================================================================

root@@******:/home/rosuser/colcon_workspace# 
```

The `acdc` folder is mounted from your host into the container. Note that your current working directory inside the container is `/home/rosuser/colcon_workspace`.


## Download and Inspect Bag file

Download the folder `left_camera_templergraben` from [__here (1.3 GB)__](https://rwth-aachen.sciebo.de/s/sbSBamXYCfQw9kM).

Save this folder to your local directory `${REPOSITORY}/bag`. This directory will be mounted into the docker container to the path `/home/rosuser/ws/bag`.

You can start the docker container now with `./run.sh` (if you haven't already).

Inside the container, you can navigate to `/home/rosuser/ws/bag` and execute `ros2 bag info left_camera_templergraben` to inspect the rosbag:

```
Files:             left_camera_templergraben.db3
Bag size:          1.2 GiB
Storage id:        sqlite3
Duration:          17.999s
Start:             Aug  9 2019 10:59:01.823 (1565341141.823)
End:               Aug  9 2019 10:59:19.823 (1565341159.823)
Messages:          2881
Topic information: 

Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1801 | Serialization Format: cdr
Topic: /sensors/camera/left/camera_info | 
Type: sensor_msgs/msg/CameraInfo | Count: 540 | Serialization Format: cdr
Topic: /sensors/camera/left/image_raw | 
Type: sensor_msgs/msg/Image | Count: 540 |  Serialization Format: cdr
```

You can see that the rosbag has a duration of 18 seconds and contains 540 image frames of type sensor_msgs/Image and 540 corresponding sensor_msgs/CameraInfo messages. We will use these camera images in this assignment in order to apply image segmentation.

## ROS' `sensor_msgs/Image` Message

The message definition [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) is ROS' standard image message format. It is used for all kind of camera image message types and can be used seamlessly with many different ROS visualization and image processing tools. Please read the documentation about the [detailed message format](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) and it's content.

## ROS' `sensor_msgs/CameraInfo`

The message definition [sensor_msgs/CameraInfo](hhttps://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) is ROS' standard camera info message format. It is send together with `sensor_msgs/Image` to provide additional information about the current camera image such as __camera calibration parameters__. Feel free to read the documentation about the [detailed message format](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html).

## Build and source the package
The code for the image segmentation inference node can be found in the directory `src/section_2/image_segmentation_r2`. The structure of this __Python package__ is illustrated in the following:

```
image_segmentation_r2/
├── package.xml
├── setup.cfg
├── setup.py
├── image_segmentation_r2
    ├── image_segmentation.py
    ├── img_utils.py
    └── __init__.py
├── launch
│   ├── image_segmentation_r2.launch.py
│   └── params.yaml
├── models
│   ├── convert_cityscapes_to_ika_reduced.xml
│   ├── mobilenet_v3_large_968_608_os8.pb
│   └── mobilenet_v3_small_968_608_os8.pb
├── resource
├── test
```

The main source code is located in the directory `image_segmentation_r2`, the pretrained segmentation models are located in `models` and the launch file and parameters are located in directory `launch`. Feel free to read all the code, parameters and launch files.

Note, that we provide here two image segmentation models for you. Both rely on the MobilnetV3 architecture:  
- `mobilenet_v3_large_968_608_os8.pb`: Larger model, slower inference, more RAM needed
- `mobilenet_v3_small_968_608_os8.pb`: Smaller model, faster inference, less RAM needed

These models are trained on a much larger dataset compared to the model you have trained in the exercise, but the overall training pipeline and additional augmentation methods applied during the training are almost identical. 

You might change the model in the `params.yaml` configuration file depending on the capabilities of your computer.

Now, let's build the package with with `colcon build`
```bash
colcon build --packages-select image_segmentation_r2 --symlink-install
```

and source the workspace

```bash
source install/setup.bash
```

# Replay rosbag and run image segmentation

We have already prepared a launch file for you to execute the image segmentation. Please read carefully through the following lines of code. 

Contents of the file `image_segmentation_r2.launch`:

```py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Get the package and params directory
    image_segmentation_dir = get_package_share_directory('image_segmentation_r2')
    config = os.path.join(image_segmentation_dir, "config","params.yaml")

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock time')

    # ROSBAG PLAY node
    rosbag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--rate', '0.1', '-l',
             '/home/rosuser/ws/bag/left_camera_templergraben',
             '--topics', '/sensors/camera/left/image_raw',
             '/sensors/camera/left/camera_info'],
        output='screen'
    )

    # IMAGE_PROC node
    image_proc_node = Node(
        package='image_proc',
        name='image_proc',
        executable='image_proc',
        namespace='sensors/camera/left',
        output='screen',
        remappings=[
            ('image', 'image_raw'),
        ],
    )
        # CAMERA SEGMENTATION NODE
    camera_segmentation_node = Node(
        package='image_segmentation_r2',
        name='image_segmentation',
        executable='image_segmentation',
        output='screen',
        parameters=[config],
        remappings=[
            ('image_color', 'sensors/camera/left/image_color')
        ]
    )

        # NODES FOR VISUALIZATION
    segmentation_viewer_node = Node(
        package='image_view',
        executable='image_view',
        name='segmentation_viewer',
        remappings=[
            ('image', 'image_segmented'),
        ],
    )

    camera_node = Node(
        package='image_view',
        executable='image_view',
        name='camera_viewer',
        remappings=[
            ('image', 'sensors/camera/left/image_color'),
        ],
        parameters=[
                {'autosize': True},
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(use_sim_time)
    ld.add_action(rosbag_play_node)
    ld.add_action(image_proc_node)
    ld.add_action(camera_node)
    ld.add_action(camera_segmentation_node)
    ld.add_action(segmentation_viewer_node)

    return ld
```

Hence, we perform the following tasks:

- __Replay the rosbag__ with a speed of 0.05. Note, that we set the speed to a very low value here, because your computer might be very slow. You can adapt this values if your computer is fast enough to compute the segmentation at a higher speed.
- __Apply `image_proc`__ to the raw sensor data. The topic `/sensors/camera/left/image_raw` was recorded in the raw data format. With `stereo_image_proc` we convert it to a RGB encoding. Read more [here](https://wiki.ros.org/image_proc) about it.
- __Start the `image_segmentation`__ node and feed it with the correct topic name and load the parameters that are necessary for the node.
- Start two __nodes for visualization__