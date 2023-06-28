#!/usr/bin/env python3

#
#  ==============================================================================
#  MIT License
#
#  Copyright 2022 Institute for Automotive Engineering of RWTH Aachen University.
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright nothow to decleare parameters in ROS'
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#  ==============================================================================
#


import os
import numpy as np
import rclpy
from rclpy.node import Node
import tensorflow as tf
import cv2

import sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from cv_bridge import CvBridge
import struct
import xml.etree.ElementTree as ET

class PCLSegmentation(Node):
    """
    Implements a point cloud segmentation node.
    """
    def predict(self, pcl_msg):
        pcl = np.array(list(pc2.read_points(pcl_msg)))

        # perform fov filter by using hv_in_range
        cond = self.hv_in_range(x=pcl[:, 0],
                                y=pcl[:, 1],
                                z=pcl[:, 2],
                                fov=[-self.left_azimuth, self.right_azimuth])
        #self.get_logger().info("pcl : {}".format(self.left_azimuth))
        # apply fov filter
        pcl = pcl[cond]
        #self.get_logger().info("pcl : {}".format(pcl))
        # get depth map
        lidar = self.pcl_spherical_projection(pcl=pcl,
                                              height=self.num_rings,
                                              width=self.num_azimuth,
                                              num_channels=5,
                                              leftPhi=np.radians(self.left_azimuth),
                                              rightPhi=np.radians(self.right_azimuth)).astype(np.float32)

        # Compute binary mask: True where the depth is bigger then 0, false in any other case
        lidar_mask = lidar[:, :, 4] > 0

        # normalize
        lidar_input = (lidar - self.normalization_mean) / self.normalization_std

        # Set lidar on all channels to zero where the mask is False. Ie. where no points are present
        lidar_input[~lidar_mask] = 0.0

        # Append mask to lidar input
        lidar_input = np.append(lidar_input, np.expand_dims(lidar_mask, -1), axis=2)

        # add batch dim
        lidar_input = np.expand_dims(lidar_input, axis=0)
        lidar_mask = np.expand_dims(lidar_mask, axis=0)

        probabilities, predictions = self.model([lidar_input, lidar_mask])

        # remove batch dim
        predictions = predictions[0]

        predictions_rgb = self.color_palette[predictions]

        if self.do_visualizations:
            # visualize
    
            pred_cls_color_bgr = predictions_rgb[..., ::-1]  #RGB TO BGR
            cv2.imshow("Segmented PCL", cv2.resize(pred_cls_color_bgr, (0, 0), fx=4, fy=4))
            cv2.waitKey(1)

        # convert to pcl msg
        x = lidar[:, :, 0].reshape(-1)  # x
        y = lidar[:, :, 1].reshape(-1)  # y
        z = lidar[:, :, 2].reshape(-1)  # z
        i = lidar[:, :, 3].reshape(-1).astype(np.uint8)  # intensity
        l = predictions.numpy().reshape(-1).astype(np.uint8)  # label
        rgbs = predictions_rgb.reshape(-1, 3)  # rgb

        rgb_float = []
        for rgb in rgbs: rgb_float.append(self.rgb_to_float((rgb*255).astype(int)))

        # delete the points where the none class is present
        delete_ids = np.where(l==self.num_classes-1)
        x = np.delete(x, delete_ids)
        y = np.delete(y, delete_ids)
        z = np.delete(z, delete_ids)
        i = np.delete(i, delete_ids)
        l = np.delete(l, delete_ids)
        rgb_float = np.delete(np.asarray(rgb_float), delete_ids)
        
        # create list of points
        points = list(zip(x, y, z, i, l, rgb_float))

        segmented_pcl_msg = pc2.create_cloud(header=pcl_msg.header,
                                             fields=self.point_field,
                                             points=points)

        ### START TASK 2 CODE HERE ###
        
        # Task 2:
        # call publisher to publish the message "segmented_pcl_msg"

        self.pub_seg.publish(segmented_pcl_msg)
        ### END TASK 2 CODE HERE ###


    def pcl_spherical_projection(self, pcl, height, width, num_channels, leftPhi, rightPhi):

        """
        Project velodyne points into front view depth map.
        :param pcl: velodyne points in shape [:,5]
        :param height: the row num of depth map, could be 64(default), 32, 16
        :param width: the col num of depth map
        :param num_channels: the channel size of depth map
            3 cartesian coordinates (x; y; z),
            an intensity measurement and
            range r = sqrt(x^2 + y^2 + z^2)
        :param leftPhi: left opening angle for the FOV of the projection
        :param rightPhi: right opening angle for the FOV of the projection
        :return: `spherical_projection`: the projected depth map of shape[H,W,C]
        """
        x = pcl[:, 0]
        y = pcl[:, 1]
        z = pcl[:, 2]
        i = pcl[:, 3]
        r = pcl[:, 4].astype(int)  # ring
        d = np.sqrt(x**2 + y**2 + z**2)

        # projection on to a 2D plane
        deltaPhi_ = (rightPhi + leftPhi) / width
        phi = np.arctan2(y, x)
        phi_ = ((leftPhi - phi) / deltaPhi_).astype(int)

        # points that exceed the boundaries must be removed
        delete_ids = np.where(np.logical_or(phi_ < 0, phi_ >= width))

        x = np.delete(x, delete_ids)
        y = np.delete(y, delete_ids)
        z = np.delete(z, delete_ids)
        i = np.delete(i, delete_ids)
        r = np.delete(r, delete_ids)
        d = np.delete(d, delete_ids)
        phi_ = np.delete(phi_, delete_ids)

        spherical_projection = np.zeros((height, width, num_channels))

        spherical_projection[(height-1)-r, phi_, 0] = x
        spherical_projection[(height-1)-r, phi_, 1] = y
        spherical_projection[(height-1)-r, phi_, 2] = z
        spherical_projection[(height-1)-r, phi_, 3] = i
        spherical_projection[(height-1)-r, phi_, 4] = d

        return spherical_projection
    
    def hv_in_range(self, x, y, z, fov, fov_type='h'):
        """
        Extract filtered in-range velodyne coordinates based on azimuth & elevation angle limit
        Args:
        `x`:velodyne points x array
        `y`:velodyne points y array
        `z`:velodyne points z array
        `fov`:a two element list, e.g.[-45,45]
        `fov_type`:the fov type, could be `h` or 'v',defualt in `h`
        Return:
        `cond`:condition of points within fov or not
        Raise:
        `NameError`:"fov type must be set between 'h' and 'v' "
        """
        d = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        if fov_type == 'h':
            return np.logical_and(np.arctan2(y, x) > (-fov[1] * np.pi / 180),
                                  np.arctan2(y, x) < (-fov[0] * np.pi / 180))
        elif fov_type == 'v':
            return np.logical_and(np.arctan2(z, d) < (fov[1] * np.pi / 180),
                                  np.arctan2(z, d) > (fov[0] * np.pi / 180))
        else:
            raise NameError("fov type must be set between 'h' and 'v' ")
   
    def rgb_to_float(self, color):
        """ Converts an RGB list to the packed float format used by PCL

            From the PCLdocs:
            "Due to historical reasons (PCL was first developed as a ROS package),
             the RGB information is packed into an integer and casted to a float"

            Args:
                color (list): 3-element list of integers [0-255,0-255,0-255]

            Returns:
                float_rgb: RGB value packed as a float
        """
        hex_r = (0xff & color[0]) << 16
        hex_g = (0xff & color[1]) << 8
        hex_b = (0xff & color[2])

        hex_rgb = hex_r | hex_g | hex_b

        float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

        return float_rgb


    def make_point_field(self):

        msg_pf1 = pc2.PointField()
        msg_pf1.name = str('x')
        msg_pf1.offset = np.int(0)
        msg_pf1.datatype = PointField.FLOAT32
        msg_pf1.count = np.int(1)

        msg_pf2 = pc2.PointField()
        msg_pf2.name = str('y')
        msg_pf2.offset = np.int(4)
        msg_pf2.datatype = PointField.FLOAT32
        msg_pf2.count = np.int(1)

        msg_pf3 = pc2.PointField()
        msg_pf3.name = str('z')
        msg_pf3.offset = np.int(8)
        msg_pf3.datatype = PointField.FLOAT32
        msg_pf3.count = np.int(1)

        msg_pf4 = pc2.PointField()
        msg_pf4.name = str('intensity')
        msg_pf4.offset = np.int(16)
        msg_pf4.datatype = np.int(7)
        msg_pf4.count = np.int(1)

        msg_pf5 = pc2.PointField()
        msg_pf5.name = str('label')
        msg_pf5.offset = np.int(20)
        msg_pf5.datatype = np.int(4)
        msg_pf5.count = np.int(1)

        msg_pf6 = pc2.PointField()
        msg_pf6.name = str('rgb')
        msg_pf6.offset = np.int(28)
        msg_pf6.datatype = PointField.FLOAT32
        msg_pf6.count = np.int(1)
        
        return [msg_pf1, msg_pf2, msg_pf3, msg_pf4, msg_pf5, msg_pf6]
    
    def parse_convert_xml(self, conversion_file_path):
        """
        Parse XML conversion file and compute color_palette 
        """
        defRoot = ET.parse(conversion_file_path).getroot()

        color_to_label = {}

        color_palette = np.zeros((256, 3), dtype=np.uint8)
        class_list = np.ones((256), dtype=np.uint8) * 255
        class_names = np.array(["" for _ in range(256)], dtype='<U25')
        for idx, defElement in enumerate(defRoot.findall("SLabel")):
            from_color = np.fromstring(defElement.get("fromColour"), dtype=int, sep=" ")
            to_class = np.fromstring(defElement.get("toValue"), dtype=int, sep=" ")
            class_name = defElement.get('Name').lower()
            if to_class in class_list:
                color_to_label[tuple(from_color)] = int(to_class)
            else:
                color_palette[idx] = from_color
                class_list[idx] = to_class
                class_names[idx] = class_name
                color_to_label[tuple(from_color)] = int(to_class)

        # Sort classes accoring to is train ID
        sort_indexes = np.argsort(class_list)

        class_list = class_list[sort_indexes]
        class_names = class_names[sort_indexes]
        color_palette = color_palette[sort_indexes]

        return color_palette, class_names, color_to_label
    
    def load_parameters(self):
        """
        Load ROS parameters and store them
        """
        self.get_logger().info("Loading parameters ...")

        # get the directory that this script is in
        script_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),os.pardir)
        self.get_logger().info(script_dir)
        

        # get the filename from the parameter and append it to the script directory
        model_path_file = self.get_parameter('model_path').get_parameter_value().string_value
        self.model_path = os.path.join(script_dir, model_path_file)

        palette_file = self.get_parameter('palette_file').get_parameter_value().string_value
        self.palette_file_path = os.path.join(script_dir, palette_file)


        self.do_visualizations = self.get_parameter('do_visualizations').get_parameter_value().bool_value
        self.num_classes = self.get_parameter('num_classes').get_parameter_value().integer_value
        self.left_azimuth = self.get_parameter('left_azimuth').get_parameter_value().double_value
        self.right_azimuth = self.get_parameter('right_azimuth').get_parameter_value().double_value
        self.num_rings = self.get_parameter('num_rings').get_parameter_value().integer_value
        self.num_azimuth = self.get_parameter('num_azimuth').get_parameter_value().integer_value
        self.normalization_mean = np.array([[self.get_parameter("normalization_mean").get_parameter_value().double_array_value]], dtype=np.float32)
        self.normalization_std = np.array([[self.get_parameter("normalization_std").get_parameter_value().double_array_value]], dtype=np.float32)
          
        # load one hot encoding
        self.color_palette, self.class_names, self.color_to_label = self.parse_convert_xml(self.palette_file_path)

    def setup(self):

        # load inference model
        self.model = tf.keras.models.load_model(self.model_path)

        # create point field for cloud_creator
        self.point_field = self.make_point_field()

        # create publisher for passing on depth estimation and camera info      
        self.pub_seg = self.create_publisher(PointCloud2, "/points2_segmented", 1)
        # listen for input image and camera info
        self.sub_pcl = self.create_subscription(PointCloud2, "/points2", self.predict, 1)


    def __init__(self):
       
        # initialize ROS node
        super().__init__('camera_segmentation')
        
        # initialize ROS node
        self.get_logger().info("Initializing pointcloud_segmentation node...")

        # decleare parameters
        self.declare_parameter('model_path') 
        self.declare_parameter('palette_file') 
        self.declare_parameter('do_visualizations') 
        self.declare_parameter('num_classes') 
        self.declare_parameter('left_azimuth') 
        self.declare_parameter('right_azimuth')
        self.declare_parameter('num_rings')
        self.declare_parameter('num_azimuth')
        self.declare_parameter('normalization_mean')
        self.declare_parameter('normalization_std') 

        # load parameters
        self.load_parameters()
 
        # setup components
        self.setup()

def main(args=None):
    rclpy.init(args=args)

    vision = PCLSegmentation()

    # keep node from exiting
    rclpy.spin(vision)
    
    #ROS2 needs .destroy_node after spinning
    vision.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()