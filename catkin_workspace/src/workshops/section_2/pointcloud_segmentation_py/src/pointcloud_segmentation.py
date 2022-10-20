#!/usr/bin/env python

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
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
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
import rospy
import tensorflow as tf
import cv2

import sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge
import struct
import xml.etree.ElementTree as ET


class PCLSegmentation:
    """
    Implements a point cloud segmentation node.
    """

    def predict(self, pcl_msg):
        pcl = np.array(list(sensor_msgs.point_cloud2.read_points(pcl_msg)))

        # perform fov filter by using hv_in_range
        cond = self.hv_in_range(x=pcl[:, 0],
                                y=pcl[:, 1],
                                z=pcl[:, 2],
                                fov=[-self.left_azimuth, self.right_azimuth])

        # apply fov filter
        pcl = pcl[cond]

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

            From the PCL docs:
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
        msg_pf1.offset = np.uint32(0)
        msg_pf1.datatype = PointField.FLOAT32
        msg_pf1.count = np.uint32(1)

        msg_pf2 = pc2.PointField()
        msg_pf2.name = str('y')
        msg_pf2.offset = np.uint32(4)
        msg_pf2.datatype = PointField.FLOAT32
        msg_pf2.count = np.uint32(1)

        msg_pf3 = pc2.PointField()
        msg_pf3.name = str('z')
        msg_pf3.offset = np.uint32(8)
        msg_pf3.datatype = PointField.FLOAT32
        msg_pf3.count = np.uint32(1)

        msg_pf4 = pc2.PointField()
        msg_pf4.name = str('intensity')
        msg_pf4.offset = np.uint32(16)
        msg_pf4.datatype = np.uint8(7)
        msg_pf4.count = np.uint32(1)

        msg_pf5 = pc2.PointField()
        msg_pf5.name = str('label')
        msg_pf5.offset = np.uint32(20)
        msg_pf5.datatype = np.uint8(4)
        msg_pf5.count = np.uint32(1)

        msg_pf6 = pc2.PointField()
        msg_pf6.name = str('rgb')
        msg_pf6.offset = np.uint32(28)
        msg_pf6.datatype = PointField.FLOAT32
        msg_pf6.count = np.uint32(1)
        return [msg_pf1, msg_pf2, msg_pf3, msg_pf4, msg_pf5, msg_pf6]


    def load_parameters(self):
        rospy.loginfo("Loading parameters ...")

        package_dir = os.path.join(os.path.dirname(__file__), os.pardir)

        prefix = "pointcloud_segmentation/"
        self.model_path               = os.path.join(package_dir, rospy.get_param(prefix + "model_path"))
        self.path_palette_file        = os.path.join(package_dir, rospy.get_param(prefix + "palette_file"))

        self.do_visualizations          = rospy.get_param(prefix + "do_visualizations")
        self.num_classes                = rospy.get_param(prefix + "num_classes")
        self.left_azimuth               = rospy.get_param(prefix + "left_azimuth")
        self.right_azimuth              = rospy.get_param(prefix + "right_azimuth")
        self.num_rings                  = rospy.get_param(prefix + "num_rings")
        self.num_azimuth                = rospy.get_param(prefix + "num_azimuth")
        self.normalization_mean         = np.array([[rospy.get_param(prefix + "normalization_mean")]], dtype=np.float32)
        self.normalization_std          = np.array([[rospy.get_param(prefix + "normalization_std")]], dtype=np.float32)

        # load one hot encoding
        self.color_palette, self.class_names, self.color_to_label = self.parse_convert_xml(self.path_palette_file)


    def setup(self):
        # load inference model
        self.model = tf.keras.models.load_model(self.model_path)

        # create point field for cloud_creator
        self.point_field = self.make_point_field()
        

        ### START TASK 1 CODE HERE ###

        # Task 1:
        # create publisher for the segmented point cloud, publish on topic "/points2_segmented"
        # The publish type is the PointCloud2 data format


        ### END TASK 1 CODE HERE ###

        # listen for input image and camera info
        self.sub_pcl = rospy.Subscriber("/points2", PointCloud2, self.predict, queue_size=1)


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


    def __init__(self):
        # initialize ROS node
        rospy.init_node("pointcloud_segmentation_py")

        # load parameters
        self.load_parameters()

        # setup components
        self.setup()

        # keep node from exiting
        rospy.spin()


if __name__ == "__main__":
    pcl_segmentation = PCLSegmentation()
