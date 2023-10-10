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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import os
from ament_index_python.packages import get_package_share_directory
from .img_utils import resize_image
import xml.etree.ElementTree as ET

WITH_TF = True
try:
    import tensorflow as tf
except:
    WITH_TF = False
    print("%s will shutdown because it was not compiled with TensorFlow")

class ImageSegmentation(Node):

    def predict(self, img_color_msg):
        t0 = time.time()

        # convert message to cv2-image
        input_img = self.cv_bridge.imgmsg_to_cv2(img_color_msg, desired_encoding="rgb8")

        # resize image
        input_img = resize_image(input_img, [self.resize_height, self.resize_width])
        
        # append batch dimension
        input_img = input_img[None]
        t1 = time.time()

        # perform semantic segmentation
        predictions = self.frozen_func(tf.cast(input_img, tf.uint8))

        t2 = time.time()

        # remove batch dimension
        prediction = tf.squeeze(predictions).numpy()

        # decode image to RGB
        prediction = self.segmentation_map_to_rgb(prediction).astype(np.uint8)

        # convert output back to message
        seg_msg = self.cv_bridge.cv2_to_imgmsg(prediction, encoding="rgb8")

        # assign header of input msg
        seg_msg.header = img_color_msg.header
        
        t3 = time.time()

        # log processing duration
        time_diffs = (t1 - t0, t2 - t1, t3 - t2, t3 - t0)
        log_message = "\n (prep) {:.2f}s | (pred) {:.2f}s | (post) {:.2f}s |(total) {:.2f}s".format(*time_diffs)
        self.get_logger().info(log_message)

        self.pub_seg.publish(seg_msg)

    @staticmethod
    def wrap_frozen_graph(graph_def, inputs, outputs, print_graph=False):
        def _imports_graph_def():
            tf.compat.v1.import_graph_def(graph_def, name="")

        wrapped_import = tf.compat.v1.wrap_function(_imports_graph_def, [])
        import_graph = wrapped_import.graph

        return wrapped_import.prune(
            tf.nest.map_structure(import_graph.as_graph_element, inputs),
            tf.nest.map_structure(import_graph.as_graph_element, outputs))

    def load_frozen_graph(self, path_to_frozen_graph):
        self.sess = None
        self.graph = tf.Graph()

        self.input_tensor_name = 'input:0'
        self.output_tensor_name = 'prediction:0'

        with tf.io.gfile.GFile(path_to_frozen_graph, 'rb') as file_handle:
            graph_def = tf.compat.v1.GraphDef()
            loaded = graph_def.ParseFromString(file_handle.read())
        
        # Wrap frozen graph to ConcreteFunctions
        self.frozen_func = self.wrap_frozen_graph(graph_def=graph_def,
                                                  inputs=[self.input_tensor_name],
                                                  outputs=[self.output_tensor_name],
                                                  print_graph=True)

    def segmentation_map_to_rgb(self, segmentation_map):
        """
        Converts segmentation map to a RGB encoding according to self.color_palette
        Eg. 0 (Class 0) -> Pixel value [128, 64, 128] which is on index 0 of self.color_palette
            1 (Class 1) -> Pixel value [244, 35, 232] which is on index 1 of self.color_palette

        self.color_palette has shape [256, 3]. Each index of the first dimension is associated
        with an RGB value. The index corresponds to the class ID.

        :param segmentation_map: ndarray numpy with shape (height, width)
        :return: RGB encoding with shape (height, width, 3)
        """

        ### START CODE HERE ###
        
        # Task 1:
        # Replace the following command
        rgb_encoding = np.random.randint(
            low=0,
            high=255,
            size=[self.resize_height, self.resize_width, 3]
        )

        ### END CODE HERE ###

        return rgb_encoding

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
        package_dir = get_package_share_directory('image_segmentation_r2')
        
        # get the filename from the parameter and append it to the script directory
        frozen_graph_file = self.get_parameter('frozen_graph').get_parameter_value().string_value
        self.frozen_graph = os.path.join(package_dir,  frozen_graph_file)

        xml_conversion_file = self.get_parameter('xml_conversion_file').get_parameter_value().string_value
        self.path_xml_conversion_file = os.path.join(package_dir, xml_conversion_file)

        self.resize_width = self.get_parameter('resize_width').get_parameter_value().integer_value
        self.resize_height = self.get_parameter('resize_height').get_parameter_value().integer_value

        # self load one hot encoding
        self.color_palette, self.class_names, self.color_to_label = self.parse_convert_xml(self.path_xml_conversion_file)

    def setup(self):

        # create cv2-msg converter bridge
        self.cv_bridge = CvBridge()
        # load frozen graph
        self.load_frozen_graph(self.frozen_graph)
        # create publisher for passing on depth estimation and camera info      
        self.pub_seg = self.create_publisher(Image, "/image_segmented", 10)
        # listen for input image and camera info
        self.sub_image = self.create_subscription(Image, "/image_color", self.predict, 10) # buff_size = 500 MB

    def __init__(self):
        # initialize ROS node
        super().__init__('camera_segmentation')

        # initialize ROS node
        self.get_logger().info("Initializing camera_segmentation node...")

        self.declare_parameter('frozen_graph', 'default_value')
        self.declare_parameter('xml_conversion_file', 'default_value')
        self.declare_parameter('resize_width', 1936)
        self.declare_parameter('resize_height', 1216)

        if WITH_TF:

            # load parameters
            self.load_parameters()
 
            # setup components
            self.setup()

def main(args=None):
    rclpy.init(args=args)

    vision = ImageSegmentation()

    # keep node from exiting
    rclpy.spin(vision)
    
    #ROS2 needs .destroy_node after spinning
    vision.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()