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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import os
from .img_utils import resize_image
import xml.etree.ElementTree as ET

WITH_TF = True
try:
    import tensorflow as tf
except:
    WITH_TF = False

class ImageSegmentation(Node):

    def predict(self, img_color_msg):
        t0 = time.time()
        input_img = self.cv_bridge.imgmsg_to_cv2(img_color_msg, desired_encoding="rgb8")
        input_img = resize_image(input_img, [self.resize_height, self.resize_width])
        input_img = input_img[None]
        t1 = time.time()
        predictions = self.frozen_func(tf.cast(input_img, tf.uint8))
        t2 = time.time()
        prediction = tf.squeeze(predictions).numpy()
        prediction = self.segmentation_map_to_rgb(prediction).astype(np.uint8)
        seg_msg = self.cv_bridge.cv2_to_imgmsg(prediction, encoding="rgb8")
        seg_msg.header = img_color_msg.header
        t3 = time.time()
        #self.get_logger().info("(prep) %fs | (pred) %fs | (post) %fs | (total) %fs", t1-t0, t2-t1, t3-t2, t3-t0)
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

        self.frozen_func = self.wrap_frozen_graph(graph_def=graph_def,
                                                  inputs=[self.input_tensor_name],
                                                  outputs=[self.output_tensor_name],
                                                  print_graph=True)

    def segmentation_map_to_rgb(self, segmentation_map):

        ### START CODE HERE ### 

        # Solution:
        # Apply the color palette to the segmentation map. self.color_palette has
        # shape [256, 3] and if for example access positon 0 in this array, we obtain
        # the RGB color code for the first class. Thus, we can apply the hole segmentation
        # maps to self.color_palette in order to convert the class id in the map to 
        # the color coding
        rgb_encoding = self.color_palette[segmentation_map]

        ### END CODE HERE ###

        return rgb_encoding

    def parse_convert_xml(self, conversion_file_path):
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
        self.declare_parameter('frozen_graph')
        self.declare_parameter('xml_conversion_file')
        self.declare_parameter('resize_width', 1936)
        self.declare_parameter('resize_height', 1216)

        # self.frozen_graph = self.get_parameter('frozen_graph').get_parameter_value().string_value
        self.frozen_graph = "/home/rosuser/ws/ros2_ws/src/image_segmentation_r2/models/mobilenet_v3_large_968_608_os8.pb"
        # self.xml_conversion_file = self.get_parameter('xml_conversion_file').get_parameter_value().string_value
        self.xml_conversion_file = "/home/rosuser/ws/ros2_ws/src/image_segmentation_r2/models/convert_cityscapes_to_ika_reduced.xml"
        self.resize_width = self.get_parameter('resize_width').get_parameter_value().integer_value
        self.resize_height = self.get_parameter('resize_height').get_parameter_value().integer_value

        self.color_palette, self.class_names, self.color_to_label = self.parse_convert_xml(self.xml_conversion_file)

    def setup(self):
        self.cv_bridge = CvBridge()
        self.load_frozen_graph(self.frozen_graph)
        self.pub_seg = self.create_publisher(Image, "/image_segmented", 10)
        self.sub_image = self.create_subscription(Image, "/image_color", self.predict, 10)

    def __init__(self):
        super().__init__('camera_segmentation')

        if WITH_TF:
            self.load_parameters()
            self.setup()

def main(args=None):
    rclpy.init(args=args)

    vision = ImageSegmentation()

    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()