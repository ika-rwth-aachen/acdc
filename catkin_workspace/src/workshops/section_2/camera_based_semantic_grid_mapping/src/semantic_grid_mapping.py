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

import rospy
import tf2_ros
import tf_conversions 
# Messages
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
# Synchronization
import message_filters
# OpenCV 
import cv2
from cv_bridge import CvBridge

import numpy as np

class IPM():
    def __init__(self) -> None:
        
        # Load parameters (dst path for images, input topic)
        self.load_parameters()
        
        # setup subscribers
        subs = []  # array with all subscribers that should be synchronized
        for image_topic, info_topic in zip(self.image_topics_in, self.info_topics_in):
            ### START Task 3 CODE HERE ###
            # create subscriber for topic
            image_sub = message_filters.Subscriber(image_topic, Image, queue_size=1)
            # create a subscriber for camera info topic
            info_sub = message_filters.Subscriber(info_topic, CameraInfo, queue_size=1)
            ### END Task 3 CODE HERE ###
            # add subscribers to array
            subs.append(image_sub)
            subs.append(info_sub)
        
        # synchronized subscriber
        self.sync_sub = message_filters.ApproximateTimeSynchronizer(subs, queue_size=5., slop=0.01)
        # Register Callback
        self.sync_sub.registerCallback(self.compute_bev) 

        # initialize publisher
        self.pub = rospy.Publisher('/BEV_image', Image, queue_size=5)

        # tf listener for coordinates transformations
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # create a cv_bridge to convert between ros and opencv images
        self.cv_bridge = CvBridge()

    def apply_ipm(self, image, K, E):
        # parameters for ipm
        # output resolution
        px_per_m = self.config["px_per_m"] 
        # output size
        width = self.config["output_width"]
        height = self.config["output_height"]
        # shift to center of the left edge of output image
        shift_x = self.config["shift_x"]
        shift_y = self.config["shift_y"]
        # image height and width
        input_img_height, input_img_width, _ = image.shape
        # mask upper half of the image  
        mask = np.zeros(image.shape[:2], dtype="uint8")
        cv2.rectangle(mask, (0, input_img_height//2 ), (input_img_width, input_img_height),  255, -1)
        image = cv2.bitwise_and(image, image, mask=mask)
        
        # define matrix that maps from the road frame to the vehicle frame
        M_2D_to_3D = np.array([[1.0, 0., 0.],
                        [0.0, 1.0, 0.],
                        [0.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0]])
        M_direction = np.array([[1.0, 0., 0.],
                        [0.0, -1.0, 0.],
                        [0.0, 0.0, 1.0]])
        M_shift = np.array([[1.0, 0., - shift_x],
                        [0.0, 1.0, -shift_y],
                        [0.0, 0.0, 1.0]])
        M_scale = np.array([[1.0/ px_per_m, 0., 0],
                        [0.0, 1.0/ px_per_m, 0],
                        [0.0, 0.0, 1.0]])
        
        M = (M_2D_to_3D).dot(M_scale).dot(M_direction.dot(M_shift))
        
        # define projection matrix
        P = K.dot(E[:-1,:])
        # calculate inverse perspective mapping matrix
        M_ipm = np.linalg.inv(P.dot(M))
        # apply perspective warping
        img_out = cv2.warpPerspective(image, M_ipm, (width, height), cv2.INTER_AREA)
        
        return img_out

        
    def compute_bev(self, *args):
        """
        applies IPM to multiple images and stitches the result into a BEV. 
        """
        images_with_info = [(args[i], args[i+1]) for i in range(0, len(args), 2)]
        # initialize output
        bev_total_img = np.zeros((self.config['output_height'], self.config['output_width'],3), np.uint8)
        # apply IPM to each Image using information from the corresponding CameraInfo messages
        common_time = rospy.Time(0)
        for image_msg, cam_info_msg in images_with_info:
            
            ### START Task 5, Part 1 CODE HERE ###
            # use tfBuffer to look up the transformation from the vehicle's base link frame to the camera frame.
            transform = self.tfBuffer.lookup_transform(cam_info_msg.header.frame_id, self.vehicle_base_link, common_time)
            ### END Task 5, Part 1 CODE HERE ###
            
            ### START Task 5, Part 2 CODE HERE ###
            # extract quaternion from transform and transform it into a list
            quaternion = transform.transform.rotation # extract from transform
            quaternion = [quaternion.x, quaternion.y , quaternion.z, quaternion.w] # convert to list

            # convert quaternion to (roll,pitch,yaw)
            roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(quaternion)          

            # compute rotation matrix
            Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw), np.cos(yaw), 0.0],
                [0.0, 0.0, 1.0]])
            Ry = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                [0.0, 1.0, 0.0],
                [-np.sin(pitch), 0.0, np.cos(pitch)]])
            Rx = np.array([[1.0, 0.0, 0.0],
                    [0.0, np.cos(roll), -np.sin(roll)],
                    [0.0, np.sin(roll), np.cos(roll)]])
            R = Rz.dot(Ry.dot(Rx))


            # extract translation from transform
            t = transform.transform.translation
            t = [t.x, t.y, t.z]

            # convert t to a numpy array
            t = np.array(t)

            # combine translation (3x1) and rotation matrix (3x3) into a 4x4 homogeneous transform
            # representing the extrinsic matrix E
            # first combine R and t
            E = np.column_stack([R,t]) # uncomment and adjust
            # then add 1 row ([0., 0., 0., 1.]) to complete the transform
            E = np.row_stack([E, np.array([0.,0.,0.,1.])]) # uncomment and adjust

            ### END Task 5, Part 2 CODE HERE ###


            ### START Task 4 CODE HERE ###
            # extract intrinsic matrix K (3x3) from camera info topic
            K = np.reshape(cam_info_msg.K, (3,3)) # replace the placeholder and use the actual camera intrinsics
            ### END Task 4 CODE HERE ###

            # decode image
            image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
		
            output_image = self.apply_ipm(image, K, E)
            # adjust BEV image
            bev_total_img[bev_total_img==(0,0,0)] = output_image [bev_total_img==(0,0,0)]
        # publish result
        msg = self.cv_bridge.cv2_to_imgmsg(bev_total_img, encoding="bgr8") 
        self.pub.publish(msg)
    
    
    def load_parameters(self):
        self.image_topics_in = rospy.get_param("~image_topics_in")
        self.info_topics_in = rospy.get_param("~info_topics_in")
        self.vehicle_base_link = rospy.get_param("~vehicle_base_link")

        config = {}
        config["px_per_m"] = rospy.get_param("~px_per_m") # number of pixels per meter
        config["output_width"] = rospy.get_param("~output_width") 
        config["output_height"] = rospy.get_param("~output_height") 
        # shift to center of output image
        config["shift_x"] = config["output_width"] / 2.0 
        config["shift_y"] = config["output_height"] /2.0
        self.config = config

if __name__ == '__main__':
    # initialize node
    rospy.init_node('inverse_perspective_mapping')
    bad_data = IPM()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node!")


