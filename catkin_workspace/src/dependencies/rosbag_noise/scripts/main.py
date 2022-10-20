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

import random
import sys
from copy import deepcopy

import rosbag
from definitions.msg import IkaObjectList, IkaSensorStamp


# to execute this script, build the catkin package with its dependencies.
# then execute:
#  rosrun rosbag_noise main.py bag_file.bag

def make_noise(object_list, mode):
    noise_list = IkaObjectList()
    noise_list.header = object_list.header

    if mode == 'camera':
        noise_list.IdSource = 16  # CAMERA
    elif mode == 'radar':
        noise_list.IdSource = 14  # RADAR

    for reference_object in object_list.objects:
        obj = deepcopy(reference_object)
        if mode == 'camera':
            obj.fMean = list(obj.fMean)
            obj.fMean[0] += random.gauss(0, 0.6)
            obj.fMean[1] += random.gauss(0, 0.2)
            obj.IdType = 4  # CAR
            obj.IdExternal = 16  # CAMERA
            obj.bObjectMeasured = 1

            obj.fCovariance = list(obj.fCovariance)
            obj.fCovariance[0] = 0.36
            obj.fCovariance[12] = 0.04
            obj.fCovariance[24] = 0.01
            obj.fCovariance[36] = 0.01
            obj.fCovariance[48] = 0.01
            obj.fCovariance[60] = 0.01
            obj.fCovariance[72] = 0.01
            obj.fCovariance[84] = 0.01
            obj.fCovariance[96] = 0.01
            obj.fCovariance[108] = 0.01
            obj.fCovariance[120] = 0.01

            sensor_stamp = IkaSensorStamp()
            sensor_stamp.IdSensor = obj.IdExternal
            sensor_stamp.IdObjectWithinSensor = obj.IdInternal
            sensor_stamp.measuredStamp = object_list.header.stamp
            obj.measHist.append(sensor_stamp)

        elif mode == 'radar':
            obj.fMean = list(obj.fMean)
            obj.fMean[0] += random.gauss(0, 0.2)
            obj.fMean[1] += random.gauss(0, 0.6)
            obj.IdType = 4  # CAR
            obj.IdExternal = 14  # RADAR
            obj.fMean[7] -= 1.5
            obj.fMean[8] -= 1
            obj.bObjectMeasured = 1

            obj.fCovariance = list(obj.fCovariance)
            obj.fCovariance[0] = 0.04
            obj.fCovariance[12] = 0.36
            obj.fCovariance[24] = 0.01
            obj.fCovariance[36] = 0.01
            obj.fCovariance[48] = 0.01
            obj.fCovariance[60] = 0.01
            obj.fCovariance[72] = 0.01
            obj.fCovariance[84] = 1.2
            obj.fCovariance[96] = 1.0
            obj.fCovariance[108] = 0.01
            obj.fCovariance[120] = 0.01

            sensor_stamp = IkaSensorStamp()
            sensor_stamp.IdSensor = obj.IdExternal
            sensor_stamp.IdObjectWithinSensor = obj.IdInternal
            sensor_stamp.measuredStamp = object_list.header.stamp
            obj.measHist.append(sensor_stamp)

        noise_list.objects.append(obj)

    return noise_list


def main():
    input_name = sys.argv[1]
    input_bag = rosbag.Bag(input_name, 'r')
    input_objectlist_topic = "/fusion/ikaObjectList"
    input_objectlist_topic_output = "/sensors/reference/ikaObjectList"

    output_name = sys.argv[1][:-4] + '_noise.bag'
    bag = rosbag.Bag(output_name, 'w')
    output_objectlist_topic_camera = '/sensors/camera_front/ikaObjectList'
    output_objectlist_topic_radar = '/sensors/radar_front/ikaObjectList'

    try:
        for topic, msg, t in input_bag.read_messages():
            if topic == input_objectlist_topic:
                bag.write(input_objectlist_topic_output, msg, t)
                # bag.write('/clock', msg.header.stamp, t)

                estimated_objects_camera = make_noise(msg, 'camera')
                bag.write(output_objectlist_topic_camera, estimated_objects_camera, t)

                estimated_objects_radar = make_noise(msg, 'radar')
                bag.write(output_objectlist_topic_radar, estimated_objects_radar, t)
            else:
                bag.write(topic, msg, t)
    finally:
        bag.close()

    print("saved to " + output_name)


if __name__ == '__main__':
    main()
