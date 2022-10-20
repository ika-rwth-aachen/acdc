// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <definitions/utility/ika_utilities.h>
#include <definitions/IkaObjectList.h>

/**
 * @brief Publisher for the transformed objects.
 */
ros::Publisher *publisher_objects_ = nullptr;

/**
 * @brief Buffer for the transformation itself.
 */
tf2_ros::Buffer *transform_buffer_ = nullptr;

/**
 * @brief Id of the source frame.
 */
std::string source_frame_id;

/**
 * @brief Id of the target frame.
 */
std::string target_frame_id;

/**
 * @brief Callback function for the objects in the source frame. Will publish the same objects on the new frame.
 * @param msg Message in source frame.
 */
void callbackIkaObjects(const definitions::IkaObjectList &msg) {
  // Get tf2 transformation
  geometry_msgs::TransformStamped transform =
    transform_buffer_->lookupTransform(target_frame_id, source_frame_id, ros::Time(0));

  // Copy the list to make it editable
  definitions::IkaObjectList buffer_transformed_objects_list = msg;
  // Update frame id of list to the new frame
  buffer_transformed_objects_list.header.frame_id = target_frame_id;
  // Transform the individual objects
  for (auto &object : buffer_transformed_objects_list.objects) {
    IkaUtilities::transformIkaObject(&object, &transform);
  }
  // Publish new objects
  publisher_objects_->publish(buffer_transformed_objects_list);
}

int main(int argc, char *argv[]) {
  // Initialise the new node
  ros::init(argc, argv, "transformation", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle;
  ROS_INFO("Transformation started.");

  // Set default read and write topics
  std::string default_subscribe_topic_objects = "/fusion/ikaObjectList";
  std::string default_publish_topic_objects = "/vehicle/ikaObjectList";
  std::string default_source_frame = "base_link";
  std::string default_target_frame = "vehicle_body";

  std::string subscribe_topic_objects;
  std::string publish_topic_objects;

  // Get read and write targets from launch file parameter
  ros::param::get("~source_topic", subscribe_topic_objects);
  ros::param::get("~target_topic", publish_topic_objects);
  ros::param::get("~source_frame_id", source_frame_id);
  ros::param::get("~target_frame_id", target_frame_id);

  ROS_INFO("Transformation subscribes to: %s", subscribe_topic_objects.c_str());
  ROS_INFO("Transformation publishes to: %s", publish_topic_objects.c_str());
  ROS_INFO("Transformation source from: %s", publish_topic_objects.c_str());
  ROS_INFO("Transformation target to: %s", publish_topic_objects.c_str());

  // Initiate global variables
  publisher_objects_ = new ros::Publisher;
  transform_buffer_ = new tf2_ros::Buffer;

  // Define publisher and subscriber
  ros::Subscriber subscriber_objects = node_handle.subscribe(subscribe_topic_objects, 10, callbackIkaObjects);
  tf2_ros::TransformListener transform_listener_recording_simulation_(*transform_buffer_);
  *publisher_objects_ = node_handle.advertise<definitions::IkaObjectList>(publish_topic_objects, 10);

  // Prevent ros from termination
  ROS_INFO("Transformation is running...");
  ros::spin();

  // Clean global variables
  delete publisher_objects_;
  delete transform_buffer_;
  return 0;
}
