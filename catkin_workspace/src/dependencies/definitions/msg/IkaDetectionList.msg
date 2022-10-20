std_msgs/Header header

# List meta information
uint8 IdSource  #see definitions/utility/sensor_definitions.h for enum of Sensors
uint8 IdInternal

# Actually detections
IkaDetection[] detections
