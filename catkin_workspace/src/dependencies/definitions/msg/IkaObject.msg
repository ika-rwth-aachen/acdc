std_msgs/Header header

# Motion Model Attributes
uint8 IdMotionModel				 # Motionmodel type 0=UNKNOWN; 1=CV; 2=CA; 3=CTRV; 4=CTRA
float32[] fMean                  # Statevector, containing attributes depend on chosen motion model
float32[] fCovariance            # Covariance-Matrix, containing attributes depend on chosen motion model

# additional relative properties
# reference is moving base_link instead of zero velocity base_link
float32 fRelVelX
float32 fRelVelY
float32 fRelAccX
float32 fRelAccY			

# Reference Point
uint8 RefLocation				# Sensor's object reference location
float32 fRefX					# X-Position of the reference point
float32 fRefY					# Y-Position of the reference point

# Object Meta Data
uint16 IdInternal               # Internal ID of the object inside the sensor.
int16 IdLane                    # Id of the lane the car is driving on. usually not used.
uint16 IdType                   # Type of the detected object.
float32[] fClassProbability		# Vector of class-wise probabilities [0..1] sum of all entries is always 1
float32 fExistenceProbability   # [0..1]

# Measurement Meta Data
uint16 IdExternal               # Source ID of the sensor.
IkaSensorStamp[] measHist       # History of measurements of this object.
time birthStamp                 # ROS time stamp of first occurrence of object
bool bObjectValid               # true if object contains any relevant information. false for gaps in objects vector.
bool bObjectNew                 # true if object got measured for the first time in the current time stamp.
bool bObjectMeasured            # true if object is being measured by a sensor in the current time stamp.

#for enum types see definitions/include/definitions/utility/object_definitions.h