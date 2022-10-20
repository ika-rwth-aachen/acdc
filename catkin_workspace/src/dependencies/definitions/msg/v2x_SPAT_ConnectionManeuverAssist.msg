uint8 connectionID
uint16 queueLength
bool queueLength_present
uint16 availableStorageLength
bool availableStorageLength_present
bool waitOnStop
bool waitOnStop_present
bool pedBicycleDetect
bool pedBicycleDetect_present

#RegionalConnectionManeuverAssist regional
    v2x_SPAT_VehicleToLanePosition[] regional_vehicleToLanePositions
    #NodeOffsetPoint OPTIONAL
        uint8 regional_rsuDistanceFromAnchor_choice
        #not used choice=0 
        #Node-XY-20b choice=20
        #Node-XY-22b choice=22
        #Node-XY-24b choice=24
        #Node-XY-26b choice=26
        #Node-XY-28b choice=28
        #Node-XY-32b choice=32
        #Node-LLmD-64b choice=64
          
        int32 regional_rsuDistanceFromAnchor_node_x_lat
        int32 regional_rsuDistanceFromAnchor_node_y_lon
        #unused
        #node-Regional