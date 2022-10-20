#enum directionalUse
uint8 LaneDirection_ingressPath = 0
uint8 LaneDirection_egressPath = 1
###############################################################

#general laneInformation 
    uint8 laneId
    uint8 directionalUse
    uint8 laneType  # Values as defined in SAE J 2735, "LaneTypeAttributes"
                    # 0: Vehicle, 2: Bike Lane

#coordinates of lane as flat earth projection from refPoint
    geometry_msgs/Point[] lane_coordinates

#list of lane connections
    v2x_MAP_Connection[] connections
