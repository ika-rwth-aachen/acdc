# SPAT.msg
#
# 
# Values of data type float64 (aka double) are in si units (but deg is used instead of radiant!)
#

#ItsPduHeader header
    uint8 header_protocolVersion
    uint8 header_messageID
    uint32 header_stationID
#SPAT spatData
    uint8 spatData_msgID
    uint8 spatData_msgSubID
    bool spatData_msgSubID_present
    string spatData_name
    bool spatData_name_present
    v2x_SPAT_IntersectionState[] spatData_intersections
    
    #unsed
    #RegionalSPAT_regional
    #uint8[] spatData_regional unused
    bool spatData_regional_present