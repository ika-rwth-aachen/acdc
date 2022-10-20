# CAM.msg
#
# Contains all transmittet cooperative awareness message information
# Values of data type float64 (aka double) are in si units (but deg is used instead of radiant!)
#

# ItsPduHeader
    uint8 header_protocolVersion
    uint8 header_messageID
    uint16 header_stationID  # stationID will be overwritten by the asn encoder
# CoopAwareness
    float64 cam_generationDeltaTime
    v2x_CAM_basic_container basic_container         # BasicContainer
    v2x_CAM_high_freq_container high_freq_container # HighFrequencyContainer
    v2x_CAM_low_freq_container low_freq_container   # LowFrequencyContainer (OPTIONAL)
    v2x_CAM_special_container special_container     # SpecialVehicleContainer (OPTIONAL)