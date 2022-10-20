string name 
bool name_present
#IntersectionReferenceID id
    uint16 id_region
    bool id_region_present
    uint16 id_id
uint8 revision
ASN_bitstring status
uint32 moy
bool moy_present
uint32 timeStamp
bool timeStamp_present
#uint8[] enabledLanes #alternative 
v2x_SPAT_LaneID[] enabledLanes
bool enabledLanes_present
v2x_SPAT_MovementState[] states#enumerated
v2x_SPAT_ConnectionManeuverAssist[] maneuverAssistList
bool maneuverAssistList_present
uint8[] priority#octet string
bool priority_present
uint8[] preempt#octet string
bool preempt_present
v2x_SPAT_PrioritizationResponse[] regional #enumerated
bool regional_present