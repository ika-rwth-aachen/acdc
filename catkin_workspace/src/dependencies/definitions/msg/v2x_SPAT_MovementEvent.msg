uint8 eventState#states
#TimeChangeDetails timing OPTIONAL
    uint16 timing_startTime
    bool timing_startTime_present
    uint16 timing_minEndTime
    uint16 timing_maxEndTime
    bool timing_maxEndTime_present
    uint16 timing_likelyTime
    bool timing_likelyTime_present
    uint8 timing_confidence
    bool timing_confidence_present
    uint16 timing_nextTime
    bool timing_nextTime_present
bool timing_present
v2x_SPAT_AdvisorySpeed[] speeds#enumerated
bool speeds_present
# regional unused
bool regional_present









