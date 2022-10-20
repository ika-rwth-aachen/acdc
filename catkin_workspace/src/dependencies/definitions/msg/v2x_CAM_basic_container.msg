# BasicContainer
    uint8 stationType
    # Reference Position
        float64 referencePosition_latitude
        float64 referencePosition_longitude
        # PosConfidenceEllipse
            uint16 referencePosition_positionConfidenceEllipse_semiMajorConfidence
            uint16 referencePosition_positionConfidenceEllipse_semiMinorConfidence
            uint16 referencePosition_positionConfidenceEllipse_semiMajorOrientationheadingValue
        # Altitude
            int16 referencePosition_altitude_altitudeValue
            uint8 referencePosition_altitude_altitudeConfidence