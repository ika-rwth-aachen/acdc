# SpecialVehicleContainer (OPTIONAL)
uint8 containerType # 0 = Absent, 1 = publicTransportContainer, 2 = ...
    # PublicTransportContainer
        bool publicTransportContainer_embarkationStatus
        # PtActivation (OPTIONAL)
        bool publicTransportContainer_ptActivationPresent
            uint8 publicTransportContainer_ptActivation_ptActivationType
            uint8[] publicTransportContainer_ptActivation_ptActivationData
    # SpecialTransportContainer
        ASN_bitstring specialTransportContainer_specialTransportType
        ASN_bitstring specialTransportContainer_lightBarSirenInUse
    # DangerousGoodsContainer
        uint8 dangerousGoodsContainer_dangerousGoodsBasic
    # RoadWorksContainerBasic
        bool roadWorksContainerBasic_roadworksSubCauseCodePresent
        uint8 roadWorksContainerBasic_roadworksSubCauseCode
        ASN_bitstring roadWorksContainerBasic_lightBarSirenInUse
        # ClosedLanes
        bool roadWorksContainerBasic_closedLanesPresent
            bool roadWorksContainerBasic_closedLanes_hardShoulderStatusPresent
            uint8 roadWorksContainerBasic_closedLanes_hardShoulderStatus
            ASN_bitstring roadWorksContainerBasic_closedLanes_drivingLaneStatus
    # RescueContainer
        ASN_bitstring rescueContainer_lightBarSirenInUse
    # EmergencyContainer
        ASN_bitstring emergencyContainer_lightBarSirenInUse
        # CauseCode
        bool emergencyContainer_incidentIndicationPresent
            uint8 emergencyContainer_incidentIndication_causeCode
            uint8 emergencyContainer_incidentIndication_subCauseCode
        bool emergencyContainer_emergencyPriorityPresent
        ASN_bitstring emergencyContainer_emergencyPriority
    # SafetyCarContainer
        ASN_bitstring safetyCarContainer_lightBarSirenInUse
        # CauseCode
        bool safetyCarContainer_incidentIndicationPresent
            uint8 safetyCarContainer_incidentIndication_causeCode
            uint8 safetyCarContainer_incidentIndication_subCauseCode
        bool safetyCarContainer_trafficRulePresent
        uint8 safetyCarContainer_trafficRule
        bool safetyCarContainer_speedLimitPresent
        uint8 safetyCarContainer_speedLimit