std_msgs/Header header

# GPS time variables
uint64 iTimeUTC_Microseconds
uint64 iTimeGPS_Microseconds
uint64 iTimeHour
uint64 iTimeMinute
uint64 iTimeSecond
float64 fTimeHSecond
uint64 iTimeDay
uint64 iTimeMonth
uint64 iTimeYear

# GPS quality variables
uint16 iSatellitesUsed # Number of satellites

# Measurement status
int16 STATUS_NO_FIX=-1   # Unable to fix position
int16 STATUS_FIX=0       # Normal fix
int16 STATUS_SBAS_FIX=1  # Fixed using a satellite-based augmentation system
int16 STATUS_GBAS_FIX=2  #          or a ground-based augmentation system
int16 STATUS_DGPS_FIX=18 # Fixed with DGPS
int16 STATUS_WAAS_FIX=33 # Fixed with WAAS
int16 nStatus

# Extended Measurement status
uint8 STATUS_EXT_NONE=0 #The GPS is not able to make this measurement.
uint8 STATUS_EXT_SEARCH=1 #The GPS system is solving ambiguities and searching for a valid solution. 
uint8 STATUS_EXT_DOPPLER=2 #GPS measurement is based on a Doppler measurement. 
uint8 STATUS_EXT_SPS=3 #Standard Positioning Service, the GPS measurement has no additional external corrections.
uint8 STATUS_EXT_DIFERENTIAL=4 #The GPS measurement used pseudo-range differential corrections. 
uint8 STATUS_EXT_RTK_FLOAT=5 #The GPS measurement used L1 carrier-phase differential corrections to give a floating ambiguity solution.
uint8 STATUS_EXT_RTK_INTEGER=6 #The GPS measurement used L1/L2 carrier-phase differential corrections to give an integer ambiguity solution. 
uint8 STATUS_EXT_WAAS=7 #The GPS measurement used SBAS corrections.
uint8 STATUS_EXT_OMNISTAR=8 #The GPS measurement used OmniSTAR VBS corrections.
uint8 STATUS_EXT_OMNISTAR_HP=9 #The GPS measurement used OmniSTAR HP corrections. 
uint8 STATUS_EXT_NO_DATA=10 #No Data.
uint8 STATUS_EXT_BLANKED=11 #Blanked. 
uint8 STATUS_EXT_DOPPLER_PP=12 #Doppler GPS measurement post-processed. 
uint8 STATUS_EXT_SPS_PP=13 #SPS GPS measurement post-processed. 
uint8 STATUS_EXT_DIFERENTIAL_PP=14 #Differential GPS measurement post-processed. 
uint8 STATUS_EXT_RTK_FLOAT_PP=15 #RTK Float GPS measurement post-processed.
uint8 STATUS_EXT_RTK_INTEGER_PP=16 #RTK Integer GPS measurement post-processed.
uint8 STATUS_EXT_OMNISTAR_XP=17 #The GPS measurement used OmniSTAR XP corrections.
uint8 STATUS_EXT_CDGPS=18 #The GPS measurement used real time Canada wide DGPS service.
uint8 STATUS_EXT_NOT_RECOGNIZED=19 #Not recognised.
uint8 STATUS_EXT_GXDOPPLER=20 #Computed by combining raw Doppler measurements. 
uint8 STATUS_EXT_GXSPS=21 #Computed by combining raw pseudo-range measurements. 
uint8 STATUS_EXT_GSDIFFERENTIAL=22 #Computed by combining raw pseudo-range measurements and differential corrections. 
uint8 STATUS_EXT_GXFLOAT=23 #Computed by combining raw pseudo-range and L1 carrier-phase measurements and differential corrections. 
uint8 STATUS_EXT_GXINTEGER=24 #Computed by combining raw pseudo-range and L1/L2 carrier-phase measurements and differential corrections. 
uint8 STATUS_EXT_IXDOPPLER=25 #Single-satellite updates from raw Doppler measurements.
uint8 STATUS_EXT_IXSPS=26 #Single-satellite updates from raw pseudo-range measurements. 
uint8 STATUS_EXT_IXDIFERENTIAL=27 #Single-satellite updates from raw pseudo-range measurements and differential corrections. 
uint8 STATUS_EXT_IXFLOAT=28 #Single-satellite updates from raw pseudo-range and L1 carrier-phase measurements and differential corrections.
uint8 STATUS_EXT_IXINTEGER=29 #Single-satellite updates from raw pseudo-range and L1/L2 carrier-phase measurements and differential corrections. 
uint8 STATUS_EXT_PPP_CONVERGING=30 #Converging PPP (Precise Point Positioning) from global PPP corrections.
uint8 STATUS_EXT_PPP=31 #Converged PPP (Precise Point Positioning) from global PPP corrections. 
uint8 STATUS_EXT_UNKNOWN=32 #Unknown.
uint8 nStatusExt

# GPS location variables
float64 fLongitude
float64 fLatitude
float64 fAltitude
float64 fPosNorthStDev
float64 fPosEastStDev
float64 fPosDownStDev

# GPS position variables
float64 fAngleHeading
float64 fAnglePitch
float64 fAngleRoll
float64 fAngleHeadingStDev
float64 fAnglePitchStDev
float64 fAngleRollStDev

# UTM Postition
float64 fUTMEast
float64 fUTMNorth
float64 fUTMHeading

# GPS velocity variables
float64 fSpeed2D
float64 fVelLateral
float64 fVelForward
float64 fVelNorth
float64 fVelEast
float64 fVelDown
float64 fVelNorthStDev
float64 fVelEastStDev
float64 fVelDownStDev

# GPS acceleration variables
float64 fAccelX
float64 fAccelY
float64 fAccelZ
float64 fAngRateZ
float64 fAngRateY
float64 fAngRateX

