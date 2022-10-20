std_msgs/Header header

#Vehicle motion variables
float32 fSteeringAngle                #[rad] Single-Track_Steering-Angle         
float32 fSteeringAngleRate            ##[deg/s]                                   
float32 fVelocity                     #[m/s] Velocity mean, signed               
float32 fVelocityFR                   #[m/s] Velocity FR, signed                 
float32 fVelocityFL                   #[m/s] Velocity FL, signed                 
float32 fVelocityRR                   #[m/s] Velocity RR, signed                 
float32 fVelocityRL                   #[m/s] Velocity RL, signed                 
float32 fAccelerationLateral          #[m/s^2]                                    
float32 fAccelerationLongitudinal     #[m/s^2]                                    
float32 fYawRate                      #[rad/s]                                   
float32 fPitch                        #[%]                                      
float32 fTrackCurvature               #[1/m]                                     
float32 nMovingDir                    #0: Standing, 1: Forward, -1 Backward
bool bStandStill
