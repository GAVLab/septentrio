Header header

uint64 GPS_ms  #Time tag of PVT fix in milliseconds of week, receiver time scale
uint16 weekNumber  #Week number
uint8 reserved     #Rserved for future use, DO NOT USE
uint8 error        #Error bit code
float32 Cov_VxVx  #variance of the x-velocity estimate
float32 Cov_VyVy  #variance of the y-velocity estimate
float32 Cov_VzVz  #variance of the z-velocity estimate
float32 Cov_dd  #variance of the clock drift bias
float32 Cov_VxVy  #covariance between the x- and y-velocity estimates
float32 Cov_VxVz  #covariance between the x- and z-velocity estimates
float32 Cov_Vxd  #covariance between the y-velocity and clock drift estimates
float32 Cov_VyVz  #covariance between the y- and z-velocity estimates
float32 Cov_Vyd  #covariance between the y-velocity and clock drift estimates
float32 Cov_Vzd  #covariance between the z-velocity and clock drift estimates