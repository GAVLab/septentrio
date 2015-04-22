Header header

uint64 GPS_ms  #Time tag of PVT fix in milliseconds of week, receiver time scale
uint16 weekNumber  #Week number
uint8 reserved     #Rserved for future use, DO NOT USE
uint8 error        #Error bit code
float32 Cov_xx  #variance of the x estimate
float32 Cov_yy  #variance of the y estimate
float32 Cov_zz  #variance of the z estimate
float32 Cov_bb  #variance of the clock bias
float32 Cov_xy  #covariance between the x and y estimates
float32 Cov_xz  #covariance between the x and z estimates
float32 Cov_xb  #covariance between the y and clock bias estimates
float32 Cov_yz  #covariance between the y and z estimates
float32 Cov_yb  #covariance between the y and clock bias estimates
float32 Cov_zb  #covariance between the z and clock bias estimates