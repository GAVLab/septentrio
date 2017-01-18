Header header

uint64 GPS_ms  #Time tag of attitude solution in milliseconds of week, receiver time scale
uint16 weekNumber  #Week number
uint8 numSat  #Average over all antennas of the number of satellites used
uint8 error  #Error bit field
uint16 mode  #Attitude mode
uint16 reserved  # for future use 
float32 heading  #Heading (degrees)
float32 pitch  #Pitch (degrees)
float32 roll  #Roll (degrees)
float32 x_omega  #Angular rate in vehicle frame (deg/s)
float32 y_omega  #Angular rate in vehicle frame (deg/s)
float32 z_omega  #Angular rate in vehicle frame (deg/s)
