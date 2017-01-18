Header header

uint64 GPS_ms        # Time tag of attitude solution in milliseconds of week, receiver time scale
uint16 weekNumber  # Week number
uint8 reserved     # Rserved for future use, DO NOT USE
uint8 error        # Error bit code
float32 var_heading          # variance of heading estimate deg^2
float32 var_pitch            # variance of pitch estiamte   deg^2 
float32 var_roll             # variance of roll estiamte    deg^2
