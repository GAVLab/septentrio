/*
 * Structures into which Septenrio data is unpack then shipped to handler
 * functions
 *
 */

#pragma pack ( push , 1)
struct SEP_FLAGS
{
    //utility flags
    bool save_data;
    bool data_ready;

    //Requested data flags
    bool rcvrtimeflag;
    bool pvtflag;
    bool attitudeflag;
};


//structure for the header message
struct Header
{
    unsigned char SYNC1; //!< first sync byte is $ or 0x24
    unsigned char SYNC2; //!< 2nd sync byte is @ or 0x40
    unsigned short CRC; //!< the Check Sum !
    unsigned short ID; //!< The ID is the "Block ID"
    unsigned short Length; //!< length of the entire message including the header. A multiple of 4 between 8 and 4096
};



//structure for the PVTCartesian parsed block 
struct PvtCartesian //message 5903
{
    unsigned int GPS_ms; //Time tag of PVT fix in milliseconds of week, receiver time scale
    unsigned short weekNumber; //Week number
    unsigned char numSat; //Number of satellites used in PVT :8 means only a single byte
    unsigned char Error; //PVT error code
    unsigned char Mode; //Mode
    unsigned char System; //Defines which GNSS system is used
    unsigned char Info; //Additional PVY information
    unsigned char SBASprn; //PRN of the SBAS satellite used for aubmentation information :8 means only a single byte
    double x_position; //X coordinate in WGS-84(m)
    double y_position; //Y coordinate in WGS-84(m)
    double z_position; //Z coordinate in WGS-84 (m)
    float x_velocity; //Velocity in X direction (m/s)
    float y_velocity; //Velocity in Y direction (m/s)
    float z_velocity; //Velocity in Z direction (m/s)
    double RxClkBias; //Receiver clock bias relative to GPS time (s)
    float RxClkDrift; //Receiver clock drift relative to GPS time (s/s)
    unsigned short MeanCorrAge; //Mean age of differential corrections if applicable (1/100 sec)
    unsigned short ReferenceID; //Reference ID of the base station for differential corrections
    float course; //Course over ground (degree) from north
};


//structure for the PosCovCartesian parsed block
struct PosCovCartesian//message 5905
{
    unsigned int GPS_ms; //Time tag of PVT fix in milliseconds of week, receiver time scale
    unsigned short weekNumber; //Week number
    unsigned char reserved;    //Rserved for future use, DO NOT USE
    unsigned char error;       //Error bit code
    float Cov_xx; //variance of the x estimate
    float Cov_yy; //variance of the y estimate
    float Cov_zz; //variance of the z estimate
    float Cov_bb; //variance of the clock bias
    float Cov_xy; //covariance between the x and y estimates
    float Cov_xz; //covariance between the x and z estimates
    float Cov_xb; //covariance between the y and clock bias estimates
    float Cov_yz; //covariance between the y and z estimates
    float Cov_yb; //covariance between the y and clock bias estimates
    float Cov_zb; //covariance between the z and clock bias estimates
};


//structure for the VelCovCaresian parsed block
struct VelCovCartesian//message 5907
{
    unsigned int GPS_ms; //Time tag of PVT fix in milliseconds of week, receiver time scale
    unsigned short weekNumber; //Week number
    unsigned char reserved;    //Rserved for future use, DO NOT USE
    unsigned char error;       //Error bit code
    float Cov_VxVx; //variance of the x-velocity estimate
    float Cov_VyVy; //variance of the y-velocity estimate
    float Cov_VzVz; //variance of the z-velocity estimate
    float Cov_dd; //variance of the clock drift bias
    float Cov_VxVy; //covariance between the x- and y-velocity estimates
    float Cov_VxVz; //covariance between the x- and z-velocity estimates
    float Cov_Vxd; //covariance between the y-velocity and clock drift estimates
    float Cov_VyVz; //covariance between the y- and z-velocity estimates
    float Cov_Vyd; //covariance between the y-velocity and clock drift estimates
    float Cov_Vzd; //covariance between the z-velocity and clock drift estimates
};


//structure for the AttitudeEuler parsed block 
struct AttitudeEuler //message 5938
{
    unsigned int GPS_ms; //Time tag of attitude solution in milliseconds of week, receiver time scale
    unsigned short weekNumber; //Week number
    unsigned char numSat; //Average over all antennas of the number of satellites used
    unsigned char error; //Error bit field
    unsigned short mode; //Attitude mode
    unsigned short reserved; // for future use 
    float heading; //Heading (degrees)
    float pitch; //Pitch (degrees)
    float roll; //Roll (degrees)
    float x_omega; //Angular rate in vehicle frame (deg/s)
    float y_omega; //Angular rate in vehicle frame (deg/s)
    float z_omega; //Angular rate in vehicle frame (deg/s)
};


//structure for the AttitudeCovEuler parsed block
struct AttitudeCovEuler//message 5939
{
    unsigned int GPS_ms;       //Time tag of attitude solution in milliseconds of week, receiver time scale
    unsigned short weekNumber; //Week number
    unsigned char reserved;    //Rserved for future use, DO NOT USE
    unsigned char error;       //Error bit code
    float var_heading;         //variance of heading estimate deg^2
    float var_pitch;           //variance of pitch estiamte   deg^2 
    float var_roll;            //variance of roll estiamte    deg^2
};


//structure for the ReceiverTime parsed block
struct ReceiverTime//message 5914
{
    unsigned int GPS_ms; //Time tag in milliseconds of week
    unsigned short weekNumber; //Week number
    signed int UTCYear;
    signed int UTCMonth;
    signed int UTCDay;
    signed int UTCHour;
    signed int UTCMin;
    signed int UTCSec;
    signed int deltaLS; //Integer second difference between GPS and UTC time Positive if GPS time is ahead of UTC. Set to -148 if not available
    unsigned char syncLevel; //Bit field indicating the synchronization level of the receiver time
};


//structure for the GPSNAV parsed block (GPS SV Ephemeris)
struct Ephemeris//message 5891
{
    unsigned int GPS_ms; //Time tag in milliseconds of week
    unsigned short weekNumber; //Week number
    unsigned char PRN; //PRN number (1 to 37)
    unsigned char reserved;
    signed short WN;
    unsigned char CAorPonL2;
    unsigned char URA;
    unsigned char health;
    unsigned char L2DataFlag;
    unsigned short IODC;
    unsigned char IODE2;
    unsigned char IODE3;
    unsigned char FitInFlg;
    unsigned char NotUsed;
    float T_gd;
    unsigned int t_oc;
    float a_f2;
    float a_f1;
    float a_f0;
    float C_rs;
    float DELTA_N;
    double M_0;
    float C_uc;
    double e;
    float C_us;
    double SQRT_A;
    unsigned int t_oe;
    float C_ic;
    double OMEGA_0;
    float C_is;
    double i_0;
    float C_rc;
    double omega;
    float OMEGADOT;
    float IDOT;
    unsigned short WNt_oc;
    unsigned short WNt_oe;
};


//structure for the heading of the MeasEpoch block
struct MeasEpoch//part of message 5889
{//this section of the message is 8 bytes long
    unsigned int GPS_ms; //Time tag in milliseconds of week
    unsigned short weekNumber; //Week number
    unsigned char N; //number of sub-blocks
    unsigned char SBLength; //number of bytes of each sub-frame
};


//structure for the MeasEpochFlag
struct MeasEpochFlag
{
    unsigned L1_loss_of_lock: 1;
    unsigned L2_loos_of_lock: 1;
    unsigned L1_smoothing: 1;
    unsigned L2_smoothing: 1;
    unsigned multipath_mitigation: 1;
    unsigned antennaID: 3;
};


//structure for the sub-blocks of the MeasEpoch block
struct MeasEpochSubBlock//part of message 5889
{
    double CACode;
    float P1_CACode;
    float P2_CACode;
    double L1Phase;
    double L2Phase;
    signed int L1Doppler;
    signed int L2Doppler;
    signed short CACN0;
    signed short P1CN0;
    signed short P2CN0;
    unsigned char SVID;
    unsigned char RXChannel;
    float LockTime;
    MeasEpochFlag Flags;

};

struct OdometryData{

    PvtCartesian pvt;
    PosCovCartesian pos_cov;
    VelCovCartesian vel_cov;
    AttitudeEuler   att;
    AttitudeCovEuler att_cov;

};


#pragma pack ( pop ) //See GavlabVehicleCode.pdf for explaination
struct RANGE_DATA
{
    int SVID_1[30];
    double CACode_1[30];
    double P1_CACode_1[30];
    double P2_CACode_1[30];
    double L1Phase_1[30];
    double L2Phase_1[30];
    double L1Doppler_1[30];
    double L2Doppler_1[30];
    double CAC2N_1[30];
    double P1C2N_1[30];
    double P2C2N_1[30];

    int SVID_2[30];
    double CACode_2[30];
    double P1_CACode_2[30];
    double P2_CACode_2[30];
    double L1Phase_2[30];
    double L2Phase_2[30];
    double L1Doppler_2[30];
    double L2Doppler_2[30];
    double CAC2N_2[30];
    double P1C2N_2[30];
    double P2C2N_2[30];

    int SVID_3[30];
    double CACode_3[30];
    double P1_CACode_3[30];
    double P2_CACode_3[30];
    double L1Phase_3[30];
    double L2Phase_3[30];
    double L1Doppler_3[30];
    double L2Doppler_3[30];
    double CAC2N_3[30];
    double P1C2N_3[30];
    double P2C2N_3[30];

};


/* CRCLookUp provided by Septenrio
 *(c) 2000-2004 Septentrio nv/sa, Belgium */
static const unsigned int CRCLookUp[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};
