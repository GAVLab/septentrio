/*
 * Written by:
 *  Lowell Brown & David Hodo
 *
 * Major edits by:
 *  Robert Cofield & William McCarty 
 *
 *  Auburn University
 *
 * Manual available at:
 *    http://www.septentrio.com/secure/polarx3_2_2/PolaRx2Manual.pdf
 *
 */
  
#ifndef _SEPTENTRIO_H
#define _SEPTENTRIO_H

#define MAX_MSG_SIZE 4096 //!< see header message length
// #define MAX_NOUT_SIZE (5000)   // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)
#define MAX_NOUT_SIZE 4096   // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)

#define SEP_SYNC_BYTE_1 0x24 //0x24 is ASCII for $ - 1st byte in each message
#define SEP_SYNC_BYTE_2 0x40 //0x24 is ASCII for @ - 2nd byte to indicate Binary message
#define SEP_SYNC_BYTE_3 0x50 //0x50 is ASCII for P - 2nd byte to indicate ASCII message


#include "septentrio_structs.h"

#include <serial/serial.h>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/join.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <fstream>


typedef boost::function<double()> GetTimeHandler;

// gps data callbacks
typedef boost::function<void(ReceiverTime&,     double&)> ReceiverTimeCallback;
typedef boost::function<void(PvtCartesian&,     double&)> PvtCartesianCallback;
typedef boost::function<void(PosCovCartesian&,  double&)> PosCovCartesianCallback;
typedef boost::function<void(VelCovCartesian&,  double&)> VelCovCaresianCallback;
typedef boost::function<void(AttitudeEuler&,    double&)> AttitudeEulerCallback;
typedef boost::function<void(AttitudeCovEuler&, double&)> AttitudeCovEulerCallback;
typedef boost::function<void(OdometryData&, double&)> OdometryCallback;
typedef boost::function<void(RangeData&, double&)> RangeCallback;

// typedef boost::function<void(CfgPrt&, double&)> PortSettingsCallback;


class Septentrio {

  public:

    Septentrio();
    ~Septentrio();

    //////////////////////////
    // Connection functions //
    //////////////////////////

    bool connect(std::string port, int baudrate=115200, std::string septentrio_port_="COM1");
    void disconnect();
    inline bool isConnected() {return is_connected;}

    //////////////////////////////
    // Septentrio Configuration //
    //////////////////////////////

    bool clearLog(); //!< turn off logs
    bool requestLog(std::string logcode_); //!< request an additional Block from the receiver
    
    bool setOutputRate(std::string r); //!< request the PVT log rate
    bool setRangeOutputRate(std::string r); //!< request the Meas (range) log rate   
    
    bool setAntennaLocations(int ant_num, double x, double y, double z);

    bool setRTK(std::string rtk_port, int rtk_baud, std::string rtk_format);
    bool setRTK(); // turn off RTK

    void SetElevationMask(int mask_angle);
    void SetAttitudeMode();
    //////////////////////////////
    // message callback setters //
    //////////////////////////////

    inline void setTimeHandler(GetTimeHandler h) {time_handler=h;};

    inline void setReceiverTimeCallback(ReceiverTimeCallback c) {receiver_time_callback = c;};
    inline void setPvtCartesianCallback(PvtCartesianCallback c) {pvt_cartesian_callback = c;};
    inline void setPosCovCartesianCallback(PosCovCartesianCallback c) {pos_cov_cartesian_callback = c;};
    inline void setVelCovCaresianCallback(VelCovCaresianCallback c) {vel_cov_cartesian_callback = c;};
    inline void setAttitudeEulerCallback(AttitudeEulerCallback c) {attitude_euler_callback = c;};
    inline void setAttitudeCovEulerCallback(AttitudeCovEulerCallback c) {attitude_cov_euler_callback = c;};
    inline void setOdometryCallback(OdometryCallback c) {odometry_callback_ = c;};
    inline void setRangeCallback(RangeCallback c) {range_callback_ = c;};
  private:

    ///////////////////////////
    // serial port functions //
    ///////////////////////////

    void startReading();
    void stopReading();
    void readSerialPort();

    // void SetRTK(string RTK_com,string RTK_baud,string RTK_correction_type);

    void bufferIncomingData(uint8_t * msg, size_t length); //!< data read from serial port is passed to this method
    void ParseASCII(unsigned char* block); //Parse ASCII message from septentrio
    void ParseBinary(unsigned char* block, unsigned short ID); //!< Parses one septentrio block

    void UpdateRange(unsigned char* block);


    /////////////////////////////////
    // Serial port reading members // 
    /////////////////////////////////

    //! Serial port object for communicating with sensor
    serial::Serial *serial_port;
    //! shared pointer to Boost thread for listening for data from novatel
    boost::shared_ptr<boost::thread> read_thread_ptr;
    bool reading_status;  //!< True if the read thread is running, false otherwise.
    bool is_connected;
    std::string septentrio_port; //!< port on the receiver that we're connected to
    double read_timestamp; // when each message was read from the serial port

    /* basic connection attributes for septentrio */
    std::string output_rate;
    std::string range_output_rate;
    std::vector<std::string> log_codes; // logs that we want

    //Buffer funcitons
    unsigned char dataBuf[MAX_MSG_SIZE]; //!<Holds incoming data; Buffer for incoming data
    unsigned int bytesRemaining; //!< Number of bytes to be read in current message.
    unsigned int bufIndex;//!< index to dataBuf
    bool readingASCII;

    /* antenna location stuff */
    double x1, y1, z1, x2, y2, z2; // manual antenna locations
    bool manual_position;
    bool good_antenna_locations;

    // stuff that gets stored for the parse function
    Header latest_header_;
    ReceiverTime latest_receivertime_;
    PvtCartesian latest_pvtxyz_;
    PosCovCartesian latest_pvtxyz_pos_cov_;
    VelCovCartesian latest_pvtxyz_vel_cov_;
    AttitudeEuler latest_atteuler_;
    AttitudeCovEuler latest_atteuler_cov_;
    MeasEpoch latest_range_heading;
    MeasEpochSubBlock latest_range_sub_block;
    RangeData latest_range_data;
    OdometryData latest_odometry_data_;

    bool is_rtk; // whether we're in RTK mode

    ///////////////////////
    // callback handlers //
    ///////////////////////

    GetTimeHandler               time_handler;
    ReceiverTimeCallback          receiver_time_callback;
    PvtCartesianCallback          pvt_cartesian_callback;
    PosCovCartesianCallback       pos_cov_cartesian_callback;
    VelCovCaresianCallback        vel_cov_cartesian_callback;
    AttitudeEulerCallback         attitude_euler_callback;
    AttitudeCovEulerCallback      attitude_cov_euler_callback;
    OdometryCallback              odometry_callback_;
    RangeCallback             range_callback_;

};

#endif  // _GSEPTENTRIO_H
