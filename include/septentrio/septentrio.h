//Written by:
//  Lowell Brown & David Hodo
//  Auburn University
#ifndef _SEPTENTRIO_H
#define _SEPTENTRIO_H

#define MAX_MSG_SIZE 4096 //!< see header message length
#define MAX_NOUT_SIZE      (5000)   // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)

#include "septentrio_structs.h"

#include <serial/serial.h>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>


typedef boost::function<double()> GetTimeCallback;

// gps data callbacks
typedef boost::function<void(SEP_RECEIVERTIME&,   double&)> ReceiverTimeCallback;
typedef boost::function<void(SEP_PVTXYZ&,         double&)> PvtCartestianCallback;
typedef boost::function<void(SEP_PVTXYZ_POS_COV&, double&)> PosCovCartesianCallback;
typedef boost::function<void(SEP_PVTXYZ_VEL_COV&, double&)> VelCovCaresianCallback;
typedef boost::function<void(SEP_ATTEULER&,       double&)> AttitudeEulerCallback;
typedef boost::function<void(SEP_ATTEULER_COV&,   double&)> AttitudeCovEulerCallback;

// typedef boost::function<void(CfgPrt&, double&)> PortSettingsCallback;


class Septentrio {

  public:

    Septentrio();
    ~Septentrio();

    bool connect(std::string port, int baudrate=115200, std::string septentrio_port_="COM1");
    void disconnect();
    inline bool isConnected() {return is_connected;}

    bool requestLog(std::string logcode_); //!< request the given Block from the receiver

    /* Setters that replace mission file reader */
    bool setOutputRate(std::string r); //!< request the PVT log rate
    bool setRangeOutputRate(std::string r); //!< request the Meas (range) log rate
    
    bool SetAntennaLocations(int ant_num, std::string x, std::string y, std::string z);

    int block_ID;
    //ofstream sep_file;
    bool display_messages;
    bool display;
    bool check;
    char sep_timestr[50];

    /* Bullshit */
    // void display_sep_file(int block_ID);
    //static void listen_thread(void* Parameter);
    // unsigned short ComputeCRC(char * buf, int buf_length);
    // bool CRC_check(char *head, char *mess, int tot_length, int CRC);

  private:

    // serial port functions
    void startReading();
    void stopReading();
    void readSerialPort();



    // bool ConfigureLogging(string &sLogs);
    // bool RequestNMEACOM3(); // request NMEA message over COM3 on Septentrio
    // void SetRTK(string RTK_com,string RTK_baud,string RTK_correction_type);
    void bufferIncomingData(uint8_t * msg, size_t length); //!< data read from serial port is passed to this method
    void ParseASCII(unsigned char* block); //Parse ASCII message from septentrio
    void ParseBinary(unsigned char* block, unsigned short ID); //!< Parses one septentrio block
    bool OnInitialized(); //!< Sets the rate of the message outputSEP_FLAGS sep_flags;

    void update_ephemeris(double &blockTime);
    void update_range(unsigned char* block, double &blockTime);

    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
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

    //Buffer funcitons
    unsigned char dataBuf[MAX_MSG_SIZE]; //!<Holds incoming data; Buffer for incoming data
    unsigned int bytesRemaining; //!< Number of bytes to be read in current message.
    unsigned int bufIndex;//!< index to dataBuf
    bool readingASCII;

    /* antenna location stuff */
    std::string x1, y1, z1, x2, y2, z2; // manual antenna locations
    bool manual_position;
    bool good_antenna_locations;

    // stuff that gets stored for the parse function
    SEP_HEADER latest_header;
    SEP_RECEIVERTIME latest_receivertime;

    /* callback handlers */
    GetTimeCallback               time_handler;
    ReceiverTimeCallback          receiver_time_callback;
    PvtCartestianCallback         pvt_cartesian_callback;
    PosCovCartesianCallback       pos_cov_cartesian_callback;
    VelCovCaresianCallback        vel_cov_cartesian_callback;
    AttitudeEulerCallback         attitude_euler_callback;
    AttitudeCovEulerCallback      attitude_cov_euler_callback;

};

#endif  /* _GSEPTENTRIO_H */
