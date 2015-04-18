//Written by:
//  Lowell Brown & David Hodo
//  Auburn University


#ifndef _SEPTENTRIO_H
#define _SEPTENTRIO_H

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


// using namespace std;
// using namespace boost;


class Septentrio {

  public:

    Septentrio();
    ~Septentrio();

    bool Connect(std::string port, int baudrate=115200, std::string septentrio_port_="COM1");
    void Disconnect();
    inline bool IsConnected() {return is_connected;}

    bool RequestLog(); //!< request the given Block from the receiver

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

    // bool ConfigureLogging(string &sLogs);
    // bool RequestNMEACOM3(); // request NMEA message over COM3 on Septentrio
    // void SetRTK(string RTK_com,string RTK_baud,string RTK_correction_type);
    void bufferIncomingData(unsigned char* msg, unsigned int length); //!< data read from serial port is passed to this method
    void ParseASCII(unsigned char* block); //Parse ASCII message from septentrio
    void ParseBlock(unsigned char* block, unsigned short ID); //!< Parses one septentrio block
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

    /* basic connection attributes */
    std::string output_rate;
    std::string range_output_rate;

    SEP_HEADER latest_header;
    SEP_PVTXYZ latest_pvtxyz;
    SEP_PVTXYZ_POS_COV latest_pvtxyz_pos_cov;
    SEP_PVTXYZ_VEL_COV latest_pvtxyz_vel_cov;
    SEP_ATTEULER latest_atteuler;
    SEP_ATTEULER_COV latest_atteuler_cov;
    SEP_RECEIVERTIME latest_receivertime;
    SEP_EPHEMERIS latest_ephemeris;
    SEP_RANGE_HEADING latest_range_heading;
    SEP_RANGE_SUB_BLOCK latest_range_sub_block;
    RANGE_DATA latest_range_data;

    //Buffer funcitons
    unsigned char dataBuf[MAX_MSG_SIZE]; //!<Holds incoming data; Buffer for incoming data
    unsigned int bytesRemaining; //!< Number of bytes to be read in current message.
    unsigned int bufIndex;//!< index to dataBuf
    bool readingASCII;

    std::string x1;
    std::string y1;
    std::string z1;
    std::string x2;
    std::string y2;
    std::string z2;
    bool manual_position;
    bool good_antenna_locations;

};

#endif  /* _GSEPTENTRIO_H */
