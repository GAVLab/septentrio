//Written by:
//  Lowell Brown & David Hodo
//  Auburn University


#ifndef _SEPTENTRIO_H
#define _SEPTENTRIO_H

#include <iostream>
#include <sstream>
#include <string.h>
#include <iomanip>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include "septentrio_structs.h"


using namespace std;
using namespace boost;


class gSeptentrio
{

  public:
    gSeptentrio();
    ~gSeptentrio(){};

    void display_sep_file(int block_ID);

    //static void listen_thread(void* Parameter);

    unsigned short ComputeCRC(char * buf, int buf_length);

    bool CRC_check(char *head, char *mess, int tot_length, int CRC);

    void readParse();

    //SerialPortBoost port;

    SEP_FLAGS sep_flags;
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
    int block_ID;
    //ofstream sep_file;
  bool display_messages;
    bool display;
    bool check;
    char sep_timestr[50];
  private:
    bool ConfigureLogging(string &sLogs);
    bool HandleLogRequest(string sParam); //< look through string of log requests and request them from the receiver
    bool RequestLog(); //!< request the given Block from the receiver
    bool RequestLogRate(string ouputRate); //!< request the PVT log rate
    bool RequestRangeLogRate(string outputRate); //!< request the Meas (range) log rate
    bool RequestNMEACOM3(); // request NMEA message over COM3 on Septentrio
    bool SetAntennaLocations(int ant_num, string x, string y, string z);
    void SetRTK(string RTK_com,string RTK_baud,string RTK_correction_type);
    void BufferIncomingData(unsigned char* msg, unsigned int length); //!< data read from serial port is passed to this method
    void ParseASCII(unsigned char* block); //Parse ASCII message from septentrio
    void ParseBlock(unsigned char* block, unsigned short ID); //!< Parses one septentrio block
    bool OnInitialized(); //!< Sets the rate of the message outputSEP_FLAGS sep_flags;
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
    void update_ephemeris(double &blockTime);
    void update_range(unsigned char* block, double &blockTime);

    //Buffer funcitons
    unsigned char dataBuf[MAX_MSG_SIZE]; //!<Holds incoming data; Buffer for incoming data
    unsigned int bytesRemaining; //!< Number of bytes to be read in current message.
    unsigned int bufIndex;//!< index to dataBuf
    bool readingASCII;

    string x1;
    string y1;
    string z1;
    string x2;
    string y2;
    string z2;
    bool manual_position;
    bool good_antenna_locations;

};

#endif  /* _GSEPTENTRIO_H */
