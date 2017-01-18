//Written by:
//  Lowell Brown & David Hodo
//  Auburn University
//  Septentrio Documentation: http://www.septentrio.com/secure/polarx3_2_2/PolaRx2Manual.pdf

#include <septentrio/septentrio.h>
 

inline void printHex(char *data, int length) {
    for (int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned) (unsigned char) data[i]);
    }
    printf("\n");
}


double defaultGetTimeHandler() {  // I think I want to do this as a chrono
  boost::posix_time::ptime present_time(
          boost::posix_time::microsec_clock::universal_time());
  boost::posix_time::time_duration duration(present_time.time_of_day());
  return duration.total_seconds();
}


void defaultReceiverTimeCallback(ReceiverTime& data, double& cpu_stamp) {
  std::cout << "defaultReceiverTimeCallback: Received ReceiverTime" << std::endl;
}


void defaultPvtCartesianCallback(PvtCartesian& data, double& cpu_stamp) {
  std::cout << "defaultPvtCartesianCallback: Received PvtCartesian" << std::endl;
}


void defaultPosCovCartesianCallback(PosCovCartesian& data, double& cpu_stamp) {
  std::cout << "defaultPosCovCartesianCallback: Received PosCovCartesian" << std::endl;
}


void defaultVelCovCartesianCallback(VelCovCartesian& data, double& cpu_stamp) {
  std::cout << "defaultVelCovCaresianCallback: Received VelCovCartesian" << std::endl;
}


void defaultAttitudeEulerCallback(AttitudeEuler& data, double& cpu_stamp) {
  std::cout << "defaultAttitudeEulerCallback: Received AttitudeEuler" << std::endl;
}


void defaultAttitudeCovEulerCallback(AttitudeCovEuler& data, double& cpu_stamp) {
  std::cout << "defaultAttitudeCovEulerCallback: Received AttitudeCovEuler" << std::endl;
}

void defaultOdometryCallback(OdometryData& data, double& cpu_stamp){
  std::cout << "defaultOdometryCallback: Received OdometryData" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

Septentrio::Septentrio()
{
  /* initialize serial reader values */
  good_antenna_locations = true;
  bytesRemaining = 0;
  bufIndex = 0;
  is_connected = false;

  time_handler =                    defaultGetTimeHandler;

  receiver_time_callback =          defaultReceiverTimeCallback;
  pvt_cartesian_callback =          defaultPvtCartesianCallback;
  pos_cov_cartesian_callback =      defaultPosCovCartesianCallback;
  vel_cov_cartesian_callback =      defaultVelCovCartesianCallback;
  attitude_euler_callback =         defaultAttitudeEulerCallback;
  attitude_cov_euler_callback =     defaultAttitudeCovEulerCallback;
  odometry_callback_ =				      defaultOdometryCallback;
  
  output_rate = "1";
  range_output_rate = "1";

  manual_position = false;
  x1 = 0.0;
  y1 = 0.0;
  z1 = 0.0;
  x2 = 0.0;
  y2 = 0.0;
  z2 = 0.0;

   //Specifically for Sarnoff
  //spits out NMEA messages over COM3 (5 pin output)
  // string NMEACOM3 = "0";//for false - default is not to do it
  // string RTK_com="COM2";
  // string RTK_baud="115200";
  // string RTK_correction_type="CMR";
  // bool rtk_input;
  // if(m_MissionReader.GetConfigurationParam("NMEA_COM3",NMEACOM3)) //Be sure to set in mission file
  // {
  //   if(NMEACOM3 == "1")
  //     RequestNMEACOM3();
  // } else {
  //   MOOSTrace("NMEA_COM3 not set in mission file - Warning Septentrio Default set to False\n");
  // } 

  // if(m_MissionReader.GetConfigurationParam("USE_RTK",rtk_input)) //Be sure to set in mission file
  // {
  // } else {
  //   MOOSTrace("USE_RTK not set in mission file - Warning Septentrio Default to false\n");
  //   rtk_input = false;
  // }
  // if (rtk_input)
  // {
  //   if(m_MissionReader.GetConfigurationParam("RTK_COM",RTK_com))
  //   {
  //   }else{
  //     MOOSTrace("RTK_COM not set in mission file - Warning Septentrio Default COM2 set\n");
  //   }
  //   if(m_MissionReader.GetConfigurationParam("RTK_BAUD",RTK_baud))
  //   {
  //   }else{
  //     MOOSTrace("RTK_BAUD not set in mission file - Warning Septentrio Default 115200 set\n");
  //   }
  //   if(m_MissionReader.GetConfigurationParam("RTK_CORRECTION_TYPE",RTK_correction_type))
  //   {
  //   }else{
  //     MOOSTrace("RTK_CORRECTION_TYPE not set in mission file - Warning Septentrio Default CMR set\n");
  //   }
  //   SetRTK(RTK_com,RTK_baud,RTK_correction_type);
  // }
}


Septentrio::~Septentrio()
{
  disconnect();
}


/////////////////////////////////////////
////////// Serial Port Methods //////////
/////////////////////////////////////////


bool Septentrio::connect(std::string port, int baudrate, std::string septentrio_port_)
{
  septentrio_port = septentrio_port_;
  // serial::Timeout my_timeout(100, 1000, 0, 1000, 0);
  // serial::Timeout my_timeout = serial::Timeout::simpleTimeout(1);
  serial::Timeout my_timeout(0, 2, 0, 2, 0);
  try {
    serial_port = new serial::Serial(port, baudrate, my_timeout);
    if (!serial_port->isOpen()) {
      // std::stringstream output;
      // output << "Serial port: " << port << " failed to open.";
      // log_error_(output.str());
      delete serial_port;
      serial_port = NULL;
      is_connected = false;
      return false;
    } else {
      // std::stringstream output;
      // output << "Serial port: " << port << " opened successfully.";
      // log_info_(output.str());
    }

    serial_port->flush();

    // !!!!!!!!!!!!!
    // Assume that connection is successful
    // !!!!!!!!!!!!!
    //
    // look for GPS by sending ping and waiting for response
    // if (!Ping()) {
    //   std::stringstream output;
    //   output << "Septentrio GPS not found on port: " << port << std::endl;
    //   log_error_(output.str());
    //   delete serial_port;
    //   serial_port = NULL;
    //   is_connected_ = false;
    //   return false;
    // } 

  } catch (std::exception e) {
    // std::stringstream output;
    // output << "Failed to open port " << port << "  Err: " << e.what();
    // log_error_(output.str());
    serial_port = NULL;
    is_connected = false;
    return false;
  }

  // start reading
  startReading();
  is_connected = true;

  clearLog(); // turn off all output
  setOutputRate(output_rate);
  setRangeOutputRate(range_output_rate);

  // set manual position
  std::string galcmd;  
  if (manual_position) {
    std::cout << "manual position true\t" << x2 << std::endl;
    galcmd = "gal\r\n";
    serial_port->write(galcmd)==galcmd.length();
    setAntennaLocations(1, x1, y1, z1);
    setAntennaLocations(2, x2, y2, z2);
  }


  return true;
}


void Septentrio::disconnect()
{
  try {
    if (reading_status) {
      stopReading();
      // TODO: wait here for reading to stop
    }
    if (serial_port != NULL) {
      if (serial_port->isOpen())
        serial_port->close();
      delete serial_port;
      serial_port = NULL;
    }
  } catch (std::exception &e) {
    // stringstream output;
    // output << "Error disconnecting from ublox: " << e.what();
    // log_error_(output.str());
    // std::cout << "Error in Disconnect()";
  }
}


void Septentrio::startReading() {
  try {
    // create thread to read from sensor
    reading_status = true;
    read_thread_ptr = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&Septentrio::readSerialPort, this)));
  } catch (std::exception &e) {
    // std::stringstream output;
    // output << "Error starting ublox read thread: " << e.what();
    // log_error_(output.str());
  }
}


void Septentrio::stopReading() {
    reading_status = false;
}


void Septentrio::readSerialPort() {
    uint8_t buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status) {
      // read data
      try {
          len = serial_port->read(buffer, MAX_NOUT_SIZE);
      } catch (std::exception &e) {
          // stringstream output;
          // output << "Error reading serial port: " << e.what();
          // log_info_(output.str());
          disconnect();
          return;
      }
      // timestamp the read
      read_timestamp = time_handler();
      // add data to the buffer to be parsed
      bufferIncomingData(buffer, len);
    }

}


//////////////////////////////
// Septentrio specific stuff for logs
/////////////////////////////////

bool Septentrio::clearLog()
{
  log_codes.clear();
  std::string cmd="SetSBFOutput " + septentrio_port + ", off " + "\r\n";
  return (serial_port->write(cmd)==cmd.length());
}

bool Septentrio::requestLog(std::string logcode_)
{
  log_codes.push_back(logcode_);
  std::string cmd = boost::algorithm::join(log_codes, "+");
  cmd = "SetSBFOutput " + septentrio_port + ", " + cmd + " \r\n";
  std::cout << "Septentrio::requestLog sending request for: " << cmd << std::endl;
  return (serial_port->write(cmd)==cmd.length());
}


bool Septentrio::setOutputRate(std::string r)
{
  output_rate = r;
  std::string cmd = "SetPVTInterval " + output_rate + "\r\n";
  //TODO: read block rate from mission file
  return (serial_port->write(cmd)==cmd.length());
};


bool Septentrio::setRangeOutputRate(std::string r)
{
  range_output_rate = r;
  std::string cmd = "SetMeasInterval " + range_output_rate + "\r\n";
  //TODO: read block rate from mission file
  return (serial_port->write(cmd)==cmd.length());
};


void Septentrio::bufferIncomingData(uint8_t *msg, size_t length)
{
  // std::cout<<"Buffering\n";
  // std::cout<<"Size of msg: " << length << std::endl;
  // add incoming data to buffer
  for (unsigned int i=0; i<length; i++) {
    //cout << i << ": " << hex << (int)msg[i] << dec  << " : " << bytesRemaining  << " : " << bufIndex << endl;
    // make sure bufIndex is not larger than buffer
    if (bufIndex >= MAX_MSG_SIZE) {
      bufIndex = 0;
      std::cout << "Septentrio: Overflowed receive buffer. Reset" << std::endl;
    }
    // looking for beginning of message
    if (bufIndex == 0) { 
      if (msg[i] == SEP_SYNC_BYTE_1) {
        // beginning of msg found - add to buffer
        // std::cout << "Beginning of message found\n";
        dataBuf[bufIndex++] = msg[i];
        bytesRemaining = 0;
        readingASCII = false;
      }
    } else if (bufIndex==1)     { // verify 2nd character of header
      if (msg[i] == SEP_SYNC_BYTE_2) { // 2nd byte ok - add to buffer
        dataBuf[bufIndex++]=msg[i];
        // std::cout<<"Binary Message\n";
      } else if (msg[i] == SEP_SYNC_BYTE_3) {
        readingASCII = true;
        dataBuf[bufIndex++] = msg[i];
        // std::cout<<"ASCII Message\n";
      } else {
        // start looking for new message again
        // std::cout << "start looking for new message again" << std::endl;
        bufIndex=0;
        bytesRemaining=0;
        readingASCII=false;
      }
    } else if (bufIndex == 7) { // reading last byte of header
        // std::cout<<"(bufIndex==7)\n";
        dataBuf[bufIndex++]=msg[i]; 
        //we should have the entire header now
        if (!readingASCII) {
          memcpy(&latest_header_, dataBuf, 8);
          bytesRemaining = latest_header_.Length - 8;//!< How many bytes are left to read in the message
        }
    } else if (bytesRemaining==1) { // add last byte and parse
      // std::cout<<"(bytesRemaining==1)\n";
      dataBuf[bufIndex++]=msg[i];
      ParseBinary(dataBuf,latest_header_.ID);
      // reset counters
      bufIndex=0;
      bytesRemaining=0;
      // std::cout << "Message Done." << std::endl;
    } else if ((bytesRemaining>1)||((bufIndex>1)&&(bufIndex<7))) { // add data to buffer
      dataBuf[bufIndex++] = msg[i];
      if (!readingASCII) {
        bytesRemaining--;
      }
    } else if ( (readingASCII) && (bufIndex > 7) ) {
      // std::cout<<"reading ascii message\n";
      // std::cout<<hex<<(int)msg[i]<< std::endl;
      if (!((msg[i]==0x23)||(msg[i]==0x0D))) {
        //looking for end of line character, a pound symbol from the septentrio, hex 0x23
        dataBuf[bufIndex++]=msg[i];
      } else {
        dataBuf[bufIndex++]=0;
        ParseASCII(dataBuf);
        readingASCII=false;
        bufIndex=0;
      }
    }
  }
}

void Septentrio::ParseASCII(unsigned char* block)
{ 
  std::stringstream ss (std::stringstream::in | std::stringstream::out);
  std::string block2;
  //string block="$PolaRx: SetAntennaLocation auto, 2, 5.23, 10.00, -2.000\n $PolaRx: SetAntennaLocation manual, 2, 0.47355, 10.51, -2.143\n";
  std::cout << "BLOCKS:\n" << block << std::endl;
  ss << block;
  getline(ss,block2);
  std::cout << block2 << std::endl;
   
  int i;
  float f;
  
  boost::char_separator<char> sep("\n");
  boost::tokenizer<boost::char_separator<char> > lines(block2,sep);
  BOOST_FOREACH(std::string line, lines) {
    std::cout << "LINE:\n";
    boost::char_separator<char> sep2(", ");
    boost::tokenizer<boost::char_separator<char> > tokens(line,sep2);

    boost::tokenizer<boost::char_separator<char> >::iterator itField = tokens.begin();
    itField++; //advance past header
    //determine ASCII message type

    //Response to "gal" (get antenna locations)
    if (*itField=="SetAntennaLocation") {
      if (*++itField=="auto") {
        //determine antenna location mode
        good_antenna_locations=false;
        std::cout << "Antenna Locations set to AUTO\n";
        //send correct antenna locations
      } else if (*itField=="manual") {         
        std::cout << "Antenna Locations set to MANUAL\n";
        //check antenna location, resend only if necessary
        i = strtod((*++itField).c_str(),NULL);
        std::cout << "Checking location of antenna # " << i << ":" << std::endl;
        if (i==1) {
          std::cout << "\tX coordinate:\n";
          //Check x coordinate
          f = strtod((*++itField).c_str(),NULL);    
          std::cout << "\t actual\t" << f << std::endl;
          std::cout << "\t desired\t" << x1<< std::endl;
          std::cout << "\t Difference\t" << abs(f-x1)<< std::endl;
          if (abs(f-x1)<.0001) {
            std::cout << "MATCH!\n";
          } else {
            good_antenna_locations=false;
            std::cout << f <<"\t"<<x1 << "\n";
            std::cout << "MISMATCH!\n";
          }
          //Check y coordinate
          f = strtod((*++itField).c_str(),NULL);
          std::cout << "\t actual\t" << f << std::endl;
          std::cout << "\t desired\t" << y1 << std::endl;
          std::cout << "\t Difference\t" << abs(f-y1) << std::endl;
          if (abs(f-y1)<.0001) {
            std::cout << "MATCH!\n";
          } else {
            good_antenna_locations=false;
            std::cout << f <<"\t" << y1 << "\n";
            std::cout << "MISMATCH!\n";
          }
          //Check z coordinate
          f = strtod((*++itField).c_str(),NULL);
          std::cout << "\t actual\t" << f << std::endl;
          std::cout << "\t desired\t" << z1 << std::endl;
          std::cout << "\t Difference\t" << abs(f-z1) << std::endl;
          if (abs(f-z1)<.0001) {
            std::cout << "MATCH!\n";
          } else {
            good_antenna_locations=false;
            std::cout << f << "\t" <<z1 << "\n";
            std::cout << "MISMATCH!\n ";
          }
        } else if (i==2) {
          std::cout << "\tX coordinate:\n";
          //Check x coordinate
          f = strtod((*++itField).c_str(),NULL);
          std::cout << "\t actual\t" << f << std::endl;
          std::cout << "\t desired\t" << x2 << std::endl;
          std::cout << "\t Difference\t" << abs(f-x2) << std::endl;
          if (abs(f-x2)<.0001) {
            std::cout << "MATCH!\n";
          } else {
            good_antenna_locations = false;
            std::cout << f <<"\t" << x2 <<"\n";
            std::cout << "MISMATCH!\n";
          }
          //Check y coordinate
          f = strtod((*++itField).c_str(),NULL);
          std::cout << "\t actual\t" << f << std::endl;
          std::cout << "\t desired\t" << y2 << std::endl;
          std::cout << "\t Difference\t" << abs(f-y2)<< std::endl;
          if (abs(f-y2)<.0001) {
            std::cout << "MATCH!\n";
          } else {
            good_antenna_locations=false;
            std::cout << f << "\t" << y2 << "\n";
            std::cout << "MISMATCH!\n";
          }
          //Check z coordinate
          f = strtod((*++itField).c_str(),NULL);
          std::cout << "\t actual\t" << f << std::endl;
          std::cout << "\t desired\t" << z2 << std::endl;
          std::cout << "\t Difference\t" << abs(f-z2) << std::endl;
          if (abs(f-z2)<.0001) {
            std::cout << "MATCH!\n";
          } else {
            good_antenna_locations=false;
            std::cout << f <<"\t" << z2 << "\n";
            std::cout << "MISMATCH!\n";
          }
        }

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //POST RESULT TO DATABASE!!!
        // PublishData("zAntennaLocations",block2, blockTime);
        // PublishData("zAntennaLocationsMatch",good_antenna_locations, blockTime);
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      } else {
        std::cout << "else\n";
        std::cout << *itField<<"#"<< std::endl;    
      }
    }
  }
  // std::cout << "\nASCII MESSAGE!!!\n";
  //cout<<block;
}


void Septentrio::ParseBinary(unsigned char* block, unsigned short ID)
{
  // std::cout << "Septentrio::ParseBinary" << std::endl;
  switch(ID) {
    // Reciever time (Current GPS and UTC time)
    case 5914:
      memcpy(&latest_receivertime_, block+8, sizeof(latest_receivertime_));
      receiver_time_callback(latest_receivertime_, read_timestamp);
      break;

    // Postion and velocity in XYZ
    case 5903:
      
      memcpy(&latest_pvtxyz_, block+8, sizeof(latest_pvtxyz_));
      pvt_cartesian_callback(latest_pvtxyz_, read_timestamp);
      latest_odometry_data_.pvt=latest_pvtxyz_;
      odometry_callback_(latest_odometry_data_,read_timestamp);
      break;

    // Position Covariance block
    case 5905:
      
      memcpy(&latest_pvtxyz_pos_cov_, block+8, sizeof(latest_pvtxyz_pos_cov_));
      pos_cov_cartesian_callback(latest_pvtxyz_pos_cov_, read_timestamp);
      latest_odometry_data_.pos_cov=latest_pvtxyz_pos_cov_;
      break;

    // Velocity Covariance block
    case 5907:
      memcpy(&latest_pvtxyz_vel_cov_, block+8, sizeof(latest_pvtxyz_vel_cov_));
      vel_cov_cartesian_callback(latest_pvtxyz_vel_cov_, read_timestamp);
      latest_odometry_data_.vel_cov=latest_pvtxyz_vel_cov_;
      break;

    // Attitude expressed as Euler Angle 
    case 5938: 
      memcpy(&latest_atteuler_, block+8, sizeof(latest_atteuler_));
      attitude_euler_callback(latest_atteuler_, read_timestamp);
      latest_odometry_data_.att=latest_atteuler_;
      break;

    //Attitude covariance (cross terms not currently given)
    case 5939:
      memcpy(&latest_atteuler_cov_, block+8, sizeof(latest_atteuler_cov_));
      attitude_cov_euler_callback(latest_atteuler_cov_, read_timestamp);
      latest_odometry_data_.att_cov=latest_atteuler_cov_;
      break;

    // //Range (MeasEpoch)
    // case 5889:
    //   update_range(block, blockTime);

    //   break;

    // //Ephemeris (GPSVav)
    // case 5891:
    //   SEP_EPHEMERIS latest_ephemeris;
    //   memcpy(&latest_ephemeris, block+8, sizeof(latest_ephemeris));
    //   update_ephemeris(blockTime);
    //   break;
  }
}


bool Septentrio::setAntennaLocations(int ant_num, double x, double y, double z)
{
  if ((ant_num != 1) && (ant_num != 2)) {
    // TODO:  log error here
    return false;
  }
  x1 = x;
  y1 = y;
  z1 = z;
  std::stringstream cmd;
  cmd <<"SetAntennaLocation manual " << ant_num << " " << x << " " << y << " " << z << "\r\n";
  return (serial_port->write(cmd.str())==cmd.str().length());
  //TODO: check for acknowledgement from receiver??
  // TODO: check positions??
  // TODO: merge this with the stuff in ParseASCII??
}


bool Septentrio::setRTK(std::string rtk_port, int rtk_baud, std::string rtk_format)
{
  std::cout << "\n\nSetting RTK on in the driver\n\n\n";
  //! TODO: check to make sure that rtk_type is one of the 3 supported options
  serial_port->write("SetPVTMode standalone+RTK \r\n");
  std::stringstream cmd;
  cmd << "Set" << rtk_format << "iNput " << rtk_port << " \r\n";
  serial_port->write(cmd.str());
  std::cout << "[Septentrio] ust wrote: " << cmd.str();
  cmd.str(std::string());
  cmd << "SetComSettings " << rtk_port << " " << rtk_baud << " \r\n";
  serial_port->write(cmd.str()); 
  std::cout << "[Septentrio] just wrote: " << cmd.str();
  // TODO: check output in ASCII
  // TODO: make this a bool function
  // TODO: check the covariance of output solution to make sure it's as low as
  //       an RTK solution should be.
  is_rtk = true;
  std::cout << "\n\n[Septentrio] Set RTK on in driver\n\n";
  return true;
}


bool Septentrio::setRTK()
{
  std::cout << "\n\nSetting RTK off in driver\n\n";
  serial_port->write("SetPVTMode standalone \r\n");
  is_rtk = false;
  return true;

}



void Septentrio::SetAttitudeMode(){
  std::cout << "\n\nSetting channel configuration to Attitude Dual Frequancy\n\n\n";
  serial_port->write("SetChannelConfiguration AttitudeDF auto\r\n");

}

void Septentrio::SetElevationMask(int mask_angle){
   std::stringstream cmd;
   std::cout << " Setting Mask angle \n";
  cmd << "SetElevationMask " << mask_angle << " \r\n";
  serial_port->write(cmd.str());

}














