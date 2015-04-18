//Written by:
//  Lowell Brown & David Hodo
//  Auburn University
//  Septentrio Documentation: http://www.septentrio.com/secure/polarx3_2_2/PolaRx2Manual.pdf

#include <septentrio/septentrio.h>


Septentrio::Septentrio()
{

  /* initialize serial reader values */
  display_messages = 0;//set to 1 to display a messages when a block is received
  good_antenna_locations = true;
  bytesRemaining = 0;
  bufIndex = 0;
  is_connected = false;

  
  setOutputRate("1");
  setRangeOutputRate("1");

  manual_position = false;
  x1 = 0.0292;
  y1 = 1.1464;
  z1 = 0;
  x2 = 1.9091;
  y2 = 0;
  z2 = 0;

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

  return true;
}


Septentrio::~Septentrio()
{
  Disconnect();
}


/////////////////////////////////////////
////////// Serial Port Methods //////////
/////////////////////////////////////////


bool Septentrio::Connect(std::string port, int baudrate=115200, std::string septentrio_port_="COM1")
{
  septentrio_port = septentrio_port_;
  //serial_port_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(1000));
  serial::Timeout my_timeout(100, 1000, 0, 1000, 0);
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

    serial_port_->flush();

    // !!!!!!!!!!!!!
    // Assume that connection is successful
    // !!!!!!!!!!!!!
    //
    // look for GPS by sending ping and waiting for response
    // if (!Ping()) {
    //   std::stringstream output;
    //   output << "Septentrio GPS not found on port: " << port << std::endl;
    //   log_error_(output.str());
    //   delete serial_port_;
    //   serial_port_ = NULL;
    //   is_connected_ = false;
    //   return false;
    // } 

  } catch (std::exception e) {
    // std::stringstream output;
    // output << "Failed to open port " << port << "  Err: " << e.what();
    // log_error_(output.str());
    serial_port_ = NULL;
    is_connected_ = false;
    return false;
  }

  // start reading
  StartReading();
  is_connected_ = true;

  std::string galcmd;
  
  if (manual_position) {
    cout<<"manual position true\t"<<x2<<endl;
    galcmd="gal\r\n";
    serial_port->write(galcmd)==galcmd.length();
    SetAntennaLocations(1, x1, y1, z1);
    SetAntennaLocations(2, x2, y2, z2);
  }


  return true;
}


void Septentrio::Disconnect()
{
  try {
    if (reading_status_) {
      StopReading();
      // TODO: wait here for reading to stop
    }
    if (serial_port_ != NULL) {
      if (serial_port_->isOpen())
        serial_port_->close();
      delete serial_port_;
      serial_port_ = NULL;
    }
  } catch (std::exception &e) {
    // stringstream output;
    // output << "Error disconnecting from ublox: " << e.what();
    // log_error_(output.str());
    // std::cout << "Error in Disconnect()";
  }
}


void Septentrio::StartReading() {
  try {
    // create thread to read from sensor
    reading_status = true;
    read_thread_ptr = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&Septentrio::ReadSerialPort, this)));
  } catch (std::exception &e) {
    // std::stringstream output;
    // output << "Error starting ublox read thread: " << e.what();
    // log_error_(output.str());
  }
}


void Septentrio::StopReading() {
    reading_status_ = false;
}


void Septentrio::ReadSerialPort() {
    uint8_t buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status) {
      // read data
      try {
          len = serial_port->read(buffer, MAX_NOUT_SIZE);
      } catch (exception &e) {
          // stringstream output;
          // output << "Error reading serial port: " << e.what();
          // log_info_(output.str());
          Disconnect();
          return;
      }
      // timestamp the read
      read_timestamp_ = time_handler_();
      // add data to the buffer to be parsed
      bufferIncomingData(buffer, len);
    }

}


//////////////////////////////
// Septentrio specific stuff for logs
/////////////////////////////////


bool Septentrio::RequestLog(std::string logcode_)
{
  string cmd="SetSBFOutput " + septentrio_port + ", " + logcode_ + "\r\n";
  return (serial_port->write(cmd)==cmd.length());
}


bool Septentrio::setOutputRate(string r)
{
  output_rate = r;
  string cmd = "SetPVTInterval " + output_rate + "\r\n";
  //TODO: read block rate from mission file
  return (serial_port->write(cmd)==cmd.length());
};


bool Septentrio::setRangeOutputRate(string r)
{
  range_output_rate = r;
  string cmd = "SetMeasInterval " + range_output_rate + "\r\n";
  //TODO: read block rate from mission file
  return (serial_port->write(cmd)==cmd.length());
};

//void gSeptentrio::display_sep_file(int block_ID)
//{
//    switch (block_ID)
//    {
//        case 5903: //PVTCartesian
//            cout << "\n\nPVTCartesian:" << endl;
//            cout << "TOW:  " << latest_pvtxyz_format.TOW << endl;
//            cout << "WNc:  " << latest_pvtxyz_format.WNc << endl;
//            cout << "NrSV:  " << latest_pvtxyz_format.NrSV << endl;
//            cout << "Error:  " << latest_pvtxyz_format.Error << endl;
//            cout << "Mode:  " << latest_pvtxyz_format.Mode << endl;
//            cout << "System:  " << latest_pvtxyz_format.System << endl;
//            cout << "Info:  " << latest_pvtxyz_format.Info << endl;
//            cout << "SBASprn:  " << latest_pvtxyz_format.SBASprn << endl;
//            cout << "x_position:  " << latest_pvtxyz_format.x_position << endl;
//            cout << "y_position:  " << latest_pvtxyz_format.y_position << endl;
//            cout << "z_position:  " << latest_pvtxyz_format.z_position << endl;
//            cout << "x_velocity:  " << latest_pvtxyz_format.x_velocity << endl;
//            cout << "y_velocity:  " << latest_pvtxyz_format.y_velocity << endl;
//            cout << "z_velocity:  " << latest_pvtxyz_format.z_velocity << endl;
//            cout << "RxClkBias:  " << latest_pvtxyz_format.RxClkBias << endl;
//            cout << "RxClkDrift:  " << latest_pvtxyz_format.RxClkDrift << endl;
//            cout << "MeanCorrAge:  " << latest_pvtxyz_format.MeanCorrAge << endl;
//            cout << "RefrenceID:  " << latest_pvtxyz_format.ReferenceID << endl;
//            cout << "COG:  " << latest_pvtxyz_format.COG << endl;
//            break;
//
//        case 5938:  //AttEuler
//            cout << "\n\nAttitudeEuler:" << endl;
//            cout << "TOW:  " << latest_atteuler_format.TOW_b << endl;
//            cout << "WNc:  " << latest_atteuler_format.WNc_b << endl;
//            cout << "NrSV:  " << latest_atteuler_format.NrSV_b << endl;
//            cout << "Error:  " << latest_atteuler_format.Error_b << endl;
//            cout << "Mode:  " << latest_atteuler_format.Mode_b << endl;
//            cout << "Heading:  " << latest_atteuler_format.Heading << endl;
//            cout << "Pitch:  " << latest_atteuler_format.Pitch << endl;
//            cout << "Roll:  " << latest_atteuler_format.Roll << endl;
//            cout << "Omega x:  " << latest_atteuler_format.x_omega << endl;
//            cout << "Omega Y:  " << latest_atteuler_format.y_omega << endl;
//            cout << "Omega Z:  " << latest_atteuler_format.z_omega << endl;
//            break;
//    }
//}


// // Could not get CRC to work properly //
// unsigned short Septentrio::ComputeCRC(char * buf, int buf_length)
// {
//       int  i;
//       unsigned short  crc = 0;

//        see for example the BINEX web site 
//       for (i=0; i < buf_length; i++) {
//             crc = (crc << 8) ^ CRCLookUp[ (crc >> 8) ^ buf[i] ];
//       }

//       return crc;
// }


// bool Septentrio::CRC_check(char *head, char *mess, int tot_length, int CRC)
// {
//     int j;
//     int Calculate_CRC;
//     char *CRC_message = new char[tot_length - 4];

//     //** Have to merge two character arrays into one to calculate CRC **//
//     for(j = 0; j <= 3 ; j++)
//     {
//         CRC_message[j] = head[j + 2];
//     }

//     for(j = 0; j <= tot_length - 8; j++)
//     {
//         CRC_message[j + 4] = mess[j];
//     }

//     Calculate_CRC = ComputeCRC(CRC_message, tot_length);

//     if (Calculate_CRC == CRC)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
//     delete [] CRC_message;

// }


// //probably want to change to generic selected COM port for NMEA output message
// bool Septentrio::RequestNMEACOM3()
// {
//   string cmd="SetNMEAOutput COM3, GLL+GSV #EoL\r\n";
// //GLL and GSV place holders until Sarnoff knows what they want

// return (SendString(cmd)==cmd.length());
// }

// bool Septentrio::ConfigureLogging(string &sLogs)
// {
//   //figure out what we are required to log....
//     //here we read in what we want to log from the mission file..
//     STRING_LIST Params;
//     if(m_MissionReader.GetConfiguration(m_sAppName,Params))
//     {
//         //this will make columns in sync log in order they
//         //were declared in *.moos file
//         Params.reverse();
    
//     //string sLogs="";
//         STRING_LIST::iterator p;
//         for(p=Params.begin();p!=Params.end();p++)
//         {
//             string sParam = *p;
//             string sWhat = MOOSChomp(sParam,"=");
    
//             if(MOOSStrCmp(sWhat,"LOG")) 
//       {
//         sLogs+=sParam+"+";
//             }//end (MOOSStrCmp(sWhat,"LOG"))
//         }//end for(p=Params.begin();p!=Params.end();p++)
//     sLogs.erase(sLogs.length()-1,1);
//     } //end if(m_MissionReader.GetConfiguration(m_sAppName,Params))
//   else
//     {
//         MOOSTrace("Warning:\n\tNo Configuration block was read - unusual but not terminal\n");
//     }
//   return true;
// }


void Septentrio::bufferIncomingData(uint8_t *msg, size_t length)
{
  //cout<<"Buffering\n";
    
  // add incoming data to buffer
  for (unsigned int i=0; i<length; i++) 
  {
    //cout << i << ": " << hex << (int)msg[i] << dec  << " : " << bytesRemaining  << " : " << bufIndex << endl;
    // make sure bufIndex is not larger than buffer
    if (bufIndex>=MAX_MSG_SIZE)
    {
      bufIndex=0;
      MOOSTrace("Septentrio: Overflowed receive buffer. Reset");
    }

    if (bufIndex==0)
    { // looking for beginning of message
      if (msg[i]==0x24)//0x24 is ASCII for $
      { // beginning of msg found - add to buffer
        dataBuf[bufIndex++]=msg[i];
        bytesRemaining=0;
        readingASCII=false;
      } // end if (msg[i]
      
    } // end if (bufIndex==0)
    else if (bufIndex==1)
    { // verify 2nd character of header
      if (msg[i]==0x40) //0x24 is ASCII for @
      { // 2nd byte ok - add to buffer
        dataBuf[bufIndex++]=msg[i];
        //cout<<"Binary Message\n";
      }
      else if (msg[i]==0x50) //0x50 is ASCII for P, indicates
      {
        readingASCII=true;
        dataBuf[bufIndex++]=msg[i];
        //cout<<"ASCII Message\n";
      } else {
        // start looking for new message again
        bufIndex=0;
        bytesRemaining=0;
        readingASCII=false;
      } // end if (msg[i]==0x40)
    } // end else if (bufIndex==1)http://www.septentrio.com/secure/polarx3_2_2/PolaRx2Manual.pdf
    else if (bufIndex==7)
    { // reading last byte of header
        //cout<<"(bufIndex==7)\n";
        dataBuf[bufIndex++]=msg[i]; 
        //we should have the entire header now
        if (!readingASCII)
        {
          memcpy(&latest_header, dataBuf, 8);
          bytesRemaining = latest_header.Length - 8;//!< How many bytes are left to read in the message
        }
    } // end else if (bufIndex==7)
    else if (bytesRemaining==1)
    { // add last byte and parse
      //cout<<"(bytesRemaining==1)\n";
      dataBuf[bufIndex++]=msg[i];
      ParseBlock(dataBuf,latest_header.ID);
      // reset counters
      bufIndex=0;
      bytesRemaining=0;
      //cout << "Message Done." << endl;
    }  // end else if (bytesRemaining==1)
    else if ((bytesRemaining>1)||((bufIndex>1)&&(bufIndex<7)))
    { // add data to buffer
      dataBuf[bufIndex++]=msg[i];
      if (!readingASCII)
        bytesRemaining--;
    }
    else if ((readingASCII)&&(bufIndex>7))
    {
      //cout<<"reading ascii message\n";
      //cout<<hex<<(int)msg[i]<<endl;
      if (!((msg[i]==0x23)||(msg[i]==0x0D))) //looking for end of line character, a pound symbol from the septentrio, hex 0x23
      {
        dataBuf[bufIndex++]=msg[i];
      }
      else
      {
        dataBuf[bufIndex++]=0;
        ParseASCII(dataBuf);
        readingASCII=false;
        bufIndex=0;
      }
    }
  } // end for
}

void Septentrio::ParseASCII(unsigned char* block)
{ 
  double blockTime=MOOSTime();
  std::stringstream ss (std::stringstream::in | std::stringstream::out);
  std::string block2;
  //string block="$PolaRx: SetAntennaLocation auto, 2, 5.23, 10.00, -2.000\n $PolaRx: SetAntennaLocation manual, 2, 0.47355, 10.51, -2.143\n";
  cout<<"BLOCKS:\n"<<block<<endl;
  ss<<block;
  getline(ss,block2);
  cout<<block2<<endl;
   
  int i;
  float f;
  
  char_separator<char> sep("\n");
  tokenizer<char_separator<char> > lines(block2,sep);
  BOOST_FOREACH(std::string line, lines)
  {
    cout<<"LINE:\n";
    char_separator<char> sep2(", ");
    tokenizer<char_separator<char> > tokens(line,sep2);

    tokenizer<char_separator<char> >::iterator itField = tokens.begin();
    itField++;//advance past header
    //determine ASCII message type
    if (*itField=="SetAntennaLocation")//Response to "gal" (get antenna locations)
    {
      if (*++itField=="auto")//determine antenna location mode
      {
        good_antenna_locations=false;
        cout<<"Antenna Locations set to AUTO\n";
        //send correct antenna locations
      }
      else if (*itField=="manual")
      {
        
        cout<<"Antenna Locations set to MANUAL\n";
        //check antenna location, resend only if necessary
        i=strtod((*++itField).c_str(),NULL);
        cout<<"Checking location of antenna # "<<i<<":"<<endl;
        if (i==1)
        {
          cout<<"\tX coordinate:\n";
          //Check x coordinate
          f=strtod((*++itField).c_str(),NULL);    
          cout<<"\t actual\t"<<f<<endl;
          cout<<"\t desired\t"<<strtod(x1.c_str(),NULL)<<endl;
          cout<<"\t Difference\t"<<abs(f-strtod(x1.c_str(),NULL))<<endl;
          if (abs(f-strtod(x1.c_str(),NULL))<.0001)
            cout<<"MATCH!\n";
          else
          {
            good_antenna_locations=false;
            cout<<f<<"\t"<<strtod(x1.c_str(),NULL)<<"\n";
            cout<<"MISMATCH!\n";
          }
          //Check y coordinate
          f=strtod((*++itField).c_str(),NULL);
          cout<<"\t actual\t"<<f<<endl;
          cout<<"\t desired\t"<<strtod(y1.c_str(),NULL)<<endl;
          cout<<"\t Difference\t"<<abs(f-strtod(y1.c_str(),NULL))<<endl;
          if (abs(f-strtod(y1.c_str(),NULL))<.0001)
            cout<<"MATCH!\n";
          else
          {
            good_antenna_locations=false;
            cout<<f<<"\t"<<strtod(y1.c_str(),NULL)<<"\n";
            cout<<"MISMATCH!\n";
          }
          //Check z coordinate
          f=strtod((*++itField).c_str(),NULL);
          cout<<"\t actual\t"<<f<<endl;
          cout<<"\t desired\t"<<strtod(z1.c_str(),NULL)<<endl;
          cout<<"\t Difference\t"<<abs(f-strtod(z1.c_str(),NULL))<<endl;
          if (abs(f-strtod(z1.c_str(),NULL))<.0001)
            cout<<"MATCH!\n";
          else
          {
            good_antenna_locations=false;
            cout<<f<<"\t"<<strtod(z1.c_str(),NULL)<<"\n";
            cout<<"MISMATCH!\n";
          }
        }
        else if (i==2)
        {
          cout<<"\tX coordinate:\n";
          //Check x coordinate
          f=strtod((*++itField).c_str(),NULL);
          cout<<"\t actual\t"<<f<<endl;
          cout<<"\t desired\t"<<strtod(x2.c_str(),NULL)<<endl;
          cout<<"\t Difference\t"<<abs(f-strtod(x2.c_str(),NULL))<<endl;
          if (abs(f-strtod(x2.c_str(),NULL))<.0001)
            cout<<"MATCH!\n";
          else
          {
            good_antenna_locations=false;
            cout<<f<<"\t"<<strtod(x2.c_str(),NULL)<<"\n";
            cout<<"MISMATCH!\n";
          }
          //Check y coordinate
          f=strtod((*++itField).c_str(),NULL);
          cout<<"\t actual\t"<<f<<endl;
          cout<<"\t desired\t"<<strtod(y2.c_str(),NULL)<<endl;
          cout<<"\t Difference\t"<<abs(f-strtod(y2.c_str(),NULL))<<endl;
          if (abs(f-strtod(y2.c_str(),NULL))<.0001)
            cout<<"MATCH!\n";
          else
          {
            good_antenna_locations=false;
            cout<<f<<"\t"<<strtod(y2.c_str(),NULL)<<"\n";
            cout<<"MISMATCH!\n";
          }
          //Check z coordinate
          f=strtod((*++itField).c_str(),NULL);
          cout<<"\t actual\t"<<f<<endl;
          cout<<"\t desired\t"<<strtod(z2.c_str(),NULL)<<endl;
          cout<<"\t Difference\t"<<abs(f-strtod(z2.c_str(),NULL))<<endl;
          if (abs(f-strtod(z2.c_str(),NULL))<.0001)
            cout<<"MATCH!\n";
          else {
            good_antenna_locations=false;
            cout<<f<<"\t"<<strtod(z2.c_str(),NULL)<<"\n";
            cout<<"MISMATCH!\n";
          }
        }
        //POST RESULT TO DATABASE!!!
        PublishToDatabase("zAntennaLocations",block2, blockTime);
        PublishToDatabase("zAntennaLocationsMatch",good_antenna_locations, blockTime);
      }
      else
      {
        cout<<"else\n";
        cout<<*itField<<"#"<<endl;    
      }
    }
  }
  cout<<"\nASCII MESSAGE!!!\n";
  //cout<<block;
}
void Septentrio::ParseBlock(unsigned char* block, unsigned short ID)
{
  //MOOSTrace("Received ID: %i \r\n",ID);
  double blockTime=MOOSTime();
  switch(ID)
  {
    case 5914: // Reciever time (Current GPS and UTC time)
      if(display_messages==1){
        cout << "Received ReceiverTime Block" << endl;
      }
      memcpy(&latest_receivertime,block+8,sizeof(latest_receivertime));
      break;
    case 5903: // Postion and velocity in XYZ
      if(display_messages==1){
        cout << "Received PVTCartesian Block" << endl;
      }
      memcpy(&latest_pvtxyz,block+8,sizeof(latest_pvtxyz));
      PublishToDatabase("zGPSSeconds_PVT",latest_pvtxyz.GPS_ms,        blockTime);
      PublishToDatabase("zGPSWeek_PVT",   latest_pvtxyz.weekNumber,    blockTime);
      PublishToDatabase("zNumSats_PVT",   latest_pvtxyz.numSat,        blockTime);

      if(latest_pvtxyz.x_position!=-2e10){
        PublishToDatabase("zECEF_X",    stringUtils::precise_to_string(latest_pvtxyz.x_position,10),    blockTime);
      }
      if(latest_pvtxyz.y_position!=-2e10){
        PublishToDatabase("zECEF_Y",    stringUtils::precise_to_string(latest_pvtxyz.y_position,10),    blockTime);
      }
      if(latest_pvtxyz.z_position!=-2e10){
        PublishToDatabase("zECEF_Z",    stringUtils::precise_to_string(latest_pvtxyz.z_position,10),    blockTime);
      }
      if(latest_pvtxyz.x_velocity!=-2e10){
        PublishToDatabase("zECEF_VX",   latest_pvtxyz.x_velocity,    blockTime);
      }
      if(latest_pvtxyz.y_velocity!=-2e10){
        PublishToDatabase("zECEF_VY",   latest_pvtxyz.y_velocity,    blockTime);
      }
      if(latest_pvtxyz.z_velocity!=-2e10){
        PublishToDatabase("zECEF_VZ",   latest_pvtxyz.z_velocity,    blockTime);
      }
      if(latest_pvtxyz.course!=-2e10){
        PublishToDatabase("zCourse",    latest_pvtxyz.course*PI/180, blockTime);
      }
      if(latest_pvtxyz.MeanCorrAge!=65535){
        PublishToDatabase("zDiffAge",   latest_pvtxyz.MeanCorrAge,   blockTime);
      }
      PublishToDatabase("zError_PVT", latest_pvtxyz.Error,         blockTime);
      PublishToDatabase("zMode_PVT",  latest_pvtxyz.Mode,          blockTime);
      break;
    case 5905: // Position Covariance block
      if(display_messages==1){
        cout << "Received PosCovCartesian Block" << endl;
      }
      memcpy(&latest_pvtxyz_pos_cov,block+8,sizeof(latest_pvtxyz_pos_cov));
      PublishToDatabase("zGPSSeconds_Pos_Cov",    latest_pvtxyz_pos_cov.GPS_ms,         blockTime);

      if(latest_pvtxyz_pos_cov.Cov_xx!=-2e10){
        PublishToDatabase("zECEF_X_Var",    latest_pvtxyz_pos_cov.Cov_xx,    blockTime);
      }
      if(latest_pvtxyz_pos_cov.Cov_yy!=-2e10){
        PublishToDatabase("zECEF_Y_Var",    latest_pvtxyz_pos_cov.Cov_yy,    blockTime);
      }
      if(latest_pvtxyz_pos_cov.Cov_zz!=-2e10){
        PublishToDatabase("zECEF_Z_Var",    latest_pvtxyz_pos_cov.Cov_zz,    blockTime);
      }
      
      break;
    case 5907: // Velocity Covariance block
      if(display_messages==1){
        cout << "Received VelCovCartesian Block" << endl;
      }
      memcpy(&latest_pvtxyz_vel_cov,block+8,sizeof(latest_pvtxyz_vel_cov));
      PublishToDatabase("zGPSSeconds_Vel_Cov",    latest_pvtxyz_vel_cov.GPS_ms,         blockTime);

      if(latest_pvtxyz_vel_cov.Cov_VxVx!=-2e10){
        PublishToDatabase("zECEF_VX_Var",   latest_pvtxyz_vel_cov.Cov_VxVx,    blockTime);
      }
      if(latest_pvtxyz_vel_cov.Cov_VyVy!=-2e10){
        PublishToDatabase("zECEF_VY_Var",   latest_pvtxyz_vel_cov.Cov_VyVy,    blockTime);
      }
      if(latest_pvtxyz_vel_cov.Cov_VzVz!=-2e10){
        PublishToDatabase("zECEF_VZ_Var",   latest_pvtxyz_vel_cov.Cov_VzVz,    blockTime);
      }
      
      break;
    case 5938: // Attitude expressed as Euler Angle
      if(display_messages==1){
        cout << "Received AttitudeEuler Block" << endl;
      }
      memcpy(&latest_atteuler,block+8,sizeof(latest_atteuler));
      PublishToDatabase("zGPSSeconds_ATT",latest_atteuler.GPS_ms,         blockTime);
      PublishToDatabase("zGPSWeek_ATT",   latest_atteuler.weekNumber,     blockTime);
      PublishToDatabase("zNumSats_ATT",   latest_atteuler.numSat,         blockTime);

      if(latest_atteuler.heading!=-2e10){
        PublishToDatabase("zYaw",       latest_atteuler.heading*PI/180, blockTime);
      }
      if(latest_atteuler.pitch!=-2e10){
        PublishToDatabase("zPitch",     latest_atteuler.pitch  *PI/180, blockTime);
      }
      if(latest_atteuler.roll!=-2e10){
        PublishToDatabase("zRoll",      latest_atteuler.roll   *PI/180, blockTime);
      }
      PublishToDatabase("zError_ATT", latest_atteuler.error,          blockTime);
      PublishToDatabase("zMode_ATT",  latest_atteuler.mode,           blockTime);

      break;
    case 5939: //Attitude covariance (cross terms not currently given)
      if(display_messages==1){
        cout << "Received AttitudeCovEuler Block" << endl;
      }
      memcpy(&latest_atteuler_cov,block+8,sizeof(latest_atteuler_cov));
      PublishToDatabase("zGPSSeconds_ATT_Cov",latest_atteuler_cov.GPS_ms,         blockTime);

      //these are published rad^2 (converted from deg^2)
      if(latest_atteuler_cov.var_heading!=-2e10){
        PublishToDatabase("zYaw_Var",   latest_atteuler_cov.var_heading *(PI/180)*(PI/180), blockTime);
      }
      if(latest_atteuler_cov.var_pitch!=-2e10){
        PublishToDatabase("zPitch_Var", latest_atteuler_cov.var_pitch   *(PI/180)*(PI/180), blockTime);
      }
      if(latest_atteuler_cov.var_roll!=-2e10){
        PublishToDatabase("zRoll_Var",  latest_atteuler_cov.var_roll    *(PI/180)*(PI/180), blockTime);
      }
      
      break;
    case 5889: //Range (MeasEpoch)
      if(display_messages==1){
        cout << "Received MeasEpoch Block" << endl;
      }
      update_range(block, blockTime);

      break;
    case 5891: //Ephemeris (GPSVav)
      memcpy(&latest_ephemeris,block+8,sizeof(latest_ephemeris));
      if(display_messages==1){
        cout << "Received GPSNav Block for SV" << (unsigned int)latest_ephemeris.PRN << endl;
      }
      update_ephemeris(blockTime);

  }
}


void Septentrio::update_ephemeris(double &blockTime){

  double ephems[22];
  ephems[0] = (unsigned int)latest_ephemeris.PRN;
  ephems[1] = latest_ephemeris.t_oe;
  ephems[2] = latest_ephemeris.SQRT_A;
  ephems[3] = latest_ephemeris.DELTA_N;
  ephems[4] = latest_ephemeris.M_0;
  ephems[5] = latest_ephemeris.e;
  ephems[6] = latest_ephemeris.omega;
  ephems[7] = latest_ephemeris.C_uc;
  ephems[8] = latest_ephemeris.C_us;
  ephems[9] = latest_ephemeris.C_rc;
  ephems[10] = latest_ephemeris.C_rs;
  ephems[11] = latest_ephemeris.C_ic;
  ephems[12] = latest_ephemeris.C_is;
  ephems[13] = latest_ephemeris.i_0;
  ephems[14] = latest_ephemeris.IDOT;
  ephems[15] = latest_ephemeris.OMEGA_0;
  ephems[16] = latest_ephemeris.OMEGADOT;
  ephems[17] = latest_ephemeris.t_oc;
  ephems[18] = latest_ephemeris.T_gd;
  ephems[19] = latest_ephemeris.a_f0;
  ephems[20] = latest_ephemeris.a_f1;
  ephems[21] = latest_ephemeris.a_f2; 

  std::string VarName="zEphem"+stringUtils::to_string((unsigned int)latest_ephemeris.PRN);
  //PublishToDatabase("zGPSSeconds",latest_ephemeris.GPS_ms,         blockTime);
  PublishToDatabase(VarName,     stringUtils::precise_to_string(ephems,22,16), blockTime);

}
void Septentrio::update_range(unsigned char* block, double &blockTime){

  int number_observations_1=0;
  int number_observations_2=0;
  int number_observations_3=0;

  int i=0;
  int byte_track=0;
  unsigned int antenna;
  
  memcpy(&latest_range_heading,block+8,sizeof(latest_range_heading));

  while(i<(unsigned int)latest_range_heading.N)
  {
    memcpy(&latest_range_sub_block,block+(byte_track+16),sizeof(latest_range_sub_block));
    antenna=(unsigned int)latest_range_sub_block.Flags.antennaID;

    switch(antenna){
      case 1:
        latest_range_data.SVID_1[number_observations_1]= (unsigned int)latest_range_sub_block.SVID;
        latest_range_data.CACode_1[number_observations_1]= (double)latest_range_sub_block.CACode;
        //latest_range_data.P1_CACode_1[number_observations_1]= (double)latest_range_sub_block.CACode + (double)latest_range_sub_block.P1_CACode;
        //latest_range_data.P2_CACode_1[number_observations_1]= (double)latest_range_sub_block.CACode + (double)latest_range_sub_block.P2_CACode;
        latest_range_data.L1Phase_1[number_observations_1]= (double)latest_range_sub_block.L1Phase;
        //latest_range_data.L2Phase_1[number_observations_1]= (double)latest_range_sub_block.L2Phase;
        latest_range_data.L1Doppler_1[number_observations_1]= (double)latest_range_sub_block.L1Doppler * .0001;
        //latest_range_data.L2Doppler_1[number_observations_1]= (double)latest_range_sub_block.L2Doppler * .0001;
        latest_range_data.CAC2N_1[number_observations_1]= (signed int)latest_range_sub_block.CACN0 * .1;
        //latest_range_data.P1C2N_1[number_observations_1]= (signed int)latest_range_sub_block.P1CN0 * .1;
        //latest_range_data.P2C2N_1[number_observations_1]= (signed int)latest_range_sub_block.P2CN0 * .1;
        number_observations_1=number_observations_1+1;

        break;
      case 2:
        
        latest_range_data.SVID_2[number_observations_2]= (unsigned int)latest_range_sub_block.SVID;
        latest_range_data.CACode_2[number_observations_2]= (double)latest_range_sub_block.CACode;
        //latest_range_data.P1_CACode_2[number_observations_2]= (double)latest_range_sub_block.CACode + (double)latest_range_sub_block.P1_CACode;
        //latest_range_data.P2_CACode_2[number_observations_2]= (double)latest_range_sub_block.CACode + (double)latest_range_sub_block.P2_CACode;
        latest_range_data.L1Phase_2[number_observations_2]= (double)latest_range_sub_block.L1Phase;
        //latest_range_data.L2Phase_2[number_observations_2]= (double)latest_range_sub_block.L2Phase;
        latest_range_data.L1Doppler_2[number_observations_2]= (double)latest_range_sub_block.L1Doppler * .0001;
        //latest_range_data.L2Doppler_2[number_observations_2]= (double)latest_range_sub_block.L2Doppler * .0001;
        latest_range_data.CAC2N_2[number_observations_2]= (signed int)latest_range_sub_block.CACN0 * .1;
        //latest_range_data.P1C2N_2[number_observations_2]= (signed int)latest_range_sub_block.P1CN0 * .1;
        //latest_range_data.P2C2N_2[number_observations_2]= (signed int)latest_range_sub_block.P2CN0 * .1;
        number_observations_2=number_observations_2+1;

        break;
      case 3:
        latest_range_data.SVID_3[number_observations_3]= (unsigned int)latest_range_sub_block.SVID;
        latest_range_data.CACode_3[number_observations_3]= (double)latest_range_sub_block.CACode;
        //latest_range_data.P1_CACode_3[number_observations_3]= (double)latest_range_sub_block.CACode + (double)latest_range_sub_block.P1_CACode;
        //latest_range_data.P2_CACode_3[number_observations_3]= (double)latest_range_sub_block.CACode + (double)latest_range_sub_block.P2_CACode;
        latest_range_data.L1Phase_3[number_observations_3]= (double)latest_range_sub_block.L1Phase;
        //latest_range_data.L2Phase_3[number_observations_3]= (double)latest_range_sub_block.L2Phase;
        latest_range_data.L1Doppler_3[number_observations_3]= (double)latest_range_sub_block.L1Doppler * .0001;
        //latest_range_data.L2Doppler_3[number_observations_3]= (double)latest_range_sub_block.L2Doppler * .0001;
        latest_range_data.CAC2N_3[number_observations_3]= (signed int)latest_range_sub_block.CACN0 * .1;
        //latest_range_data.P1C2N_3[number_observations_3]= (signed int)latest_range_sub_block.P1CN0 * .1;
        //latest_range_data.P2C2N_3[number_observations_3]= (signed int)latest_range_sub_block.P2CN0 * .1;
        number_observations_3=number_observations_3+1;
    }

    i=i+1;
    byte_track=byte_track+(unsigned int)latest_range_heading.SBLength;
  }

  PublishToDatabase("zGPSSeconds_Range",latest_range_heading.GPS_ms,        blockTime);
  PublishToDatabase("zGPSWeek_Range",   latest_range_heading.weekNumber,    blockTime);
  PublishToDatabase("zNumObservations1",number_observations_1,              blockTime);
  PublishToDatabase("zNumObservations2",number_observations_2,              blockTime);
  PublishToDatabase("zNumObservations3",number_observations_3,              blockTime);


  PublishToDatabase("zSVID_1",  stringUtils::to_string(latest_range_data.SVID_1,number_observations_1), blockTime);
  PublishToDatabase("zCACode_1",  stringUtils::precise_to_string(latest_range_data.CACode_1,number_observations_1,12), blockTime);
  //PublishToDatabase("zP1_CACode_1",  stringUtils::precise_to_string(latest_range_data.P1_CACode_1,number_observations_1,12), blockTime);
  //PublishToDatabase("zP2_CACode_1",  stringUtils::precise_to_string(latest_range_data.P2_CACode_1,number_observations_1,12), blockTime);
  PublishToDatabase("zL1Phase_1",  stringUtils::precise_to_string(latest_range_data.L1Phase_1,number_observations_1,14), blockTime);
  //PublishToDatabase("zL2Phase_1",  stringUtils::precise_to_string(latest_range_data.L2Phase_1,number_observations_1,14), blockTime);
  PublishToDatabase("zL1Doppler_1",  stringUtils::precise_to_string(latest_range_data.L1Doppler_1,number_observations_1,10), blockTime);
  //PublishToDatabase("zL2Doppler_1",  stringUtils::precise_to_string(latest_range_data.L2Doppler_1,number_observations_1,10), blockTime);
  PublishToDatabase("zCAC2N_1",  stringUtils::precise_to_string(latest_range_data.CAC2N_1,number_observations_1,5), blockTime);
  //PublishToDatabase("zP1C2N_1",  stringUtils::precise_to_string(latest_range_data.P1C2N_1,number_observations_1,5), blockTime);
  //PublishToDatabase("zP2C2N_1",  stringUtils::precise_to_string(latest_range_data.P2C2N_1,number_observations_1,5), blockTime);

  PublishToDatabase("zSVID_2",  stringUtils::to_string(latest_range_data.SVID_2,number_observations_2), blockTime);
  PublishToDatabase("zCACode_2",  stringUtils::precise_to_string(latest_range_data.CACode_2,number_observations_2,12), blockTime);
  //PublishToDatabase("zP1_CACode_2",  stringUtils::precise_to_string(latest_range_data.P1_CACode_2,number_observations_2,12), blockTime);
  //PublishToDatabase("zP2_CACode_2",  stringUtils::precise_to_string(latest_range_data.P2_CACode_2,number_observations_2,12), blockTime);
  PublishToDatabase("zL1Phase_2",  stringUtils::precise_to_string(latest_range_data.L1Phase_2,number_observations_2,14), blockTime);
  //PublishToDatabase("zL2Phase_2",  stringUtils::precise_to_string(latest_range_data.L2Phase_2,number_observations_2,14), blockTime);
  PublishToDatabase("zL1Doppler_2",  stringUtils::precise_to_string(latest_range_data.L1Doppler_2,number_observations_2,10), blockTime);
  //PublishToDatabase("zL2Doppler_2",  stringUtils::precise_to_string(latest_range_data.L2Doppler_2,number_observations_2,10), blockTime);
  PublishToDatabase("zCAC2N_2",  stringUtils::precise_to_string(latest_range_data.CAC2N_2,number_observations_2,5), blockTime);
  //PublishToDatabase("zP1C2N_2",  stringUtils::precise_to_string(latest_range_data.P1C2N_2,number_observations_2,5), blockTime);
  //PublishToDatabase("zP2C2N_2",  stringUtils::precise_to_string(latest_range_data.P2C2N_2,number_observations_2,5), blockTime);

  PublishToDatabase("zSVID_3",  stringUtils::to_string(latest_range_data.SVID_3,number_observations_3), blockTime);
  PublishToDatabase("zCACode_3",  stringUtils::precise_to_string(latest_range_data.CACode_3,number_observations_3,12), blockTime);
  //PublishToDatabase("zP1_CACode_3",  stringUtils::precise_to_string(latest_range_data.P1_CACode_3,number_observations_3,12), blockTime);
  //PublishToDatabase("zP2_CACode_3",  stringUtils::precise_to_string(latest_range_data.P2_CACode_3,number_observations_3,12), blockTime);
  PublishToDatabase("zL1Phase_3",  stringUtils::precise_to_string(latest_range_data.L1Phase_3,number_observations_3,14), blockTime);
  //PublishToDatabase("zL2Phase_3",  stringUtils::precise_to_string(latest_range_data.L2Phase_3,number_observations_3,14), blockTime);
  PublishToDatabase("zL1Doppler_3",  stringUtils::precise_to_string(latest_range_data.L1Doppler_3,number_observations_3,10), blockTime);
  //PublishToDatabase("zL2Doppler_3",  stringUtils::precise_to_string(latest_range_data.L2Doppler_3,number_observations_3,10), blockTime);
  PublishToDatabase("zCAC2N_3",  stringUtils::precise_to_string(latest_range_data.CAC2N_3,number_observations_3,5), blockTime);
  //PublishToDatabase("zP1C2N_3",  stringUtils::precise_to_string(latest_range_data.P1C2N_3,number_observations_3,5), blockTime);
  //PublishToDatabase("zP2C2N_3",  stringUtils::precise_to_string(latest_range_data.P2C2N_3,number_observations_3,5), blockTime);

}
bool Septentrio::SetAntennaLocations(int ant_num, std::string x, std::string y, std::string z)
{

/*
  string septentrioCom; //The comm port number on the Septentrio reciever
  if(!m_MissionReader.GetConfigurationParam("SEPTENTRIO_COM",septentrioCom))
  {
    MOOSTrace("SEPTENTRIO_COM not set in mission file - Set to Default: COM1\n");
    septentrioCom="COM1";
  }//Be sure to set in mission file
*/    
  
  std::string cmd;

  if (ant_num==1) {
    cmd = "SetAntennaLocation manual 1 " + x + " " + y + " " + z + "\r\n";
    return (serial_port->write(cmd)==cmd.length());
  } else if (ant_num==2) {
    cmd = "SetAntennaLocation manual 2 " + x + " " + y + " " + z + "\r\n";
    return (serial_port->write(cmd)==cmd.length());
  } else {
    // MOOSTrace("ERROR!!!!\n");
    return 0;
  }

  //TODO: check for acknowledgement from receiver??
}

// void Septentrio::SetRTK(string RTK_com,string RTK_baud,string RTK_correction_type)
// {
//   string cmd;

//   cmd="SetPVTMode standalone+RTK \r\n";
//   SendString(cmd)==cmd.length();
//   cmd="Set"+RTK_correction_type+"iNput "+RTK_com+" \r\n";
//   SendString(cmd)==cmd.length();
//   cmd="SetComSettings "+RTK_com+" "+RTK_baud+" \r\n";
//   SendString(cmd)==cmd.length();

// }
