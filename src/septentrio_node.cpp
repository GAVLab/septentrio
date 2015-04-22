/*
 * ROS Node that wraps the Septentrio driver
 */
#include <septentrio/septentrio_node.h>


inline void printHex(char *data, int length) {
    for (int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned) (unsigned char) data[i]);
    }
    printf("\n");
}


SeptentrioNode::SeptentrioNode()
{
  // receiver_time_pub =       nh.advertise<septentrio::>
  pvt_cartesian_pub =       nh.advertise<septentrio::PvtCartesianMsg>(    "pvt_cartesian",      1000);
  pos_cov_cartesian_pub =   nh.advertise<septentrio::PosCovCartesianMsg>( "pos_cov_cartesian",  1000);
  vel_cov_cartesian_pub =   nh.advertise<septentrio::VelCovCartesianMsg>( "vel_cov_cartesian",  1000);
  attitude_euler_pub =      nh.advertise<septentrio::AttitudeEulerMsg>(   "attitude_euler",     1000);
  attitude_cov_euler_pub =  nh.advertise<septentrio::AttitudeCovEulerMsg>("attitude_cov_euler", 1000);

  // setReceiverTimeCallback
  gps.setPvtCartesianCallback(     boost::bind(&SeptentrioNode::pvtCartestianCallback,    this, _1, _2) );
  gps.setPosCovCartesianCallback(  boost::bind(&SeptentrioNode::posCovCartesianCallback,  this, _1, _2) );
  gps.setVelCovCaresianCallback(   boost::bind(&SeptentrioNode::velCovCaresianCallback,   this, _1, _2) );
  gps.setAttitudeEulerCallback(    boost::bind(&SeptentrioNode::attitudeEulerCallback,    this, _1, _2) );
  gps.setAttitudeCovEulerCallback( boost::bind(&SeptentrioNode::attitudeCovEulerCallback, this, _1, _2) );

  std::string port_, sep_port_, output_rate_, range_output_rate_;
  int baud_;
  std::string name_ = ros::this_node::getName();
  nh.getParam(name_+"/port", port_);
  nh.getParam(name_+"/baudrate", baud_);
  nh.getParam(name_+"/septentrio_port", sep_port_);
  nh.getParam(name_+"/output_rate"      , output_rate_);
  nh.getParam(name_+"/range_output_rate", range_output_rate_);
  ROS_INFO_STREAM("Septentrio:\n\tPort: " << port_ << 
                  "\n\tBaud: " << baud_ <<
                  "\n\tSeptentrio Port: " << sep_port_ << "\n");
  try {
    gps.connect(port_, baud_, sep_port_);
    gps.setOutputRate(output_rate_);
    gps.setRangeOutputRate(range_output_rate_);
  } catch (std::exception e) {
    std::cout << "Error in connecting" << std::endl;
  }
  if (!gps.isConnected()) {
    std::cout << "Did not connect." << std::endl;
  } else {
    ROS_INFO_STREAM("Septentrio connected!\n");
  }
  
  std::vector<std::string> logs_;
  nh.getParam(name_+"/logs", logs_);
  std::cout << "Septentrio requesting logs: ";
  for (std::vector<std::string>::iterator it=logs_.begin(); it!=logs_.end(); ++it) {
    if (!gps.requestLog(*it)) {
      ROS_WARN_STREAM("Couldn't request desired logs: " << *it);
    }
    std::cout << *it << ", ";
  }
  std::cout << std::endl;
}

double SeptentrioNode::getTimeCallback()
{
  return ros::Time::now().toSec();
}

void SeptentrioNode::receiverTimeCallback(ReceiverTime& data, double& read_stamp)
{
}


void SeptentrioNode::pvtCartestianCallback(PvtCartesian& data, double& read_stamp)
{
  // pvt_cartesian = data;
  septentrio::PvtCartesianMsg msg;
  msg.GPS_ms      = data.GPS_ms;
  msg.weekNumber  = data.weekNumber;
  msg.numSat      = data.numSat;
  msg.Error       = data.Error;
  msg.Mode        = data.Mode;
  msg.System      = data.System;
  msg.Info        = data.Info;
  msg.SBASprn     = data.SBASprn;
  msg.x_position  = data.x_position;
  msg.y_position  = data.y_position;
  msg.z_position  = data.z_position;
  msg.x_velocity  = data.x_velocity;
  msg.y_velocity  = data.y_velocity;
  msg.z_velocity  = data.z_velocity;
  msg.RxClkBias   = data.RxClkBias;
  msg.RxClkDrift  = data.RxClkDrift;
  msg.MeanCorrAge = data.MeanCorrAge;
  msg.ReferenceID = data.ReferenceID;
  msg.course      = data.course;
  pvt_cartesian_pub.publish(msg);
}


void SeptentrioNode::posCovCartesianCallback(PosCovCartesian& data, double& read_stamp)
{
  septentrio::PosCovCartesianMsg msg;
  msg.GPS_ms     = data.GPS_ms;    
  msg.weekNumber = data.weekNumber;        
  msg.reserved   = data.reserved;      
  msg.error      = data.error;   
  msg.Cov_xx     = data.Cov_xx;    
  msg.Cov_yy     = data.Cov_yy;    
  msg.Cov_zz     = data.Cov_zz;    
  msg.Cov_bb     = data.Cov_bb;    
  msg.Cov_xy     = data.Cov_xy;    
  msg.Cov_xz     = data.Cov_xz;    
  msg.Cov_xb     = data.Cov_xb;    
  msg.Cov_yz     = data.Cov_yz;    
  msg.Cov_yb     = data.Cov_yb;    
  msg.Cov_zb     = data.Cov_zb;
  pos_cov_cartesian_pub.publish(msg);
}


void SeptentrioNode::velCovCaresianCallback(VelCovCartesian& data, double& read_stamp)
{
  septentrio::VelCovCartesianMsg msg;
  msg.GPS_ms     = data.GPS_ms;
  msg.weekNumber = data.weekNumber;
  msg.reserved   = data.reserved;
  msg.error      = data.error;
  msg.Cov_VxVx   = data.Cov_VxVx;
  msg.Cov_VyVy   = data.Cov_VyVy;
  msg.Cov_VzVz   = data.Cov_VzVz;
  msg.Cov_dd     = data.Cov_dd;
  msg.Cov_VxVy   = data.Cov_VxVy;
  msg.Cov_VxVz   = data.Cov_VxVz;
  msg.Cov_Vxd    = data.Cov_Vxd;
  msg.Cov_VyVz   = data.Cov_VyVz;
  msg.Cov_Vyd    = data.Cov_Vyd;
  msg.Cov_Vzd    = data.Cov_Vzd;
  vel_cov_cartesian_pub.publish(msg);
}


void SeptentrioNode::attitudeEulerCallback(AttitudeEuler& data, double& read_stamp)
{
  septentrio::AttitudeEulerMsg msg;
  msg.GPS_ms     = data.GPS_ms;
  msg.weekNumber = data.weekNumber;
  msg.numSat     = data.numSat;
  msg.error      = data.error;
  msg.mode       = data.mode;
  msg.reserved   = data.reserved;
  msg.heading    = data.heading;
  msg.pitch      = data.pitch;
  msg.roll       = data.roll;
  msg.x_omega    = data.x_omega;
  msg.y_omega    = data.y_omega;
  msg.z_omega    = data.z_omega;
  attitude_euler_pub.publish(msg);
}


void SeptentrioNode::attitudeCovEulerCallback(AttitudeCovEuler& data, double& read_stamp)
{
  septentrio::AttitudeCovEulerMsg msg;
  msg.GPS_ms      = data.GPS_ms;
  msg.weekNumber  = data.weekNumber;
  msg.reserved    = data.reserved;
  msg.error       = data.error;
  msg.var_heading = data.var_heading;
  msg.var_pitch   = data.var_pitch;
  msg.var_roll    = data.var_roll;
  attitude_cov_euler_pub.publish(msg);
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "septentrio");
  SeptentrioNode sn;
  std::cout << "SeptentrioNode created\n";
  ros::spin();
  return 0;
}