/*
 * ROS Node that wraps the Septentrio driver
 */
#include <septentrio/septentrio_node.h>

  
SeptentrioNode::SeptentrioNode()
{
  std::string name_ = ros::this_node::getName();

  // receiver_time_pub =       nh.advertise<septentrio::>
  pvt_cartesian_pub =       nh.advertise<septentrio::PvtCartesianMsg>(    name_+"/pvt_cartesian",      1000);
  pos_cov_cartesian_pub =   nh.advertise<septentrio::PosCovCartesianMsg>( name_+"/pos_cov_cartesian",  1000);
  vel_cov_cartesian_pub =   nh.advertise<septentrio::VelCovCartesianMsg>( name_+"/vel_cov_cartesian",  1000);
  attitude_euler_pub =      nh.advertise<septentrio::AttitudeEulerMsg>(   name_+"/attitude_euler",     1000);
  attitude_cov_euler_pub =  nh.advertise<septentrio::AttitudeCovEulerMsg>(name_+"/attitude_cov_euler", 1000);
  range_pub_= nh.advertise<septentrio::RangeMsg>(name_+"/raw_meas",1);
  odom_pub_ =nh.advertise<nav_msgs::Odometry>(name_+"/odom", 1);


  gps.setTimeHandler(boost::bind(&SeptentrioNode::getTimeHandler, this));

  // setReceiverTimeCallback
  gps.setReceiverTimeCallback(     boost::bind(&SeptentrioNode::receiverTimeCallback,    this, _1, _2) );
  gps.setPvtCartesianCallback(     boost::bind(&SeptentrioNode::pvtCartestianCallback,    this, _1, _2) );
  gps.setPosCovCartesianCallback(  boost::bind(&SeptentrioNode::posCovCartesianCallback,  this, _1, _2) );
  gps.setVelCovCaresianCallback(   boost::bind(&SeptentrioNode::velCovCaresianCallback,   this, _1, _2) );
  gps.setAttitudeEulerCallback(    boost::bind(&SeptentrioNode::attitudeEulerCallback,    this, _1, _2) );
  gps.setAttitudeCovEulerCallback( boost::bind(&SeptentrioNode::attitudeCovEulerCallback, this, _1, _2) );
  gps.setOdometryCallback(boost::bind(&SeptentrioNode::OdometryCallback,this, _1,_2));
  gps.setRangeCallback(boost::bind(&SeptentrioNode::RangeCallback,this, _1,_2));

  // get params, rosparams
  std::string port_, sep_port_, output_rate_, range_output_rate_;
  int baud_;
  std::map<std::string,double> antenna_1_loc_, antenna_2_loc_;
  nh.getParam(name_+"/port", port_);
  nh.getParam(name_+"/baudrate", baud_);
  nh.getParam(name_+"/septentrio_port", sep_port_);
  nh.getParam(name_+"/output_rate"      , output_rate_);
  nh.getParam(name_+"/range_output_rate", range_output_rate_);
  nh.getParam(name_+"/antenna_location/1", antenna_1_loc_);
  nh.getParam(name_+"/antenna_location/2", antenna_2_loc_);
  ROS_INFO_STREAM("Septentrio:" <<
                  "\n\tPort: " << port_ << 
                  "\n\tBaud: " << baud_ <<
                  "\n\tSeptentrio Port: " << sep_port_ <<
                  "\n\tOutput Rate: " << output_rate_ <<
                  "\n\tRange Output Rate: " << range_output_rate_ <<
                  "\n\tAntenna 1 location:" << 
                  "\n\t\tx: " << antenna_1_loc_["x"] << 
                  "\n\t\ty: " << antenna_1_loc_["y"] <<
                  "\n\t\tz: " << antenna_1_loc_["z"] <<
                  "\n\tAntenna 2 location: " <<
                  "\n\t\tx: " << antenna_2_loc_["x"] <<
                  "\n\t\ty: " << antenna_2_loc_["y"] <<
                  "\n\t\tz: " << antenna_2_loc_["z"]
                  );
  // hook it up to the septentrio
  try {
    gps.connect(port_, baud_, sep_port_);
  } catch (std::exception e) {
    ROS_ERROR_STREAM("Error in connecting: " << e.what());
  }
  if (!gps.isConnected()) {
    ROS_ERROR_STREAM("Did not connect.");
  } else {
    ROS_INFO_STREAM("Septentrio connected!\n");
  }
  // configure the already-connected receiver
  gps.setOutputRate(output_rate_);
   boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  gps.setRangeOutputRate(range_output_rate_);
   boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  gps.setAntennaLocations(1, antenna_1_loc_["x"], antenna_1_loc_["y"], antenna_1_loc_["z"]);
  gps.setAntennaLocations(2, antenna_2_loc_["x"], antenna_2_loc_["y"], antenna_2_loc_["z"]);
  ROS_INFO_STREAM("Septentrio configuration done.");
  // request logs
  try {
    std::vector<std::string> logs_;
    nh.getParam(name_+"/logs", logs_);
    std::cout << "Septentrio requesting logs: ";
    for (std::vector<std::string>::iterator it=logs_.begin(); it!=logs_.end(); ++it) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(10)); 
      if (!gps.requestLog(*it)) {
        ROS_WARN_STREAM("Couldn't request desired logs: " << *it);
      }
      std::cout << *it << ", ";
    }
    std::cout << std::endl;
  } catch (std::exception e) {
    ROS_ERROR_STREAM("Couldn't configure logs: " << e.what());
  }
  // check for RTK request
  // std::map<std::string,std::string> rtk_;
  // ROS_INFO_STREAM("\n\nasking for param: "<<name_+"/rtk" );
  std::string rtk_port_, rtk_format_;
  bool request_rtk=false;
  int rtk_baud_;
  nh.getParam(name_+"/rtk_port", rtk_port_);
  nh.getParam(name_+"/rtk_baud", rtk_baud_);
  nh.getParam(name_+"/rtk_format", rtk_format_);
  nh.getParam(name_+"/request_rtk", request_rtk);

  gps.SetElevationMask(15);
  gps.SetAttitudeMode();
  
  if (request_rtk && rtk_format_.length() > 0){
    ROS_INFO_STREAM("\nNode is requesting RTK\n");
    gps.setRTK(rtk_port_, rtk_baud_, rtk_format_);
  } else {
    gps.setRTK();
    ROS_INFO_STREAM("\n\nRequest standard position\n\n");
  }
}

double SeptentrioNode::getTimeHandler()
{
  return ros::Time::now().toSec();
}

void SeptentrioNode::receiverTimeCallback(ReceiverTime& data, double& read_stamp)
{

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SteptentrioNode::RangeCallback  [Private]  --- publishes standard odometry message
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SeptentrioNode::RangeCallback(RangeData& data, double& read_stamp)
{septentrio::RangeMsg msg;
  msg.header.stamp=ros::Time().fromSec(read_stamp);
  for (int i=0;i<30;i++){

    msg.SVID_1[i]= data.SVID_1[i];
    msg.CACode_1[i]= data.CACode_1[i];
    msg.L1Phase_1[i]= data.L1Phase_1[i];
    msg.L1Doppler_1[i]= data.L1Doppler_1[i];
    msg.CAC2N_1[i]= data.CAC2N_1[i];

    msg.SVID_2[i]= data.SVID_2[i];
    msg.CACode_2[i]= data.CACode_2[i];
    msg.L1Phase_2[i]= data.L1Phase_2[i];
    msg.L1Doppler_2[i]= data.L1Doppler_2[i];
    msg.CAC2N_2[i]= data.CAC2N_2[i];

    msg.SVID_3[i]= data.SVID_3[i];
    msg.CACode_3[i]= data.CACode_3[i];
    msg.L1Phase_3[i]= data.L1Phase_3[i];
    msg.L1Doppler_3[i]= data.L1Doppler_3[i];
    msg.CAC2N_3[i]= data.CAC2N_3[i];
}
  range_pub_.publish(msg);

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SteptentrioNode::OdometryCallback  [Private]  --- publishes standard odometry message
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SeptentrioNode::OdometryCallback(OdometryData& data,double& read_stamp){
  ROS_DEBUG("got odometry message");
	nav_msgs::Odometry msg;
	msg.header.stamp=ros::Time().fromSec(read_stamp);
	msg.header.frame_id="ECEF";
	msg.pose.pose.position.x=data.pvt.x_position;
	msg.pose.pose.position.y=data.pvt.y_position;
	msg.pose.pose.position.z=data.pvt.z_position;
	msg.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(data.att.roll * degrees_to_radians,data.att.pitch * degrees_to_radians,psi2theta(data.att.heading * degrees_to_radians));
	msg.pose.covariance[0]=data.pos_cov.Cov_xx;
	msg.pose.covariance[1]=data.pos_cov.Cov_xy;
	msg.pose.covariance[2]=data.pos_cov.Cov_xz;
	msg.pose.covariance[6]=data.pos_cov.Cov_xy;
	msg.pose.covariance[7]=data.pos_cov.Cov_yy;
	msg.pose.covariance[8]=data.pos_cov.Cov_yz;
	msg.pose.covariance[12]=data.pos_cov.Cov_xz;
	msg.pose.covariance[13]=data.pos_cov.Cov_yz;
	msg.pose.covariance[14]=data.pos_cov.Cov_zz;
	msg.pose.covariance[21]=data.att_cov.var_roll;
	msg.pose.covariance[28]=data.att_cov.var_pitch;
	msg.pose.covariance[35]=data.att_cov.var_heading;

	msg.twist.twist.linear.x=data.pvt.x_velocity;
	msg.twist.twist.linear.y=data.pvt.y_velocity;
	msg.twist.twist.linear.z=data.pvt.z_velocity;
	msg.twist.twist.angular.x=data.att.x_omega;
	msg.twist.twist.angular.y=data.att.y_omega;
	msg.twist.twist.angular.z=data.att.z_omega;
	msg.twist.covariance[0]=data.vel_cov.Cov_VxVx;
	msg.twist.covariance[1]=data.vel_cov.Cov_VxVy;
	msg.twist.covariance[2]=data.vel_cov.Cov_VxVz;
	msg.twist.covariance[6]=data.vel_cov.Cov_VxVy;
	msg.twist.covariance[7]=data.vel_cov.Cov_VyVy;
	msg.twist.covariance[8]=data.vel_cov.Cov_VyVz;
	msg.twist.covariance[12]=data.vel_cov.Cov_VxVz;
	msg.twist.covariance[13]=data.vel_cov.Cov_VyVz;
	msg.twist.covariance[14]=data.vel_cov.Cov_VzVz;
  odom_pub_.publish(msg);

}


void SeptentrioNode::pvtCartestianCallback(PvtCartesian& data, double& read_stamp)
{
  // pvt_cartesian = data;
  septentrio::PvtCartesianMsg msg;
  msg.header.stamp = ros::Time().fromSec(read_stamp);
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
  msg.header.stamp = ros::Time().fromSec(read_stamp);
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

  // TODO: if in RTK mode: check cov to make sure it's as low as it should be

  pos_cov_cartesian_pub.publish(msg);
}


void SeptentrioNode::velCovCaresianCallback(VelCovCartesian& data, double& read_stamp)
{
  septentrio::VelCovCartesianMsg msg;
  msg.header.stamp = ros::Time().fromSec(read_stamp);
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
  msg.header.stamp = ros::Time().fromSec(read_stamp);
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
  msg.header.stamp = ros::Time().fromSec(read_stamp);
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