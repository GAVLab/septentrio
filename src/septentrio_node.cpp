/*
 * ROS Node that wraps the Septentrio driver
 */
#include <septentrio/septentrio_node.h>



SeptentrioNode::SeptentrioNode()
{
  // receiver_time_pub =       nh.advertise<septentrio::>
  pvt_cartesian_pub =       nh.advertise<septentrio::PvtCartesianMsg>("~pvt_cartesian", 1000);
  // pos_cov_cartesian_pub =   nh.advertise<septentrio::>
  // vel_cov_cartesian_pub =   nh.advertise<septentrio::>
  // attitude_euler_pub =      nh.advertise<septentrio::>
  // attitude_cov_euler_pub =  nh.advertise<septentrio::>
}

double SeptentrioNode::getTimeCallback()
{
  return ros::Time::now().toSec();
}

void SeptentrioNode::receiverTimeCallback(ReceiverTime& data, double& read_stamp)
{
  // receiver_time = data;
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
  // pos_cov_cartesian = data;
}


void SeptentrioNode::velCovCaresianCallback(VelCovCartesian& data, double& read_stamp)
{
  // vel_cov_cartesian = data;
}


void SeptentrioNode::attitudeEulerCallback(AttitudeEuler& data, double& read_stamp)
{
  // attitude_euler = data;
}


void SeptentrioNode::attitudeCovEulerCallback(AttitudeCovEuler& data, double& read_stamp)
{
  // attitude_cov_euler = data;
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "septentrio");
  SeptentrioNode sn;
  ros::spin();
  return 0;
}