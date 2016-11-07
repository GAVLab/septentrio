#include <septentrio/septentrio.h>

#include <ros/ros.h>
#include <septentrio/PvtCartesianMsg.h>
#include <septentrio/PosCovCartesianMsg.h>
#include <septentrio/VelCovCartesianMsg.h>
#include <septentrio/AttitudeEulerMsg.h>
#include <septentrio/AttitudeCovEulerMsg.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



class SeptentrioNode {

  public:

    SeptentrioNode();  

  private:

    /*
     *  Allows ROS time to be tagged to each message as soon as it comes across
     *  the serial port.
     */
    double getTimeHandler();

    //////////////////////////////////////////////
    // Callbacks to receive from the Septentrio //
    //////////////////////////////////////////////

    void receiverTimeCallback(ReceiverTime&, double&);
    void pvtCartestianCallback(PvtCartesian&, double&);
    void posCovCartesianCallback(PosCovCartesian&, double&);
    void velCovCaresianCallback(VelCovCartesian&, double&);
    void attitudeEulerCallback(AttitudeEuler&, double&);
    void attitudeCovEulerCallback(AttitudeCovEuler&, double&);
    void OdometryCallback(OdometryData& ,double&);

  inline double psi2theta(double psi)
  {
    return M_PI / 2 - psi;
  }

    ////////////////////
    // ROS attributes // 
    ////////////////////

    ros::NodeHandle nh;
    ros::Publisher receiver_time_pub;
    ros::Publisher pvt_cartesian_pub;
    ros::Publisher pos_cov_cartesian_pub;
    ros::Publisher vel_cov_cartesian_pub;
    ros::Publisher attitude_euler_pub;
    ros::Publisher attitude_cov_euler_pub;
    ros::Publisher odom_pub_;

    const double degrees_to_radians = M_PI / 180.0;

    // driver object
    Septentrio gps;
    

};