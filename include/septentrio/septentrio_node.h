#include <septentrio/septentrio.h>
#include <ros/ros.h>
#include <septentrio/PvtCartesianMsg.h>
#include <septentrio/PosCovCartesianMsg.h>
#include <septentrio/VelCovCartesianMsg.h>
#include <septentrio/AttitudeEulerMsg.h>
#include <septentrio/AttitudeCovEulerMsg.h>


class SeptentrioNode {

  public:

    SeptentrioNode();  

  private:

    // Callbacks to receive from the Septentrio
    double getTimeCallback();
    void receiverTimeCallback(ReceiverTime&, double&);
    void pvtCartestianCallback(PvtCartesian&, double&);
    void posCovCartesianCallback(PosCovCartesian&, double&);
    void velCovCaresianCallback(VelCovCartesian&, double&);
    void attitudeEulerCallback(AttitudeEuler&, double&);
    void attitudeCovEulerCallback(AttitudeCovEuler&, double&);

    ros::NodeHandle nh;
    ros::Publisher receiver_time_pub;
    ros::Publisher pvt_cartesian_pub;
    ros::Publisher pos_cov_cartesian_pub;
    ros::Publisher vel_cov_cartesian_pub;
    ros::Publisher attitude_euler_pub;
    ros::Publisher attitude_cov_euler_pub;

    // driver object
    Septentrio gps;

};