#ifndef _MUL_PILOT_
#define _MUL_PILOT_

#include <rcomponent/rcomponent.h>

// Insert here general includes:
#include <math.h>

// Insert here msg and srv includes:
#include <std_msgs/String.h>
#include <robotnik_msgs/StringStamped.h>
#include <robotnik_msgs/State.h>

#include <std_srvs/Trigger.h>

class MulPilot : public rcomponent::RComponent
{
public:
  MulPilot(ros::NodeHandle h);
  ~MulPilot() override;

protected:
  /*** RComponent stuff ***/

  //! Setups all the ROS' stuff
  int rosSetup() override;
  //! Shutdowns all the ROS' stuff
  int rosShutdown() override;
  //! Reads data a publish several info into different topics
  void rosPublish() override;
  //! Reads params from params server
  void rosReadParams() override;
  //! Actions performed on init state
  void initState() override;
  //! Actions performed on standby state
  void standbyState() override;
  //! Actions performed on ready state
  void readyState() override;
  //! Actions performed on the emergency state
  void emergencyState() override;
  //! Actions performed on Failure state
  void failureState() override;

  /* RComponent stuff !*/

  /* ROS Stuff */

  // Publishers

  //! To publish the basic information
  ros::Publisher status_pub_;
  ros::Publisher status_stamped_pub_;

  ros::Publisher robot_status_pub_;
  string robot_status_pub_name_;

  ros::Publisher robot_result_pub_;
  string robot_result_pub_name_;

  ros::Publisher interface_pub_;
  string interface_pub_name_;

  //! Subscribers
  ros::Subscriber proxsensor_status_sub_;
  string proxsensor_status_sub_name_;

  ros::Subscriber iot_rtls_positions_sub_;
  string iot_rtls_positions_sub_name_;

  //! Services
  // ros::ServiceServer example_server_;

  //! Callbacks
  // Subscription Callbacks
  void proxsensorStatusSubCb(const robotnik_msgs::State::ConstPtr &msg);
  void iotRtlsPositionsSubCb(const robotnik_msgs::State::ConstPtr &msg);

  // Service Callbacks
  // bool exampleServerCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  /* ROS stuff !*/

  /* MulPilot stuff */

  std_msgs::String status_;

  /* MulPilot stuff !*/
};

#endif // _MUL_PILOT_
