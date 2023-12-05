#ifndef _MUL_PILOT_
#define _MUL_PILOT_

#include <rcomponent/rcomponent.h>

// Insert here general includes:
#include <actionlib/client/simple_action_client.h>
#include <math.h>

// Msgs
#include <odin_msgs/ProxSensor.h>
#include <odin_msgs/RobotStatus.h>
#include <odin_msgs/RobotTask.h>
#include <odin_msgs/RTLS.h>
#include <odin_msgs/SmartboxStatus.h>
#include <robotnik_msgs/StringStamped.h>
#include <std_msgs/String.h>

// Srvs
#include <std_srvs/Trigger.h>

// Actions
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_simple_command_manager_msgs/RobotSimpleCommandAction.h>

class MulPilot : public rcomponent::RComponent
{
public:
  MulPilot(ros::NodeHandle h);
  ~MulPilot() override;

protected:
  /*** RComponent Stuff ***/

  //! Setups all the ROS' Stuff
  int rosSetup() override;
  //! Shutdowns all the ROS' Stuff
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

  /* RComponent Stuff !*/

  /*** ROS Stuff ***/

  //! Publishers
  ros::Publisher status_pub_;
  ros::Publisher status_stamped_pub_;

  ros::Publisher robot_status_pub_;
  string robot_status_pub_name_;

  ros::Publisher robot_result_pub_;
  string robot_result_pub_name_;

  ros::Publisher interface_pub_;
  string interface_pub_name_;

  //! Subscribers
  ros::Subscriber proxsensor_sub_;
  string proxsensor_sub_name_;

  ros::Subscriber smartbox_sub_;
  string smartbox_sub_name_;

  ros::Subscriber rtls_sub_;
  string rtls_sub_name_;

  //! Services Servers
  ros::ServiceServer out_of_battery_srv_;
  ros::ServiceServer location_received_srv_;
  ros::ServiceServer goal_calculated_srv_;
  ros::ServiceServer arrived_at_rack_srv_;
  ros::ServiceServer rack_picked_srv_;
  ros::ServiceServer arrived_at_home_srv_;

  //! Services Clients
  ros::ServiceClient out_of_battery_client_;
  ros::ServiceClient location_received_client_;
  ros::ServiceClient goal_calculated_client_;
  ros::ServiceClient arrived_at_rack_client_;
  ros::ServiceClient rack_picked_client_;
  ros::ServiceClient arrived_at_home_client_;

  //! Action Clients
  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_;
  move_base_msgs::MoveBaseGoal move_base_goal_;

  std::shared_ptr<actionlib::SimpleActionClient<robot_simple_command_manager_msgs::RobotSimpleCommandAction>> command_sequencer_ac_;
  robot_simple_command_manager_msgs::RobotSimpleCommandGoal command_sequencer_goal_;

  //! Callbacks
  //! Subscription Callbacks
  void proxsensorSubCb(const odin_msgs::ProxSensor::ConstPtr &msg);
  void smartboxSubCb(const odin_msgs::SmartboxStatus::ConstPtr &msg);
  void rtlsSubCb(const odin_msgs::RTLS::ConstPtr &msg);

  //! Service Callbacks
  bool outOfBatteryServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool locationReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool goalCalculatedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool arrivedAtRackServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool rackPickedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  bool arrivedAtHomeServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  //! Action Callbacks
  void moveBaseResultCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
  void commandSequencerResultCb(const actionlib::SimpleClientGoalState &state, const robot_simple_command_manager_msgs::RobotSimpleCommandResultConstPtr &result);

  /* ROS Stuff !*/

  /*** MulPilot Stuff ***/

  std_msgs::String status_;
  string current_state_;
  string previous_state_;

  //! State Machine
  void runRobotStateMachine();
  void changeState(const string &next_state, const string &additional_information);

  //! WAITING_FOR_MISSION
  void waitingForMissionState();
  bool mission_received_;

  //! GETTING_LOCATION
  void gettingLocationState();
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double x1_{0.0};
  double y1_{0.0};
  double z1_{0.0};
  double x2_{0.0};
  double y2_{0.0};
  double z2_{0.0};
  double x_goal_{0.0};
  double y_goal_{0.0};
  double z_goal_{0.0};

  //! CALCULATING_GOAL
  void calculatingGoalState();

  //! NAVIGATING_TO_RACK
  void navigatingToRackState();
  bool navigation_command_sent_;

  //! PICKING_RACK
  void pickingRackState();
  string pick_sequence_;
  bool pick_command_sent_;

  //! NAVIGATING_TO_HOME
  void navigatingToHomeState();

  /* MulPilot Stuff !*/
};

#endif // _MUL_PILOT_
