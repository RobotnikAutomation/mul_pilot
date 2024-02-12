#ifndef _MUL_PILOT_
#define _MUL_PILOT_

#include <rcomponent/rcomponent.h>

// General includes
#include <actionlib/client/simple_action_client.h>
#include <math.h>

// Msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <odin_msgs/HMIBase.h>
#include <odin_msgs/ProxSensor.h>
#include <odin_msgs/RobotStatus.h>
#include <odin_msgs/RobotTask.h>
#include <odin_msgs/RTLSBase.h>
#include <odin_msgs/SmartboxStatus.h>
#include <robotnik_msgs/BatteryStatus.h>
#include <robotnik_msgs/ElevatorStatus.h>
#include <robotnik_msgs/StringStamped.h>
#include <std_msgs/String.h>

// Srvs
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

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

  ros::Publisher state_machine_state_pub_;
  std_msgs::String current_state_data_;
 
  //! Subscribers
  ros::Subscriber proxsensor_sub_;
  string proxsensor_sub_name_;

  ros::Subscriber smartbox_sub_;
  string smartbox_sub_name_;

  ros::Subscriber rtls_sub_;
  string rtls_sub_name_;

  ros::Subscriber hmi_sub_;
  string hmi_sub_name_;

  ros::Subscriber elevator_sub_;
  string elevator_sub_name_;

  ros::Subscriber battery_sub_;
  string battery_sub_name_;

  ros::Subscriber pose_sub_;
  string pose_sub_name_;

  //! Services Servers
  ros::ServiceServer mission_received_srv_;
  ros::ServiceServer elevator_down_srv_;
  ros::ServiceServer rack_position_received_srv_;
  ros::ServiceServer goal_calculated_srv_;
  ros::ServiceServer arrived_at_rack_srv_;
  ros::ServiceServer rack_picked_srv_;
  ros::ServiceServer arrived_at_poi_srv_;
  ros::ServiceServer go_to_lab_srv_;
  ros::ServiceServer arrived_at_lab_srv_;
  ros::ServiceServer release_rack_srv_;
  ros::ServiceServer rack_homed_srv_;
  ros::ServiceServer rack_placed_srv_;
  ros::ServiceServer rack_released_srv_;
  ros::ServiceServer arrived_at_home_srv_;

  //! Services Clients
  // ros::ServiceClient out_of_battery_client_;
  // ros::ServiceClient rack_position_received_client_;
  // ros::ServiceClient goal_calculated_client_;
  // ros::ServiceClient arrived_at_rack_client_;
  // ros::ServiceClient rack_picked_client_;
  // ros::ServiceClient arrived_at_home_client_;

  //! Action Clients
  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_;
  move_base_msgs::MoveBaseGoal move_base_goal_;

  std::shared_ptr<actionlib::SimpleActionClient<robot_simple_command_manager_msgs::RobotSimpleCommandAction>> command_sequencer_ac_;
  robot_simple_command_manager_msgs::RobotSimpleCommandGoal command_sequencer_goal_;

  //! Callbacks
  //! Subscription Callbacks
  void proxsensorSubCb(const odin_msgs::ProxSensor::ConstPtr &msg);
  void smartboxSubCb(const odin_msgs::SmartboxStatus::ConstPtr &msg);
  void rtlsSubCb(const odin_msgs::RTLSBase::ConstPtr &msg);
  void hmiSubCb(const odin_msgs::HMIBase::ConstPtr &msg);
  void elevatorSubCb(const robotnik_msgs::ElevatorStatus::ConstPtr &msg);
  void batterySubCb(const robotnik_msgs::BatteryStatus::ConstPtr &msg);
  void poseSubCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  //! Service Callbacks
  // WAITING_FOR_MISSION --> CHECKING_ELEVATOR
  bool missionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // CHECKING_ELEVATOR --> GETTING_RACK_POSITION or NAVIGATING_TO_POI
  bool elevatorDownServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  // GETTING_RACK_POSITION --> CALCULATING_GOAL
  bool rackPositionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // CALCULATING_GOAL --> NAVIGATING_TO_RACK
  bool goalCalculatedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // NAVIGATING_TO_RACK --> PICKING_RACK
  bool arrivedAtRackServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // PICKING_RACK --> NAVIGATING_TO_POI
  bool rackPickedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // NAVIGATING_TO_POI --> WAITING_IN_POI
  bool arrivedAtPoiServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // WAITING_IN_POI --> NAVIGATING_TO_LAB
  bool goToLabServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // NAVIGATING_TO_LAB --> WAITING_IN_LAB
  bool arrivedAtLabServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // WAITING_IN_LAB --> HOMING_RACK or RELEASING_RACK
  bool releaseRackServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  // HOMING_RACK --> PLACING_RACK
  bool rackHomedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // PLACING_RACK --> NAVIGATING_TO_HOME
  bool rackPlacedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // RELEASING_RACK --> NAVIGATING_TO_HOME
  bool rackReleasedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // NAVIGATING_TO_HOME --> WAITING_FOR_MISSION
  bool arrivedAtHomeServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  //! Action Callbacks
  void moveBaseResultCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
  void commandSequencerResultCb(const actionlib::SimpleClientGoalState &state, const robot_simple_command_manager_msgs::RobotSimpleCommandResultConstPtr &result);
  /* ROS Stuff !*/

  /*** MulPilot Stuff ***/
  std_msgs::String status_;
  string current_state_;
  string previous_state_;
  float battery_status_{0.0};
  geometry_msgs::PoseWithCovarianceStamped pose_;

  //! State Machine
  void runRobotStateMachine();
  void changeState(const string &next_state, const string &additional_information);

  //! WAITING_FOR_MISSION
  void waitingForMissionState();
  bool mission_received_;
  float poi_x_{0.0};
  float poi_y_{0.0};
  float poi_rot_z_{0.0};
  float poi_rot_w_{1.0};
  float home_x_{0.0};
  float home_y_{0.0};

  //! CHECKING_ELEVATOR
  void checkingElevatorState();

  //! GETTING_RACK_POSITION
  void gettingRackPositionState();
  double rack_x_{0.0};
  double rack_y_{0.0};
  double rack_z_{0.0};
  double x1_{1.59983614424};
  double y1_{1.7909510717};
  double z1_{0.0};
  double x2_{12.614839128};
  double y2_{2.96726033821};
  double z2_{0.0};
  double x_goal_{0.0};
  double y_goal_{0.0};
  double z_goal_{0.0};
  double z_orient_goal{0.0};
  double w_orient_goal{0.0};

  //! CALCULATING_GOAL
  void calculatingGoalState();

  //! NAVIGATING_TO_RACK
  void navigatingToRackState();
  bool navigation_command_sent_;

  //! PICKING_RACK
  void pickingRackState();
  string pick_sequence_;
  bool sequence_sent_;

  //! NAVIGATING_TO_POI
  void navigatingToPoiState();

  //! WAITING_IN_POI
  void waitingInPoiState();
  float lab_pos_x_{0.0};
  float lab_pos_y_{0.0};
  float lab_pos_z_{0.0};
  float lab_ori_x_{0.0};
  float lab_ori_y_{0.0};
  float lab_ori_z_{0.0};
  float lab_ori_w_{0.0};

  //! NAVIGATING_TO_LAB
  void navigatingToLabState();

  //! WAITING_IN_LAB
  void waitingInLabState();

  //! HOMING_RACK
  void homingRackState();

  //! PLACING_RACK
  void placingRackState();
  string place_sequence_;

  //! RELEASING_RACK
  void releasingRackState();
  string release_sequence_;

  //! NAVIGATING_TO_HOME
  void navigatingToHomeState();
  /* MulPilot Stuff !*/
};

#endif // _MUL_PILOT_
