#include <mul_pilot/mul_pilot.h>

MulPilot::MulPilot(ros::NodeHandle h) : rcomponent::RComponent(h)
{
  component_name.assign(pnh_.getNamespace());
  rosReadParams();
}

MulPilot::~MulPilot()
{
}

void MulPilot::rosReadParams()
{
  bool required = true;
  bool not_required = false;

  readParam(pnh_, "desired_freq", desired_freq_, 10.0, not_required);
  readParam(pnh_, "robot_status_pub", robot_status_pub_name_, "/mul_pilot/robot_status", required);
  readParam(pnh_, "robot_result_pub", robot_result_pub_name_, "/mul_pilot/robot_result", required);
  readParam(pnh_, "proxsensor_sub", proxsensor_sub_name_, "/mul_pilot/proxsensor", required);
  readParam(pnh_, "rtls_sub", rtls_sub_name_, "/mul_pilot/rtls", required);
  readParam(pnh_, "smartbox_sub", smartbox_sub_name_, "/mul_pilot/smartbox", required);
  readParam(pnh_, "hmi_sub", hmi_sub_name_, "/mul_pilot/hmi", required);
  readParam(pnh_, "elevator_sub", elevator_sub_name_, "/robot/robotnik_base_control/elevator_status", required);
  readParam(pnh_, "pick_sequence", pick_sequence_, "TEST_ALLINEAMENTO_RUOTE", required);
  readParam(pnh_, "place_sequence", place_sequence_, "PLACE_SEQUENCE", required);
  readParam(pnh_, "release_sequence", release_sequence_, "RELEASE_AND_HOME", required);
}

int MulPilot::rosSetup()
{
  RComponent::rosSetup();

  bool required = true;
  bool not_required = false;

  /*** ROS Stuff ***/
  //! Publishers
  status_pub_ = pnh_.advertise<std_msgs::String>("status", 10);
  status_stamped_pub_ = pnh_.advertise<robotnik_msgs::StringStamped>("status_stamped", 10);
  robot_status_pub_ = pnh_.advertise<odin_msgs::RobotStatus>(robot_status_pub_name_, 10);
  robot_result_pub_ = pnh_.advertise<odin_msgs::RobotTask>(robot_result_pub_name_, 10);

  //! Subscribers
  proxsensor_sub_ = nh_.subscribe<odin_msgs::ProxSensor>(proxsensor_sub_name_, 10, &MulPilot::proxsensorSubCb, this);
  addTopicsHealth(&proxsensor_sub_, proxsensor_sub_name_, 50.0, not_required);
  smartbox_sub_ = nh_.subscribe<odin_msgs::SmartboxStatus>(smartbox_sub_name_, 10, &MulPilot::smartboxSubCb, this);
  addTopicsHealth(&smartbox_sub_, smartbox_sub_name_, 50.0, not_required);
  rtls_sub_ = nh_.subscribe<odin_msgs::RTLSBase>(rtls_sub_name_, 10, &MulPilot::rtlsSubCb, this);
  addTopicsHealth(&rtls_sub_, rtls_sub_name_, 50.0, not_required);
  hmi_sub_ = nh_.subscribe<odin_msgs::HMIBase>(hmi_sub_name_, 10, &MulPilot::hmiSubCb, this);
  addTopicsHealth(&hmi_sub_, hmi_sub_name_, 50.0, not_required);
  elevator_sub_ = nh_.subscribe<robotnik_msgs::ElevatorStatus>(elevator_sub_name_, 10, &MulPilot::elevatorSubCb, this);
  addTopicsHealth(&elevator_sub_, elevator_sub_name_, 50.0, not_required);

  //! Service Servers
  mission_received_srv_ = pnh_.advertiseService("/mul_pilot/mission_received", &MulPilot::missionReceivedServiceCb, this);
  elevator_down_srv_ = pnh_.advertiseService("/mul_pilot/elevator_down", &MulPilot::elevatorDownServiceCb, this);
  rack_position_received_srv_ = pnh_.advertiseService("/mul_pilot/rack_position_received", &MulPilot::rackPositionReceivedServiceCb, this);
  goal_calculated_srv_ = pnh_.advertiseService("/mul_pilot/goal_calculated", &MulPilot::goalCalculatedServiceCb, this);
  arrived_at_rack_srv_ = pnh_.advertiseService("/mul_pilot/arrived_at_rack", &MulPilot::arrivedAtRackServiceCb, this);
  rack_picked_srv_ = pnh_.advertiseService("/mul_pilot/rack_picked", &MulPilot::rackPickedServiceCb, this);
  arrived_at_poi_srv_ = pnh_.advertiseService("/mul_pilot/arrived_at_poi", &MulPilot::arrivedAtPoiServiceCb, this);
  go_to_lab_srv_ = pnh_.advertiseService("/mul_pilot/go_to_lab", &MulPilot::goToLabServiceCb, this);
  arrived_at_lab_srv_ = pnh_.advertiseService("/mul_pilot/arrived_at_lab", &MulPilot::arrivedAtLabServiceCb, this);
  release_rack_srv_ = pnh_.advertiseService("/mul_pilot/release_rack", &MulPilot::releaseRackServiceCb, this);
  rack_homed_srv_ = pnh_.advertiseService("/mul_pilot/rack_homed", &MulPilot::rackHomedServiceCb, this);
  rack_placed_srv_ = pnh_.advertiseService("/mul_pilot/rack_placed", &MulPilot::rackPlacedServiceCb, this);
  rack_released_srv_ = pnh_.advertiseService("/mul_pilot/rack_released", &MulPilot::rackReleasedServiceCb, this);
  arrived_at_home_srv_ = pnh_.advertiseService("/mul_pilot/arrived_at_home", &MulPilot::arrivedAtHomeServiceCb, this);

  //! Service Clients
  // rack_position_received_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/rack_position_received");
  // goal_calculated_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/goal_calculated");
  // arrived_at_rack_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/arrived_at_rack");
  // rack_picked_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/rack_picked");
  // arrived_at_home_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/arrived_at_home");

  //! Action Clients
  move_base_ac_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(pnh_, "/robot/move_base", true);
  command_sequencer_ac_ = std::make_shared<actionlib::SimpleActionClient<robot_simple_command_manager_msgs::RobotSimpleCommandAction>>(pnh_, "/robot/command_sequencer/action", true);
  /* ROS Stuff !*/

  return rcomponent::OK;
}

int MulPilot::rosShutdown()
{
  return RComponent::rosShutdown();
}

void MulPilot::rosPublish()
{
  RComponent::rosPublish();

  if (getState() == robotnik_msgs::State::READY_STATE)
  {
    robotnik_msgs::StringStamped status_stamped;

    status_pub_.publish(status_);

    status_stamped.header.stamp = ros::Time::now();
    status_stamped.string = status_.data;
    status_stamped_pub_.publish(status_stamped);
  }
}

void MulPilot::initState()
{
  RComponent::initState();

  current_state_ = "WAITING_FOR_MISSION";
  previous_state_ = "";

  navigation_command_sent_ = false;
  sequence_sent_ = false;

  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void MulPilot::standbyState()
{
  if (checkTopicsHealth() == false)
  {
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
  }
  else
  {
    switchToState(robotnik_msgs::State::READY_STATE);
  }
}

void MulPilot::readyState()
{
  if (checkTopicsHealth() == false)
  {
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
  }

  runRobotStateMachine();
}

void MulPilot::emergencyState()
{
  if (checkTopicsHealth() == true)
  {
    switchToState(robotnik_msgs::State::STANDBY_STATE);
  }
}

void MulPilot::failureState()
{
  RComponent::failureState();
}

/*** State Machine ***/
void MulPilot::runRobotStateMachine()
{
  if (current_state_ == "WAITING_FOR_MISSION")
  {
    waitingForMissionState();
  }
  else if (current_state_ == "CHECKING_ELEVATOR")
  {
    checkingElevatorState();
  }
  else if (current_state_ == "GETTING_RACK_POSITION")
  {
    gettingRackPositionState();
  }
  else if (current_state_ == "CALCULATING_GOAL")
  {
    calculatingGoalState();
  }
  else if (current_state_ == "NAVIGATING_TO_RACK")
  {
    navigatingToRackState();
  }
  else if (current_state_ == "PICKING_RACK")
  {
    pickingRackState();
  }
  else if (current_state_ == "NAVIGATING_TO_POI")
  {
    navigatingToPoiState();
  }
  else if (current_state_ == "WAITING_IN_POI")
  {
    waitingInPoiState();
  }
  else if (current_state_ == "NAVIGATING_TO_LAB")
  {
    navigatingToLabState();
  }
  else if (current_state_ == "WAITING_IN_LAB")
  {
    waitingInLabState();
  }
  else if (current_state_ == "HOMING_RACK")
  {
    homingRackState();
  }
  else if (current_state_ == "PLACING_RACK")
  {
    placingRackState();
  }
  else if (current_state_ == "RELEASING_RACK")
  {
    releasingRackState();
  }
  else if (current_state_ == "NAVIGATING_TO_HOME")
  {
    navigatingToHomeState();
  }
  else
  {
    ROS_WARN("The current state is unknown!");
  }
}

void MulPilot::changeState(const string &next_state, const string &additional_information)
{
  RCOMPONENT_WARN_STREAM(additional_information);
  RCOMPONENT_WARN_STREAM(current_state_ << " --> " << next_state);

  string temp_state = current_state_;
  current_state_ = next_state;
  previous_state_ = temp_state;

  navigation_command_sent_ = false;
  sequence_sent_ = false;
}
/* State Machine !*/

/*** States ***/

//! WAITING_FOR_MISSION
// The RB-1 waits for a mission in its home position and stores POI coordinates when received
void MulPilot::waitingForMissionState()
{
  RCOMPONENT_INFO_STREAM("WAITING_FOR_MISSION");
}

//! CHECKING_ELEVATOR
// The RB-1 checks the elevator position (up or down)
void MulPilot::checkingElevatorState()
{
  RCOMPONENT_INFO_STREAM("CHECKING_ELEVATOR");
}

//! GETTING_RACK_POSITION
// The RB-1 gets the approximate rack position from the RTLS
void MulPilot::gettingRackPositionState()
{
  RCOMPONENT_INFO_STREAM("GETTING_RACK_POSITION");
}

//! CALCULATING_GOAL
// The RB-1 calculates which is the closest rack goal
void MulPilot::calculatingGoalState()
{
  RCOMPONENT_INFO_STREAM("CALCULATING_GOAL");

  // TODO: Get correct coordinates from docking station location
  double distance1;
  distance1 = sqrt(pow(rack_x_ - x1_, 2) + pow(rack_y_ - y1_, 2) + pow(rack_z_ - z1_, 2));

  double distance2;
  distance2 = sqrt(pow(rack_x_ - x2_, 2) + pow(rack_y_ - y2_, 2) + pow(rack_z_ - z2_, 2));

  if (distance1 < distance2)
  {
    x_goal_ = x1_;
    y_goal_ = y1_;
    z_goal_ = z1_;
  }
  else
  {
    x_goal_ = x2_;
    y_goal_ = y2_;
    z_goal_ = z2_;
  }

  std_srvs::TriggerRequest goal_calculated_srv_request;
  std_srvs::TriggerResponse goal_calculated_srv_response;

  if (goalCalculatedServiceCb(goal_calculated_srv_request, goal_calculated_srv_response))
  {
    if (goal_calculated_srv_response.success)
    {
      RCOMPONENT_INFO_STREAM("Successfully switched from CALCULATING_GOAL to NAVIGATING_TO_RACK");
    }
    else
    {
      RCOMPONENT_WARN_STREAM("Failed to switch from CALCULATING_GOAL to NAVIGATING_TO_RACK: " << goal_calculated_srv_response.message.c_str());
    }
  }
  else
  {
    RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/goal_calculated");
  }
}

//! NAVIGATING_TO_RACK
// The RB-1 navigates to the rack
void MulPilot::navigatingToRackState()
{
  RCOMPONENT_INFO_STREAM("NAVIGATING_TO_RACK");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the rack...");

    // TODO: Set the correct frame and coordinates
    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = x_goal_;
    move_base_goal_.target_pose.pose.position.y = y_goal_;
    move_base_goal_.target_pose.pose.position.z = z_goal_;
    move_base_goal_.target_pose.pose.orientation.x = 0.0;
    move_base_goal_.target_pose.pose.orientation.y = 0.0;
    move_base_goal_.target_pose.pose.orientation.z = 0.998652902351;
    move_base_goal_.target_pose.pose.orientation.w = 0.0518881549817;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&MulPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! PICKING_RACK
// The RB-1 picks the rack
void MulPilot::pickingRackState()
{
  RCOMPONENT_INFO_STREAM("PICKING_RACK");
  if (!sequence_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending sequence to pick the rack...");

    // TODO: Set correct command
    command_sequencer_goal_.command.command = pick_sequence_;
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&MulPilot::commandSequencerResultCb, this, _1, _2));

    sequence_sent_ = true;
  }
}

//! NAVIGATING_TO_POI
// The RB-1 navigates to the POI position
void MulPilot::navigatingToPoiState()
{
  RCOMPONENT_INFO_STREAM("NAVIGATING_TO_POI");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the POI...");
    RCOMPONENT_INFO_STREAM("x: " << poi_x_ << ", y: " << poi_y_);

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = poi_x_;
    move_base_goal_.target_pose.pose.position.y = poi_y_;
    move_base_goal_.target_pose.pose.position.z = 0.0;
    move_base_goal_.target_pose.pose.orientation.x = 0.0;
    move_base_goal_.target_pose.pose.orientation.y = 0.0;
    move_base_goal_.target_pose.pose.orientation.z = -0.703345435322;
    move_base_goal_.target_pose.pose.orientation.w = 0.710848224737;

    RCOMPONENT_INFO_STREAM(move_base_goal_);
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&MulPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! WAITING_IN_POI
// The RB-1 waits in the POI position
void MulPilot::waitingInPoiState()
{
  RCOMPONENT_INFO_STREAM("WAITING_IN_POI");
}

//! NAVIGATING_TO_LAB
// The RB-1 navigates to the lab
void MulPilot::navigatingToLabState()
{
  RCOMPONENT_INFO_STREAM("NAVIGATING_TO_LAB");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the lab...");
    RCOMPONENT_INFO_STREAM("x: " << lab_pos_x_ << ", y: " << lab_pos_y_);

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = lab_pos_x_;
    move_base_goal_.target_pose.pose.position.y = lab_pos_y_;
    move_base_goal_.target_pose.pose.position.z = lab_pos_z_;
    move_base_goal_.target_pose.pose.orientation.x = lab_ori_x_;
    move_base_goal_.target_pose.pose.orientation.y = lab_ori_y_;
    move_base_goal_.target_pose.pose.orientation.z = lab_ori_z_;
    move_base_goal_.target_pose.pose.orientation.w = lab_ori_w_;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&MulPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! WAITING_IN_LAB
// The RB-1 waits in the lab
void MulPilot::waitingInLabState()
{
  RCOMPONENT_INFO_STREAM("WAITING_IN_LAB");
}

//! HOMING_RACK
// The RB-1 brings back the rack to its initial position
void MulPilot::homingRackState()
{
  RCOMPONENT_INFO_STREAM("HOMING_RACK");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the rack's home...");

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = x_goal_;
    move_base_goal_.target_pose.pose.position.y = y_goal_;
    move_base_goal_.target_pose.pose.position.z = z_goal_;
    move_base_goal_.target_pose.pose.orientation.x = 0.0;
    move_base_goal_.target_pose.pose.orientation.y = 0.0;
    move_base_goal_.target_pose.pose.orientation.z = -0.707106781;
    move_base_goal_.target_pose.pose.orientation.w = 0.707106781;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&MulPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! PLACING_RACK
// The RB-1 places the rack
void MulPilot::placingRackState()
{
  RCOMPONENT_INFO_STREAM("PLACING_RACK");
  if (!sequence_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending sequence to place the rack...");

    // TODO: Set correct command
    command_sequencer_goal_.command.command = place_sequence_;
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&MulPilot::commandSequencerResultCb, this, _1, _2));

    sequence_sent_ = true;
  }
}

//! RELEASING_RACK
// The RB-1 releases the rack
void MulPilot::releasingRackState()
{
  RCOMPONENT_INFO_STREAM("RELEASING_RACK");
  if (!sequence_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending sequence to release the rack...");

    // TODO: Set correct command
    command_sequencer_goal_.command.command = release_sequence_;
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&MulPilot::commandSequencerResultCb, this, _1, _2));

    sequence_sent_ = true;
  }
}

//! NAVIGATING_TO_HOME
// The RB-1 navigates to its home position
void MulPilot::navigatingToHomeState()
{
  RCOMPONENT_INFO_STREAM("NAVIGATING_TO_HOME");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the home position...");

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = 0.953378100745;
    move_base_goal_.target_pose.pose.position.y = 0.526700783056;
    move_base_goal_.target_pose.pose.position.z = 0.0;
    move_base_goal_.target_pose.pose.orientation.x = 0.0;
    move_base_goal_.target_pose.pose.orientation.y = 0.0;
    move_base_goal_.target_pose.pose.orientation.z = -0.707106781;
    move_base_goal_.target_pose.pose.orientation.w = 0.707106781;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&MulPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}
/* States !*/

/*** Transitions ***/
//! WAITING_FOR_MISSION --> CHECKING_ELEVATOR
bool MulPilot::missionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "WAITING_FOR_MISSION")
  {
    changeState("CHECKING_ELEVATOR", "Mission received!");
    response.success = true;
    response.message = "Mission received! Switching from WAITING_FOR_MISSION to CHECKING_ELEVATOR.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in WAITING_FOR_MISSION state!";
    return true;
  }
  return false;
}

//! CHECKING_ELEVATOR --> GETTING_RACK_POSITION or NAVIGATING_TO_POI
bool MulPilot::elevatorDownServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  if (current_state_ == "CHECKING_ELEVATOR")
  {
    if (request.data)
    {
      changeState("GETTING_RACK_POSITION", "The elevator is down!");
      response.success = true;
      response.message = "The elevator is down! Switching from CHECKING_ELEVATOR to GETTING_RACK_POSITION.";
      return true;
    }
    else
    {
      changeState("NAVIGATING_TO_POI", "The elevator is up!");
      response.success = true;
      response.message = "The elevator is up! Switching from CHECKING_ELEVATOR to NAVIGATING_TO_POI.";
      return true;
    }
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in CHECKING_ELEVATOR state!";
    return true;
  }
  return false;
}

//! GETTING_RACK_POSITION --> CALCULATING_GOAL
bool MulPilot::rackPositionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "GETTING_RACK_POSITION")
  {
    changeState("CALCULATING_GOAL", "Rack position received from RTLS: x=" + std::to_string(rack_x_) + ", y=" + std::to_string(rack_y_) + ", z=" + std::to_string(rack_z_));
    response.success = true;
    response.message = "Location received! Switching from GETTING_RACK_POSITION to CALCULATING_GOAL.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in GETTING_RACK_POSITION state!";
    return true;
  }
  return false;
}

// CALCULATING_GOAL --> NAVIGATING_TO_RACK
bool MulPilot::goalCalculatedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "CALCULATING_GOAL")
  {
    changeState("NAVIGATING_TO_RACK", "Goal calculated!");
    response.success = true;
    response.message = "Goal calculated! Switching from CALCULATING_GOAL to NAVIGATING_TO_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in CALCULATING_GOAL state!";
    return true;
  }
  return false;
}

//! NAVIGATING_TO_RACK --> PICKING_RACK
bool MulPilot::arrivedAtRackServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "NAVIGATING_TO_RACK")
  {
    changeState("PICKING_RACK", "Arrived at rack!");
    response.success = true;
    response.message = "Arrived at rack! Switching from NAVIGATING_TO_RACK to PICKING_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in NAVIGATING_TO_RACK state!";
    return true;
  }
  return false;
}

//! PICKING_RACK --> NAVIGATING_TO_POI
bool MulPilot::rackPickedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "PICKING_RACK")
  {
    changeState("NAVIGATING_TO_POI", "Rack picked!");
    response.success = true;
    response.message = "Rack picked! Switching from PICKING_RACK to NAVIGATING_TO_POI.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in PICKING_RACK state!";
    return true;
  }
  return false;
}

//! NAVIGATING_TO_POI --> WAITING_IN_POI
bool MulPilot::arrivedAtPoiServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "NAVIGATING_TO_POI")
  {
    changeState("WAITING_IN_POI", "Arrived at POI!");
    response.success = true;
    response.message = "Arrived at POI! Switching from NAVIGATING_TO_POI to WAITING_IN_POI.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in NAVIGATING_TO_POI state!";
    return true;
  }
  return false;
}

//! WAITING_IN_POI --> NAVIGATING_TO_LAB
bool MulPilot::goToLabServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "WAITING_IN_POI")
  {
    changeState("NAVIGATING_TO_LAB", "Going to lab!");
    response.success = true;
    response.message = "Going to lab! Switching from WAITING_IN_POI to NAVIGATING_TO_LAB.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in WAITING_IN_POI state!";
    return true;
  }
  return false;
}

//! NAVIGATING_TO_LAB --> WAITING_IN_LAB
bool MulPilot::arrivedAtLabServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "NAVIGATING_TO_LAB")
  {
    changeState("WAITING_IN_LAB", "Arrived at lab!");
    response.success = true;
    response.message = "Arrived at lab! Switching from NAVIGATING_TO_LAB to WAITING_IN_LAB.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in NAVIGATING_TO_LAB state!";
    return true;
  }
  return false;
}

//! WAITING_IN_LAB --> HOMING_RACK or RELEASING_RACK
bool MulPilot::releaseRackServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  if (current_state_ == "WAITING_IN_LAB")
  {
    if (request.data)
    {
      changeState("RELEASING_RACK", "The elevator is down!");
      response.success = true;
      response.message = "The elevator is down! Switching from WAITING_IN_LAB to RELEASING_RACK.";
      return true;
    }
    else
    {
      changeState("HOMING_RACK", "The elevator is up!");
      response.success = true;
      response.message = "The elevator is up! Switching from WAITING_IN_LAB to HOMING_RACK.";
      return true;
    }
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in WAITING_IN_LAB state!";
    return true;
  }
  return false;
}

//! HOMING_RACK --> PLACING_RACK
bool MulPilot::rackHomedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "HOMING_RACK")
  {
    changeState("PLACING_RACK", "Rack homed!");
    response.success = true;
    response.message = "Rack homed! Switching from HOMING_RACK to PLACING_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in HOMING_RACK state!";
    return true;
  }
  return false;
}

//! PLACING_RACK --> NAVIGATING_TO_HOME
bool MulPilot::rackPlacedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "PLACING_RACK")
  {
    changeState("NAVIGATING_TO_HOME", "Rack placed!");
    response.success = true;
    response.message = "Rack placed! Switching from PLACING_RACK to NAVIGATING_TO_HOME.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in PLACING_RACK state!";
    return true;
  }
  return false;
}

//! RELEASING_RACK --> NAVIGATING_TO_HOME
bool MulPilot::rackReleasedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "RELEASING_RACK")
  {
    changeState("NAVIGATING_TO_HOME", "Rack released!");
    response.success = true;
    response.message = "Rack released! Switching from RELEASING_RACK to NAVIGATING_TO_HOME.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in RELEASING_RACK state!";
    return true;
  }
  return false;
}

//! NAVIGATING_TO_HOME --> WAITING_FOR_MISSION
bool MulPilot::arrivedAtHomeServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "NAVIGATING_TO_HOME")
  {
    changeState("WAITING_FOR_MISSION", "Arrived at home!");
    response.success = true;
    response.message = "Arrived at home! Switching from NAVIGATING_TO_HOME to WAITING_FOR_MISSION.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in NAVIGATING_TO_HOME state!";
    return true;
  }
  return false;
}
/* Transitions !*/

/* Callbacks */
//! Subscription Callbacks
// WAITING_FOR_MISSION --> CHECKING_ELEVATOR
void MulPilot::proxsensorSubCb(const odin_msgs::ProxSensor::ConstPtr &msg)
{
  if (current_state_ == "WAITING_FOR_MISSION")
  {
    std::string message = msg->message;
    RCOMPONENT_WARN_STREAM("Received message from Proximity Sensor: " + message);

    if (message == "action needed")
    {
      std_srvs::TriggerRequest mission_received_srv_request;
      std_srvs::TriggerResponse mission_received_srv_response;

      poi_x_ = msg->data.Posx;
      poi_y_ = msg->data.Posy;
      // RCOMPONENT_WARN_STREAM("POI coordinates: x=" << poi_x_ << ", y=" << poi_y_ << ", rot_z=" << poi_rot_z_ << ", rot_w=" << poi_rot_w_);
      RCOMPONENT_WARN_STREAM("POI coordinates: x=" << poi_x_ << ", y=" << poi_y_ << ", rot_z=" << poi_rot_z_);
      if (missionReceivedServiceCb(mission_received_srv_request, mission_received_srv_response))
      {
        if (mission_received_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from WAITING_FOR_MISSION to CHECKING_ELEVATOR");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from WAITING_FOR_MISSION to CHECKING_ELEVATOR: " << mission_received_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/mission_received");
      }
    }
  }
  tickTopicsHealth(proxsensor_sub_name_);
}

// WAITING_FOR_MISSION --> CHECKING_ELEVATOR
void MulPilot::smartboxSubCb(const odin_msgs::SmartboxStatus::ConstPtr &msg)
{
  if (current_state_ == "WAITING_FOR_MISSION")
  {
    float battery = msg->data.battery;

    if (battery < 10.0)
    {
      std_srvs::TriggerRequest mission_received_srv_request;
      std_srvs::TriggerResponse mission_received_srv_response;

      if (missionReceivedServiceCb(mission_received_srv_request, mission_received_srv_response))
      {
        if (mission_received_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from WAITING_FOR_MISSION to CHECKING_ELEVATOR");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from WAITING_FOR_MISSION to CHECKING_ELEVATOR: " << mission_received_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/mission_received");
      }
    }
  }
  tickTopicsHealth(smartbox_sub_name_);
}

// CHECKING_ELEVATOR --> GETTING_RACK_POSITION or NAVIGATING_TO_POI
void MulPilot::elevatorSubCb(const robotnik_msgs::ElevatorStatus::ConstPtr &msg)
{
  if (current_state_ == "CHECKING_ELEVATOR")
  {
    std::string message = msg->position;
    RCOMPONENT_WARN_STREAM("Received message from Elevator: " + message);

    // CHECKING_ELEVATOR --> GETTING_RACK_POSITION
    if (message == "down")
    {
      std_srvs::SetBoolRequest elevator_down_srv_request;
      std_srvs::SetBoolResponse elevator_down_srv_response;

      elevator_down_srv_request.data = true;

      if (elevatorDownServiceCb(elevator_down_srv_request, elevator_down_srv_response))
      {
        if (elevator_down_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from CHECKING_ELEVATOR to GETTING_RACK_POSITION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from CHECKING_ELEVATOR to GETTING_RACK_POSITION: " << elevator_down_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/elevator_down");
      }
    }

    // CHECKING_ELEVATOR --> NAVIGATING_TO_POI
    else if (message == "up")
    {
      std_srvs::SetBoolRequest elevator_down_srv_request;
      std_srvs::SetBoolResponse elevator_down_srv_response;

      elevator_down_srv_request.data = false;

      if (elevatorDownServiceCb(elevator_down_srv_request, elevator_down_srv_response))
      {
        if (elevator_down_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from CHECKING_ELEVATOR to NAVIGATING_TO_POI");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from CHECKING_ELEVATOR to NAVIGATING_TO_POI: " << elevator_down_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/elevator_down");
      }
    }
  }
  tickTopicsHealth(elevator_sub_name_);
}

// GETTING_RACK_POSITION --> CALCULATING_GOAL
void MulPilot::rtlsSubCb(const odin_msgs::RTLSBase::ConstPtr &msg)
{
  if (current_state_ == "GETTING_RACK_POSITION")
  {
    if (msg->data.id == "ble-pd-601283DE0245" or msg->data.id == "ble-pd-601283DE0246")
    {
      rack_x_ = msg->data.data.x;
      rack_y_ = msg->data.data.y;
      rack_z_ = msg->data.data.z;

      std_srvs::TriggerRequest rack_position_received_srv_request;
      std_srvs::TriggerResponse rack_position_received_srv_response;

      if (rackPositionReceivedServiceCb(rack_position_received_srv_request, rack_position_received_srv_response))
      {
        if (rack_position_received_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from GETTING_RACK_POSITION to CALCULATING_GOAL");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from GETTING_RACK_POSITION to CALCULATING_GOAL: " << rack_position_received_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/rack_position_received");
      }
    }
  }
  tickTopicsHealth(rtls_sub_name_);
}

void MulPilot::hmiSubCb(const odin_msgs::HMIBase::ConstPtr &msg)
{
  // WAITING_IN_POI --> NAVIGATING_TO_LAB
  if (current_state_ == "WAITING_IN_POI")
  {
    std::string message = msg->data.data.taskType;
    RCOMPONENT_WARN_STREAM("Received GO_TO_LAB from HMI");

    if (message == "GO_TO_LAB")
    {
      if (msg->data.data.endLocation.position.size() > 1 && msg->data.data.endLocation.orientation.size() > 1)
      {
        lab_pos_x_ = msg->data.data.endLocation.position[0];
        lab_pos_y_ = msg->data.data.endLocation.position[1];
        lab_pos_z_ = 0.0;             // msg->data.endLocation.position[2];
        lab_ori_x_ = 0.0;             // msg->data.endLocation.orientation[0];
        lab_ori_y_ = 0.0;             // msg->data.endLocation.orientation[1];
        lab_ori_z_ = -0.710204380492; // msg->data.endLocation.orientation[2];
        lab_ori_w_ = 0.703995552493;  // msg->data.endLocation.orientation[3];
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Invalid position and orientation data");
        return;
      }

      std_srvs::TriggerRequest go_to_lab_srv_request;
      std_srvs::TriggerResponse go_to_lab_srv_response;

      if (goToLabServiceCb(go_to_lab_srv_request, go_to_lab_srv_response))
      {
        if (go_to_lab_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from WAITING_IN_POI to NAVIGATING_TO_LAB");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from WAITING_IN_POI to NAVIGATING_TO_LAB: " << go_to_lab_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/go_to_lab");
      }
    }
  }

  // WAITING_IN_LAB --> RELEASING_RACK or HOMING_RACK
  if (current_state_ == "WAITING_IN_LAB")
  {
    std::string message = msg->data.data.taskType;
    float message_2 = msg->data.data.endLocation.position[0];
    // RCOMPONENT_WARN_STREAM("Received message from HMI: " + message);

    // WAITING_IN_LAB --> RELEASING_RACK
    if (message == "RELEASE_AND_HOME")
    {
      RCOMPONENT_WARN_STREAM("Received RELEASE_AND_HOME from HMI");
      std_srvs::SetBoolRequest release_and_home_srv_request;
      std_srvs::SetBoolResponse release_and_home_srv_response;

      release_and_home_srv_request.data = true;

      if (releaseRackServiceCb(release_and_home_srv_request, release_and_home_srv_response))
      {
        if (release_and_home_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from WAITING_IN_LAB to RELEASING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from WAITING_IN_LAB to RELEASING_RACK: " << release_and_home_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/release_rack");
      }
    }

    // WAITING_IN_LAB --> HOMING_RACK
    // else if (message == "BRING_RACK_HOME")
    else if (message_2 == 1.0)
    {
      RCOMPONENT_WARN_STREAM("Received BRING_RACK_HOME from HMI");
      std_srvs::SetBoolRequest release_and_home_srv_request;
      std_srvs::SetBoolResponse release_and_home_srv_response;

      release_and_home_srv_request.data = false;

      if (releaseRackServiceCb(release_and_home_srv_request, release_and_home_srv_response))
      {
        if (release_and_home_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from WAITING_IN_LAB to HOMING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from WAITING_IN_LAB to HOMING_RACK: " << release_and_home_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/release_rack");
      }
    }
  }
  tickTopicsHealth(hmi_sub_name_);
}

//! Action Callbacks
void MulPilot::moveBaseResultCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_srvs::TriggerRequest move_base_srv_request;
    std_srvs::TriggerResponse move_base_srv_response;

    // NAVIGATING_TO_RACK --> PICKING_RACK
    if (current_state_ == "NAVIGATING_TO_RACK")
    {
      if (arrivedAtRackServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from NAVIGATING_TO_RACK to PICKING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from NAVIGATING_TO_RACK to PICKING_RACK: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/arrived_at_rack");
      }
    }

    // NAVIGATING_TO_POI --> WAITING_IN_POI
    if (current_state_ == "NAVIGATING_TO_POI")
    {
      if (arrivedAtPoiServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from NAVIGATING_TO_POI to WAITING_IN_POI");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from NAVIGATING_TO_POI to WAITING_IN_POI: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/arrived_at_poi");
      }
    }

    // NAVIGATING_TO_LAB --> WAITING_IN_LAB
    if (current_state_ == "NAVIGATING_TO_LAB")
    {
      if (arrivedAtLabServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from NAVIGATING_TO_LAB to WAITING_IN_LAB");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from NAVIGATING_TO_LAB to WAITING_IN_LAB: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/arrived_at_lab");
      }
    }

    // HOMING_RACK --> PLACING_RACK
    if (current_state_ == "HOMING_RACK")
    {
      if (rackHomedServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from HOMING_RACK to PLACING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from HOMING_RACK to PLACING_RACK: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/rack_homed");
      }
    }

    // NAVIGATING_TO_HOME --> WAITING_FOR_MISSION
    if (current_state_ == "NAVIGATING_TO_HOME")
    {
      if (arrivedAtHomeServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from NAVIGATING_TO_HOME to WAITING_FOR_MISSION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from NAVIGATING_TO_HOME to WAITING_FOR_MISSION: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/arrived_at_home");
      }
    }
  }
}

void MulPilot::commandSequencerResultCb(const actionlib::SimpleClientGoalState &state, const robot_simple_command_manager_msgs::RobotSimpleCommandResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_srvs::TriggerRequest command_sequencer_srv_request;
    std_srvs::TriggerResponse command_sequencer_srv_response;

    // PICKING_RACK --> NAVIGATING_TO_POI
    if (current_state_ == "PICKING_RACK")
    {
      if (rackPickedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from PICKING_RACK to NAVIGATING_TO_POI");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from PICKING_RACK to NAVIGATING_TO_POI: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/rack_picked");
      }
    }

    // PLACING_RACK --> NAVIGATING_TO_HOME
    if (current_state_ == "PLACING_RACK")
    {
      if (rackPlacedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from PLACING_RACK to NAVIGATING_TO_HOME");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from PLACING_RACK to NAVIGATING_TO_HOME: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/rack_placed");
      }
    }

    // RELEASING_RACK --> NAVIGATING_TO_HOME
    if (current_state_ == "RELEASING_RACK")
    {
      if (rackReleasedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from RELEASING_RACK to NAVIGATING_TO_HOME");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from RELEASING_RACK to NAVIGATING_TO_HOME: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/rack_released");
      }
    }
  }
}
/* Callbacks !*/
