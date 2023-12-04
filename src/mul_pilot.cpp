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
  readParam(pnh_, "robot_status_pub", robot_status_pub_name_, "robot_status", required);
  readParam(pnh_, "robot_result_pub", robot_result_pub_name_, "robot_result", required);
  readParam(pnh_, "interface_pub", interface_pub_name_, "interface", required);
  readParam(pnh_, "proxsensor_status_sub", proxsensor_status_sub_name_, "proxsensor_status", required);
  readParam(pnh_, "iot_rtls_positions_sub", iot_rtls_positions_sub_name_, "iot_rtls_positions", required);
  readParam(pnh_, "smartbox_status_sub", smartbox_status_sub_name_, "smartbox_status", required);
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
  state_pub_ = pnh_.advertise<std_msgs::String>("rb1_state", 10);

  robot_status_pub_ = pnh_.advertise<odin_msgs::RobotStatus>(robot_status_pub_name_, 10);
  robot_result_pub_ = pnh_.advertise<odin_msgs::RobotTask>(robot_result_pub_name_, 10);
  interface_pub_ = pnh_.advertise<odin_msgs::RobotTask>(interface_pub_name_, 10);

  //! Subscribers
  // Proximity Sensor
  proxsensor_status_sub_ = nh_.subscribe<odin_msgs::ProxSensor>(proxsensor_status_sub_name_, 10, &MulPilot::proxsensorStatusSubCb, this);
  addTopicsHealth(&proxsensor_status_sub_, proxsensor_status_sub_name_, 50.0, required);

  // RTLS
  iot_rtls_positions_sub_ = nh_.subscribe<odin_msgs::RTLS>(iot_rtls_positions_sub_name_, 10, &MulPilot::iotRtlsPositionsSubCb, this);
  addTopicsHealth(&iot_rtls_positions_sub_, iot_rtls_positions_sub_name_, 50.0, required);

  // Smartbox
  smartbox_status_sub_ = nh_.subscribe<odin_msgs::SmartboxStatus>(smartbox_status_sub_name_, 10, &MulPilot::smartboxStatusSubCb, this);
  addTopicsHealth(&smartbox_status_sub_, smartbox_status_sub_name_, 50.0, required);

  //! Service Servers
  out_of_battery_srv_ = pnh_.advertiseService("out_of_battery", &MulPilot::outOfBatteryServiceCb, this);
  location_received_srv_ = pnh_.advertiseService("location_received", &MulPilot::locationReceivedServiceCb, this);
  arrived_at_rack_srv_ = pnh_.advertiseService("arrived_at_rack", &MulPilot::arrivedAtRackServiceCb, this);
  rack_picked_srv_ = pnh_.advertiseService("rack_picked", &MulPilot::rackPickedServiceCb, this);
  arrived_at_home_srv_ = pnh_.advertiseService("arrived_at_home", &MulPilot::arrivedAtHomeServiceCb, this);

  //! Service Clients
  out_of_battery_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/out_of_battery");
  location_received_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/location_received");
  arrived_at_rack_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/arrived_at_rack");
  rack_picked_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/rack_picked");
  arrived_at_home_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/arrived_at_home");

  //! Actions
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

  current_state_ros_.data = current_state_;
  state_pub_.publish(current_state_ros_);

  navigation_command_sent_ = false;
  pick_command_sent_ = false;

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
  else if (current_state_ == "GETTING_LOCATION")
  {
    gettingLocationState();
  }
  else if (current_state_ == "NAVIGATING_TO_RACK")
  {
    navigatingToRackState();
  }
  else if (current_state_ == "PICKING_RACK")
  {
    pickingRackState();
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

  current_state_ros_.data = current_state_;
  state_pub_.publish(current_state_ros_);

  navigation_command_sent_ = false;
  pick_command_sent_ = false;
}
/* State Machine !*/

/*** States ***/
//! WAITING_FOR_MISSION
void MulPilot::waitingForMissionState()
{
  ROS_INFO("WAITING_FOR_MISSION");
}

//! GETTING_LOCATION
void MulPilot::gettingLocationState()
{
  ROS_INFO("GETTING_LOCATION");
}

//! NAVIGATING_TO_RACK
void MulPilot::navigatingToRackState()
{
  ROS_INFO("NAVIGATING_TO_RACK");
  if (!navigation_command_sent_)
  {
    ROS_INFO("Sending command to navigate to the rack...");

    // TODO: Change this to the correct frame and coordinates
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.pose.position.x = 3.0;
    move_base_goal_.target_pose.pose.position.y = -1.0;
    move_base_goal_.target_pose.pose.position.z = 0.0;
    move_base_goal_.target_pose.pose.orientation.x = 0.0;
    move_base_goal_.target_pose.pose.orientation.y = 0.0;
    move_base_goal_.target_pose.pose.orientation.z = -0.707106781;
    move_base_goal_.target_pose.pose.orientation.w = 0.707106781;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&MulPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! PICKING_RACK
void MulPilot::pickingRackState()
{
  ROS_INFO("PICKING_RACK");
  if (!pick_command_sent_)
  {
    ROS_INFO("Sending sequence to pick the rack...");

    // TODO: Check functionality
    command_sequencer_goal_.command.command = "PICK_RACK";
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&MulPilot::commandSequencerResultCb, this, _1, _2));

    pick_command_sent_ = true;
  }
}

//! NAVIGATING_TO_HOME
void MulPilot::navigatingToHomeState()
{
  ROS_INFO("NAVIGATING_TO_HOME");
}
/* States !*/

/*** Transitions ***/
//! WAITING_FOR_MISSION --> GETTING_LOCATION
bool MulPilot::outOfBatteryServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "WAITING_FOR_MISSION")
  {
    changeState("GETTING_LOCATION", "Smartbox is out of battery!");
    response.success = true;
    response.message = "Smartbox is out of battery! Changing state to GETTING_LOCATION.";
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

//! GETTING_LOCATION --> NAVIGATING_TO_RACK
bool MulPilot::locationReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "GETTING_LOCATION")
  {
    changeState("NAVIGATING_TO_RACK", "Location received from RTLS!");
    response.success = true;
    response.message = "Location received! Changing state to NAVIGATING_TO_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in GETTING_LOCATION state!";
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
    response.message = "Arrived at rack! Changing state to PICKING_RACK.";
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

//! PICKING_RACK --> NAVIGATING_TO_HOME
bool MulPilot::rackPickedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "PICKING_RACK")
  {
    changeState("NAVIGATING_TO_HOME", "Rack picked!");
    response.success = true;
    response.message = "Rack picked! Changing state to NAVIGATING_TO_HOME.";
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

//! NAVIGATING_TO_HOME --> WAITING_FOR_MISSION
bool MulPilot::arrivedAtHomeServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "NAVIGATING_TO_HOME")
  {
    changeState("WAITING_FOR_MISSION", "Arrived at home!");
    response.success = true;
    response.message = "Arrived at home! Changing state to WAITING_FOR_MISSION.";
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
void MulPilot::proxsensorStatusSubCb(const odin_msgs::ProxSensor::ConstPtr &msg)
{
  RCOMPONENT_WARN_STREAM("Received msg (Proximity Sensor): " + msg->version);
  tickTopicsHealth("proxsensor_status");
}

void MulPilot::iotRtlsPositionsSubCb(const odin_msgs::RTLS::ConstPtr &msg)
{
  RCOMPONENT_WARN_STREAM("Received msg (RTLS): " + msg->version);
  tickTopicsHealth("iot_rtls_positions");
}

void MulPilot::smartboxStatusSubCb(const odin_msgs::SmartboxStatus::ConstPtr &msg)
{
  RCOMPONENT_WARN_STREAM("Received msg (Smartbox): " + msg->version);
  tickTopicsHealth("smartbox_status");
}

//! Action Callbacks
void MulPilot::moveBaseResultCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_srvs::Trigger move_base_srv_trigger;

    //! NAVIGATING_TO_RACK
    if (current_state_ == "NAVIGATING_TO_RACK")
    {
      if (arrived_at_rack_client_.call(move_base_srv_trigger))
      {
        if (move_base_srv_trigger.response.success)
        {
          ROS_INFO("Successfully changed state to PICKING_RACK");
        }
        else
        {
          ROS_WARN("Failed to change state to PICKING_RACK: %s", move_base_srv_trigger.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("Failed to call service /mul_pilot/arrived_at_rack");
      }
    }

    //! NAVIGATING_TO_HOME
    if (current_state_ == "NAVIGATING_TO_HOME")
    {
      if (arrived_at_home_client_.call(move_base_srv_trigger))
      {
        if (move_base_srv_trigger.response.success)
        {
          ROS_INFO("Successfully changed state to WAITING_FOR_MISSION");
        }
        else
        {
          ROS_WARN("Failed to change state to WAITING_FOR_MISSION: %s", move_base_srv_trigger.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("Failed to call service /mul_pilot/arrived_at_home");
      }
    }
  }
}

void MulPilot::commandSequencerResultCb(const actionlib::SimpleClientGoalState &state, const robot_simple_command_manager_msgs::RobotSimpleCommandResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_srvs::Trigger command_sequencer_srv_trigger;

    //! PICKING_RACK
    if (current_state_ == "PICKING_RACK")
    {
      if (rack_picked_client_.call(command_sequencer_srv_trigger))
      {
        if (command_sequencer_srv_trigger.response.success)
        {
          ROS_INFO("Successfully changed state to NAVIGATING_TO_HOME");
        }
        else
        {
          ROS_WARN("Failed to change state to NAVIGATING_TO_HOME: %s", command_sequencer_srv_trigger.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("Failed to call service /mul_pilot/rack_picked");
      }
    }
  }
}
/* Callbacks !*/
