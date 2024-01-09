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
  readParam(pnh_, "interface_pub", interface_pub_name_, "/mul_pilot/interface", required);
  readParam(pnh_, "proxsensor_sub", proxsensor_sub_name_, "/mul_pilot/proxsensor", required);
  readParam(pnh_, "rtls_sub", rtls_sub_name_, "/mul_pilot/rtls", required);
  readParam(pnh_, "smartbox_sub", smartbox_sub_name_, "/mul_pilot/smartbox", required);
  readParam(pnh_, "pick_sequence", pick_sequence_, "ELEVATOR_UP_DOWN_VICTOR", required);
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
  interface_pub_ = pnh_.advertise<odin_msgs::RobotTask>(interface_pub_name_, 10);

  //! Subscribers
  // Proximity Sensor
  proxsensor_sub_ = nh_.subscribe<odin_msgs::ProxSensor>(proxsensor_sub_name_, 10, &MulPilot::proxsensorSubCb, this);
  addTopicsHealth(&proxsensor_sub_, proxsensor_sub_name_, 50.0, not_required);

  // Smartbox
  smartbox_sub_ = nh_.subscribe<odin_msgs::SmartboxStatus>(smartbox_sub_name_, 10, &MulPilot::smartboxSubCb, this);
  addTopicsHealth(&smartbox_sub_, smartbox_sub_name_, 50.0, not_required);

  // RTLS
  rtls_sub_ = nh_.subscribe<odin_msgs::RTLS>(rtls_sub_name_, 10, &MulPilot::rtlsSubCb, this);
  addTopicsHealth(&rtls_sub_, rtls_sub_name_, 50.0, not_required);

  //! Service Servers
  out_of_battery_srv_ = pnh_.advertiseService("/mul_pilot/out_of_battery", &MulPilot::outOfBatteryServiceCb, this);
  location_received_srv_ = pnh_.advertiseService("/mul_pilot/location_received", &MulPilot::locationReceivedServiceCb, this);
  goal_calculated_srv_ = pnh_.advertiseService("/mul_pilot/goal_calculated", &MulPilot::goalCalculatedServiceCb, this);
  arrived_at_rack_srv_ = pnh_.advertiseService("/mul_pilot/arrived_at_rack", &MulPilot::arrivedAtRackServiceCb, this);
  rack_picked_srv_ = pnh_.advertiseService("/mul_pilot/rack_picked", &MulPilot::rackPickedServiceCb, this);
  arrived_at_home_srv_ = pnh_.advertiseService("/mul_pilot/arrived_at_home", &MulPilot::arrivedAtHomeServiceCb, this);

  //! Service Clients
  out_of_battery_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/out_of_battery");
  location_received_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/location_received");
  goal_calculated_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/goal_calculated");
  arrived_at_rack_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/arrived_at_rack");
  rack_picked_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/rack_picked");
  arrived_at_home_client_ = pnh_.serviceClient<std_srvs::Trigger>("/mul_pilot/arrived_at_home");

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
  pick_command_sent_ = false;
}

/* State Machine !*/

/*** States ***/

//! WAITING_FOR_MISSION
void MulPilot::waitingForMissionState()
{
  RCOMPONENT_INFO_STREAM("WAITING_FOR_MISSION");
}

//! GETTING_LOCATION
void MulPilot::gettingLocationState()
{
  RCOMPONENT_INFO_STREAM("GETTING_LOCATION");
}

//! CALCULATING_GOAL
void MulPilot::calculatingGoalState()
{
  RCOMPONENT_INFO_STREAM("CALCULATING_GOAL");

  // TODO: Get correct coordinates from docking station location
  double distance1;
  distance1 = sqrt(pow(x_ - x1_, 2) + pow(y_ - y1_, 2) + pow(z_ - z1_, 2));

  double distance2;
  distance2 = sqrt(pow(x_ - x2_, 2) + pow(y_ - y2_, 2) + pow(z_ - z2_, 2));

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
    move_base_goal_.target_pose.pose.orientation.z = -0.707106781;
    move_base_goal_.target_pose.pose.orientation.w = 0.707106781;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&MulPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! PICKING_RACK
void MulPilot::pickingRackState()
{
  RCOMPONENT_INFO_STREAM("PICKING_RACK");
  if (!pick_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending sequence to pick the rack...");

    // TODO: Set correct command
    command_sequencer_goal_.command.command = pick_sequence_;
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&MulPilot::commandSequencerResultCb, this, _1, _2));

    pick_command_sent_ = true;
  }
}

//! NAVIGATING_TO_HOME
void MulPilot::navigatingToHomeState()
{
  RCOMPONENT_INFO_STREAM("NAVIGATING_TO_HOME");
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
    response.message = "Smartbox is out of battery! Switching from WAITING_FOR_MISSION to GETTING_LOCATION.";
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

//! GETTING_LOCATION --> CALCULATING_GOAL
bool MulPilot::locationReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "GETTING_LOCATION")
  {
    changeState("CALCULATING_GOAL", "Location received from RTLS: x=" + std::to_string(x_) + ", y=" + std::to_string(y_) + ", z=" + std::to_string(z_));
    response.success = true;
    response.message = "Location received! Switching from GETTING_LOCATION to CALCULATING_GOAL.";
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

//! PICKING_RACK --> NAVIGATING_TO_HOME
bool MulPilot::rackPickedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "PICKING_RACK")
  {
    changeState("NAVIGATING_TO_HOME", "Rack picked!");
    response.success = true;
    response.message = "Rack picked! Switching from PICKING_RACK to NAVIGATING_TO_HOME.";
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
// WAITING_FOR_MISSION --> GETTING_LOCATION
void MulPilot::proxsensorSubCb(const odin_msgs::ProxSensor::ConstPtr &msg)
{
  if (current_state_ == "WAITING_FOR_MISSION")
  {
    std::string message = msg->message;
    RCOMPONENT_WARN_STREAM("Received msg (Proximity Sensor): " + message);

    if (message == "action needed")
    {
      std_srvs::TriggerRequest out_of_battery_srv_request;
      std_srvs::TriggerResponse out_of_battery_srv_response;

      if (outOfBatteryServiceCb(out_of_battery_srv_request, out_of_battery_srv_response))
      {
        if (out_of_battery_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from WAITING_FOR_MISSION to GETTING_LOCATION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from WAITING_FOR_MISSION to GETTING_LOCATION: " << out_of_battery_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/out_of_battery");
      }
    }
  }
  tickTopicsHealth(proxsensor_sub_name_);
}

// WAITING_FOR_MISSION --> GETTING_LOCATION
void MulPilot::smartboxSubCb(const odin_msgs::SmartboxStatus::ConstPtr &msg)
{
  if (current_state_ == "WAITING_FOR_MISSION")
  {
    float battery = msg->data.battery;

    if (battery < 10.0)
    {
      std_srvs::TriggerRequest out_of_battery_srv_request;
      std_srvs::TriggerResponse out_of_battery_srv_response;

      if (outOfBatteryServiceCb(out_of_battery_srv_request, out_of_battery_srv_response))
      {
        if (out_of_battery_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from WAITING_FOR_MISSION to GETTING_LOCATION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from WAITING_FOR_MISSION to GETTING_LOCATION: " << out_of_battery_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/out_of_battery");
      }
    }
  }
  tickTopicsHealth(smartbox_sub_name_);
}

// GETTING_LOCATION --> CALCULATING_GOAL
void MulPilot::rtlsSubCb(const odin_msgs::RTLS::ConstPtr &msg)
{
  if (current_state_ == "GETTING_LOCATION")
  {
    x_ = msg->data.x;
    y_ = msg->data.y;
    z_ = msg->data.z;

    std_srvs::TriggerRequest location_received_srv_request;
    std_srvs::TriggerResponse location_received_srv_response;

    if (locationReceivedServiceCb(location_received_srv_request, location_received_srv_response))
    {
      if (location_received_srv_response.success)
      {
        RCOMPONENT_INFO_STREAM("Successfully switched from GETTING_LOCATION to CALCULATING_GOAL");
      }
      else
      {
        RCOMPONENT_WARN_STREAM("Failed to switch from GETTING_LOCATION to CALCULATING_GOAL: " << location_received_srv_response.message.c_str());
      }
    }
    else
    {
      RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/location_received");
    }
  }
  tickTopicsHealth(rtls_sub_name_);
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

    // PICKING_RACK --> NAVIGATING_TO_HOME
    if (current_state_ == "PICKING_RACK")
    {
      if (rackPickedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from PICKING_RACK to NAVIGATING_TO_HOME");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from PICKING_RACK to NAVIGATING_TO_HOME: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /mul_pilot/rack_picked");
      }
    }
  }
}

/* Callbacks !*/
