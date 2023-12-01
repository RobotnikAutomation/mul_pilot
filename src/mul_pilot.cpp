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
}

int MulPilot::rosSetup()
{
  RComponent::rosSetup();

  bool required = true;
  bool not_required = false;

  // Publisher
  status_pub_ = pnh_.advertise<std_msgs::String>("status", 10);
  status_stamped_pub_ = pnh_.advertise<robotnik_msgs::StringStamped>("status_stamped", 10);

  robot_status_pub_ = pnh_.advertise<odin_msgs::RobotStatus>(robot_status_pub_name_, 10);
  robot_result_pub_ = pnh_.advertise<odin_msgs::RobotTask>(robot_result_pub_name_, 10);
  interface_pub_ = pnh_.advertise<odin_msgs::RobotTask>(interface_pub_name_, 10);

  // Subscriber
  proxsensor_status_sub_ = nh_.subscribe<odin_msgs::ProxSensor>(proxsensor_status_sub_name_, 10, &MulPilot::proxsensorStatusSubCb, this);
  addTopicsHealth(&proxsensor_status_sub_, proxsensor_status_sub_name_, 50.0, required);

  iot_rtls_positions_sub_ = nh_.subscribe<odin_msgs::RTLS>(iot_rtls_positions_sub_name_, 10, &MulPilot::iotRtlsPositionsSubCb, this);
  addTopicsHealth(&iot_rtls_positions_sub_, iot_rtls_positions_sub_name_, 50.0, required);

  // Service
  // example_server_ = pnh_.advertiseService("example", &MulPilot::exampleServerCb, this);

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

void MulPilot::proxsensorStatusSubCb(const odin_msgs::ProxSensor::ConstPtr &msg)
{
  RCOMPONENT_WARN_STREAM("Received msg: " + msg->version);

  tickTopicsHealth("proxsensor_status");
}

void MulPilot::iotRtlsPositionsSubCb(const odin_msgs::RTLS::ConstPtr &msg)
{
  RCOMPONENT_WARN_STREAM("Received msg: " + msg->version);

  tickTopicsHealth("iot_rtls_positions");
}

// bool MulPilot::exampleServerCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
// {
//   RCOMPONENT_WARN_STREAM("Received srv trigger petition.");
//   if (state != robotnik_msgs::State::READY_STATE)
//   {
//     response.success = false;
//     response.message = "Received srv trigger petition. Component not ready.";
//     return true;
//   }
//   else
//   {
//     response.success = true;
//     response.message = "Received srv trigger petition. Component ready.";
//     return true;
//   }
//   return false;
// }
