#include <mul_pilot/mul_pilot.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mul_pilot");
  ros::NodeHandle n;
  
  MulPilot mul_pilot(n);
  mul_pilot.start();
}
