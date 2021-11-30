#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "differential_driver");
  ros::NodeHandle nh;

  // Construct an object here

  ros::spin();
  return 0;
}
