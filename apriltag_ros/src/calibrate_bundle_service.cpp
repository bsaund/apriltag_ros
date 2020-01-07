#include <ros/ros.h>
#include <apriltag_ros/CalibrateBundle.h>

using namespace apriltag_ros;
bool calibrateBundle(CalibrateBundle::Request &req,
                     CalibrateBundle::Response &res)
{
    ROS_INFO("Called calibration service");
    return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_ros");
  ros::NodeHandle n;

  ros::ServiceServer calibration_service = n.advertiseService("calibrate_bundle", calibrateBundle);

  ros::spin();
  return 0;
}
