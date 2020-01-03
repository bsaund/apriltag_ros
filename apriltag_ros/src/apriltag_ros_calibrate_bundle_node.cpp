#include <ros/ros.h>
#include "apriltag_ros/bundle_calibrator.h"

using namespace apriltag_ros;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_ros");
    std::cout << "running bundle calibrator node\n";
    BundleCalibrator bundle_calibrator;

    ros::spin();
}
