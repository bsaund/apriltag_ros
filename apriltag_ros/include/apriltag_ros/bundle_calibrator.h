#ifndef APRILTAG_ROS_BUNDLE_CALIBRATOR
#define APRILTAG_ROS_BUNDLE_CALIBRATOR

#include <ros/ros.h>

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"


namespace apriltag_ros
{
    
    class BundleCalibrator
    {
    public:
        BundleCalibrator();

    protected:
        void tagDetectionCallback(const AprilTagDetectionArray &tag_detection_array);
        bool tooSimilarToPrevious(const AprilTagDetectionArray &query);
            
    protected:
        ros::Subscriber tag_detector_sub;
        
        std::vector<AprilTagDetectionArray> calibration_data;
    };
    
} // namespace apriltag_ros

#endif // APRILTAG_ROS_BUNDLE_CALIBRATOR
