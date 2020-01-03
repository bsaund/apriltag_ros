#include "apriltag_ros/bundle_calibrator.h"
#include <algorithm>


using namespace apriltag_ros;


BundleCalibrator::BundleCalibrator()
{
    std::cout << "Initializing bundle calibrator\n";
    ros::NodeHandle n;
    tag_detector_sub = n.subscribe("tag_detections", 100, &BundleCalibrator::tagDetectionCallback, this);
};


bool BundleCalibrator::tooSimilarToPrevious(const AprilTagDetectionArray &query)
{
    if(calibration_data.size() == 0)
    {
        return false;
    }
    
    const AprilTagDetectionArray prev = calibration_data.back();

    std::vector<int> q_ids;
    for(const auto &tag: query.detections)
    {
        q_ids.push_back(tag.id[0]);
    }

    std::vector<int> p_ids;
    for(const auto &tag: prev.detections)
    {
        p_ids.push_back(tag.id[0]);
    }

    if(q_ids.size() != p_ids.size())
    {
        return false;
    }
    
    std::sort(q_ids.begin(), q_ids.end());
    std::sort(p_ids.begin(), p_ids.end());

    for(size_t i=0; i<q_ids.size(); i++)
    {
        if(q_ids[i] != p_ids[i])
        {
            return false;
        }
    }


    

    return true;
};

void BundleCalibrator::tagDetectionCallback(const AprilTagDetectionArray &tag_detection_array)
{
    //Remove bundle detections
    AprilTagDetectionArray tags;
    tags.header = tag_detection_array.header;
    for(const auto &tag: tag_detection_array.detections)
    {
        if(tag.id.size() == 1)
        {
            tags.detections.push_back(tag);
        }
    }

    if(tags.detections.size() < 2)
    {
        return;
    }



    if(!tooSimilarToPrevious(tags))
    {
        calibration_data.push_back(tags);
        std::cout << calibration_data.size() << " data points\n";
    }
};

