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

    {
        //Check if detected tags are the different
        if(prev.detections.size() != query.detections.size())
        {
            return false;
        }
        for(size_t i=0; i<prev.detections.size(); i++)
        {
            if(prev.detections[i].id[0] != query.detections[i].id[0])
            {
                return false;
            }
        }
    }

    for(size_t i=0; i<prev.detections.size(); i++)
    {
        const auto &pp = prev.detections[i].pose.pose.pose.position;
        const auto &qp = query.detections[i].pose.pose.pose.position;

        double d_sq = (pp.x - qp.x)*(pp.x-qp.x) +
            (pp.y - qp.y)*(pp.x-qp.y) +
            (pp.z - qp.z)*(pp.x-qp.z);
        double tag_size = prev.detections[i].size[0];
        if(d_sq > (tag_size * tag_size))
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

    auto detection_sort_fn = [](const AprilTagDetection &lhs, const AprilTagDetection &rhs)
        {return lhs.id[0] < rhs.id[0];};


    std::sort(tags.detections.begin(), tags.detections.end(), detection_sort_fn);

    if(!tooSimilarToPrevious(tags))
    {
        calibration_data.push_back(tags);
        std::cout << calibration_data.size() << " data points\n";
    }
};

