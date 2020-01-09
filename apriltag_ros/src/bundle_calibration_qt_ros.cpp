#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <cmath>

#include "apriltag_ros/bundle_calibration_qt_ros.hpp"
#include "apriltag_ros/ceres_bundle_solver.hpp"



/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

using namespace apriltag_ros;

/*****************************************************************************
 ** Implementation
 *****************************************************************************/


/*****************************************************************************
 ** Calibration Data Structures
 *****************************************************************************/
tag_for_calibration::tag_for_calibration(const apriltag_detection_t* original)
    : id(original->id)
{
    // Add to image point vector the tag corners in the image frame
    // Going counterclockwise starting from the bottom left corner
    double tag_x[4] = {-1,1,1,-1};
    double tag_y[4] = {1,1,-1,-1}; // Negated because AprilTag tag local
    // frame has y-axis pointing DOWN
    // while we use the tag local frame
    // with y-axis pointing UP
    for(int i=0; i<4; i++)
    {
        double im_x, im_y;
        homography_project(original->H, tag_x[i], tag_y[i], &im_x, &im_y);
        corners[i][0] = im_x;
        corners[i][1] = im_y;
    }
}


calibration_datum::calibration_datum(const zarray_t* detections)
{
    for(int i=0; i<zarray_size(detections); i++)
    {
        apriltag_detection_t* detection;
        zarray_get(detections, i, &detection);
        tags.emplace_back(detection);
    }

    std::sort(tags.begin(), tags.end(),
              [](tag_for_calibration &lhs, tag_for_calibration &rhs) {return lhs.id < rhs.id;});
}




/*****************************************************************************
 ** QNode
 *****************************************************************************/


QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"apriltag_ros_bundle_calibration");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    it_ = std::unique_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(nh));
        // std::make_unique<image_transport::ImageTransport>(nh);
        // std::shared_ptr<image_transport::ImageTransport>(
        // new image_transport::ImageTransport(nh));

    camera_image_subscriber_ =
        it_->subscribeCamera("image_rect", 1,
                             &QNode::imageCallback, this);

    tag_detector_ = std::unique_ptr<TagDetector>(
        new TagDetector(pnh));
        // std::make_unique<TagDetector>(pnh);

    // Add your ros communications here.
    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(1);
    int count = 0;
    while ( ros::ok() ) {
        ros::spinOnce();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

QPixmap& QNode::getPixmap()
{
    // Note: This should be guarded by a mutex, since we are updating the image in a different thread.
    // In practice it is not critical for this use case since the returned pixmap is just used to update
    // the diplayed camera image
    return pixmap;
}


void QNode::addToObservedSet(int id)
{
    if(observed_tags.count(id))
    {
        return;
    }

    observed_tags.insert(id);
    Q_EMIT newTagObserved(id);
}


bool QNode::tooSimilarToPrevious(const calibration_datum &cur) const
{
    if(calibration_data.size() == 0)
    {
        return false;
    }

    const calibration_datum &prev = calibration_data.back();

    
    if(prev.tags.size() != cur.tags.size())
    {
        return false;
    }

    // if(cur.tags.size() > 0)
    // {
    //     std::cout << "cur corner 0: (" << cur.tags[0].corners[0][0] << ", " << cur.tags[0].corners[0][1] << ")\n";
    // }

    for(int i=0; i<prev.tags.size(); i++)
    {
        if(prev.tags[i].id != cur.tags[i].id)
        {
            return false;
        }

        // std::cout << "\n";
        for(int j=0; j<4; j++)
        {
            if(std::abs(cur.tags[i].corners[j][0] - prev.tags[i].corners[j][0]) >
               PIXEL_MOTION_THRESHOLD_FOR_NEW_CALIBRATION_POINT ||
               std::abs(cur.tags[i].corners[j][1] - prev.tags[i].corners[j][1]) >
               PIXEL_MOTION_THRESHOLD_FOR_NEW_CALIBRATION_POINT)
            {
                return false;
            }
        }
    }
    return true;
}


void QNode::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    
    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    try
    {
        cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    tag_detector_->detectTags(cv_image_,camera_info);

    calibration_datum detections(tag_detector_->getDetections());


    if(detections.tags.size() > 1 && 
       !tooSimilarToPrevious(detections))
    {
        calibration_data.push_back(detections);
    }
    
    std::vector<zarray_t> calibration_data;

    visible_tags.clear();
    for(const tag_for_calibration &tag: detections.tags)
    {
        addToObservedSet(tag.id);
        visible_tags.insert(tag.id);
    }
    tag_detector_->drawDetections(cv_image_);

    int h = cv_image_->image.size().height;
    int w = cv_image_->image.size().width;

    cv::cvtColor(cv_image_->image, cv_image_->image, cv::COLOR_BGR2RGB);

    
    pixmap = QPixmap::fromImage(QImage(cv_image_->image.data, w, h, w*3, QImage::Format_RGB888));
    
    Q_EMIT imageUpdated();
}

/*
 *  Remove all tags that are not in the bundle to calibrate
 */
std::vector<calibration_datum> QNode::cleanCalibrationData(std::set<int> tags_to_calibrate) const
{
    // std::cout << "Cleaning calibration data, keeping only: ";
    // for(int t: tags_to_calibrate)
    // {
    //     std::cout << t << ", ";
    // }
    // std::cout << "\n";
    std::vector<calibration_datum> cleaned_calibration_data;

    for(const calibration_datum &original_datum: calibration_data)
    {
        calibration_datum cleaned_datum;
        for(const tag_for_calibration &orig_tag: original_datum.tags)
        {
            if(tags_to_calibrate.count(orig_tag.id))
            {
                cleaned_datum.tags.push_back(orig_tag);
            }
        }
        if(cleaned_datum.tags.size() > 1)
        {
            cleaned_calibration_data.push_back(cleaned_datum);
        }
    }
    return cleaned_calibration_data;
}


void QNode::calibrateBundle(int bundle_id)
{

    auto tag_ids = getTagBundleDescriptions()[bundle_id-1].bundleIds();

    auto data = cleanCalibrationData(std::set<int>(tag_ids.begin(), tag_ids.end()));
    std::cout << "Calibration data size: " << data.size() << "\n";

    std::cout << "Calibrating bundle with tag ids: ";    
    for(int t: tag_ids)
    {
        std::cout << t << ", ";
    }
    std::cout << "\n";

    CeresBundleSolver s;
    s.solve(9);
}
