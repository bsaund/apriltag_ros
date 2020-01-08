#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "apriltag_ros/bundle_calibration_qt_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

using namespace apriltag_ros;

/*****************************************************************************
 ** Implementation
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


bool QNode::tooSimilarToPrevious(const zarray_t* detection) const
{
    return false;
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


    auto detections = tag_detector_->getDetections();

    if(!tooSimilarToPrevious(detections))
    {
        calibration_data.emplace_back(zarray_copy(detections));
    }
    std::vector<zarray_t> calibration_data;
    // std::cout << "Detected tag: ";

    visible_tags.clear();
    for(int i=0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        addToObservedSet(det->id);
        visible_tags.insert(det->id);
        // std::cout << det->id << ", ";
    }
    // std::cout << "\n";
    // observed_tags///
    
    
    tag_detector_->drawDetections(cv_image_);

    int h = cv_image_->image.size().height;
    int w = cv_image_->image.size().width;

    cv::cvtColor(cv_image_->image, cv_image_->image, cv::COLOR_BGR2RGB);

    
    pixmap = QPixmap::fromImage(QImage(cv_image_->image.data, w, h, w*3, QImage::Format_RGB888));
    
    Q_EMIT imageUpdated();
}

