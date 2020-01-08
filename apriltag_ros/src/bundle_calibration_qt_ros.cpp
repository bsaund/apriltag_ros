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
    return pixmap;
}


void QNode::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    
    std::cout << "New image received\n";
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

    // Publish detected tags in the image by AprilTag 2
    tag_detector_->detectTags(cv_image_,camera_info);
//     tag_detections_publisher_.publish(
// );

    // Publish the camera image overlaid by outlines of the detected tags and
    // their payload values
    // if (draw_tag_detections_image_)
    // {
    tag_detector_->drawDetections(cv_image_);
    // tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    // }

    int h = cv_image_->image.size().height;
    int w = cv_image_->image.size().width;
    std::cout << "cv_image is: (" << w << ", " << h << ")\n"; 

    cv::cvtColor(cv_image_->image, cv_image_->image, cv::COLOR_BGR2RGB);
    pixmap = QPixmap::fromImage(QImage(cv_image_->image.data, w, h, w*3, QImage::Format_RGB888));

    std::cout << "pixmap width: " << pixmap.width() << "\n";
    
    
    Q_EMIT imageUpdated();
}

