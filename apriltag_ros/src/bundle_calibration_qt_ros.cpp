#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <cmath>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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
 ** QNode
 *****************************************************************************/


QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv),
    tf_listener(tf_buffer)
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
    tag_detections_publisher_ =
        nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

    
    auto tag_bundle_descriptions = getTagBundleDescriptions();
    for (unsigned int j=0; j<tag_bundle_descriptions.size(); j++)
    {
        TagBundleDescription bundle = tag_bundle_descriptions[j];
        for(const int tag_id: bundle.bundleIds())
        {
            tag_sizes[tag_id] = bundle.memberSize(tag_id);
        }
    }

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


bool QNode::tooSimilarToPrevious(const calibration_snapshot &cur) const
{
    if(calibration_data.size() == 0)
    {
        return false;
    }

    const calibration_snapshot &prev = calibration_data.back();

    
    if(prev.tags.size() != cur.tags.size())
    {
        return false;
    }

    for(int i=0; i<prev.tags.size(); i++)
    {
        if(prev.tags[i].id != cur.tags[i].id)
        {
            return false;
        }

        // std::cout << "\n";
        for(int j=0; j<4; j++)
        {
            if(std::abs(cur.tags[i].im_corners[j][0] - prev.tags[i].im_corners[j][0]) >
               PIXEL_MOTION_THRESHOLD_FOR_NEW_CALIBRATION_POINT ||
               std::abs(cur.tags[i].im_corners[j][1] - prev.tags[i].im_corners[j][1]) >
               PIXEL_MOTION_THRESHOLD_FOR_NEW_CALIBRATION_POINT)
            {
                return false;
            }
        }
    }
    return true;
}

void QNode::publishBundleTagTransforms(const sensor_msgs::CameraInfoConstPtr& camera_info,
                                       const std_msgs::Header& header,
                                       std::string camera_frame_name)
{
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(camera_info);

    AprilTagDetectionArray tag_detection_array;
    std::map<int, AprilTagDetection> tag_detections;




    const zarray_t* detections = tag_detector_->getDetections();
    std::map<int, geometry_msgs::TransformStamped> transforms;
    for(int i=0; i<zarray_size(detections); i++)
    {
        // Get the i-th detected tag
        apriltag_detection_t *detection;
        zarray_get(detections, i, &detection);
        int tag_id = detection->id;


        if(tag_sizes.count(tag_id == 0)) // Don't add tags for which we do not know size
        {
            continue;
        }


        AprilTagDetection solution = tag_detector_->solveTagTransform(tag_sizes[tag_id],
                                                                      detection, camera_model, header);

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = camera_frame_name;
        tf_msg.child_frame_id = "tag_" + std::to_string(tag_id);
        
        tf_msg.transform.translation.x = solution.pose.pose.pose.position.x;
        tf_msg.transform.translation.y = solution.pose.pose.pose.position.y;
        tf_msg.transform.translation.z = solution.pose.pose.pose.position.z;
        tf_msg.transform.rotation.x = solution.pose.pose.pose.orientation.x;
        tf_msg.transform.rotation.y = solution.pose.pose.pose.orientation.y;
        tf_msg.transform.rotation.z = solution.pose.pose.pose.orientation.z;
        tf_msg.transform.rotation.w = solution.pose.pose.pose.orientation.w;
        transforms[tag_id] = tf_msg;
    }

    if(transforms.size() == 0)
    {
        return;
    }


    /*
     *  Set up tf tree such that a tag is always the parent of the camera, 
     *  and lower id tags are direct parents of larger id tags
     *  
     *  This ensure the TF tree spans all tags and camera snapshots, 
     *  assuming sufficient calibration is gathered.
     *
     *  Note: If the camera were the parent, as in continous_detector, the TF tree
     *  will not chain snapshots together (so all tags would have to be viewed in a single frame)
     */
    std::map<int, geometry_msgs::TransformStamped>::iterator it = transforms.begin();
    geometry_msgs::TransformStamped base_tf = it->second;
    tf2::Transform tmp;
    tf2::fromMsg(base_tf.transform, tmp);
    base_tf.transform = tf2::toMsg(tmp.inverse());  //transform from lowest id tag to camera
    auto camera_name = base_tf.header.frame_id;
    base_tf.header.frame_id = base_tf.child_frame_id;
    base_tf.child_frame_id = camera_name;
    tf_br.sendTransform(base_tf);

    it++;
    while(it != transforms.end())
    {
        tf2::doTransform(it->second, it->second, base_tf);
        it->second.header.frame_id = base_tf.header.frame_id;
        tf_br.sendTransform(it->second);
        it++;
    }

    
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

    // tag_detections_publisher_.publish(tag_detector_->detectTags(cv_image_,camera_info));
    // tag_detector_->detectTags(cv_image_,camera_info);
    tag_detector_->updateDetections(cv_image_);

    publishBundleTagTransforms(camera_info, image_rect->header, "camera");

    calibration_snapshot detections(tag_detector_->getDetections(), tag_sizes);


    if(detections.tags.size() > 1 && 
       !tooSimilarToPrevious(detections))
    {
        detections.camera_name = "camera_" + std::to_string(calibration_data.size());
        detections.camera_info = camera_info;
        publishBundleTagTransforms(camera_info, image_rect->header,
                                   detections.camera_name);

        calibration_data.push_back(detections);
    }
    
    std::vector<zarray_t> calibration_data;

    visible_tags.clear();
    for(const tag_correspondence &tag: detections.tags)
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
std::vector<calibration_snapshot> QNode::cleanCalibrationData(std::set<int> tags_to_calibrate) const
{
    std::vector<calibration_snapshot> cleaned_calibration_data;

    for(const calibration_snapshot &original_datum: calibration_data)
    {
        calibration_snapshot cleaned_datum;
        cleaned_datum.camera_name = original_datum.camera_name;
        cleaned_datum.camera_info = original_datum.camera_info;
        for(const tag_correspondence &orig_tag: original_datum.tags)
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

    std::map<int, raw_pose> tag_poses;
    std::map<std::string, raw_pose> camera_poses;

    tag_poses[tag_ids[0]] = raw_pose();

    std::cout << "Initial tag poses: \n";
    std::string master_tag = "tag_" + std::to_string(tag_ids[0]);

    for(int i=1; i<tag_ids.size(); i++)
    {
        geometry_msgs::TransformStamped transform;
        std::string this_tag = "tag_" + std::to_string(tag_ids[i]);
        transform = tf_buffer.lookupTransform(master_tag, this_tag, ros::Time(0));
        tag_poses[tag_ids[i]] = raw_pose(transform);
        std::cout << "tag " << tag_ids[i] << ": " << tag_poses[tag_ids[i]] << "\n";
    }

    for(const auto &datum: data)
    {
        geometry_msgs::TransformStamped transform;

        transform = tf_buffer.lookupTransform(datum.camera_name, master_tag, ros::Time(0));
        camera_poses[datum.camera_name] = raw_pose(transform);
        std::cout << datum.camera_name << ": " << camera_poses[datum.camera_name] << "\n";
        
    }

    std::cout << "Calibration data size: " << data.size() << "\n";

    std::cout << "Calibrating bundle with tag ids: ";    
    for(int t: tag_ids)
    {
        std::cout << t << ", ";
    }
    std::cout << "\n";

    CeresBundleSolver s;
    s.solve(data, tag_poses, camera_poses, tag_ids[0]);

    for(const auto &id: tag_ids)
    {
        geometry_msgs::TransformStamped tag_msg;
        tag_msg.transform = tag_poses[id].toRosTransformMsg();
        tag_msg.header.frame_id = "tag_1";
        tag_msg.child_frame_id = "tag_" + std::to_string(id) + "_calibrated";
        tf_br.sendTransform(tag_msg);
    }
    writeBundle(getTagBundleDescriptions()[bundle_id-1], tag_poses);
}


void QNode::writeBundle(const TagBundleDescription &bundle, std::map<int, raw_pose> tag_poses) const
{
    std::cout <<
        "\n\nCOPY THIS INTO THE 'tags.yaml' FILE\n\n\n" <<
        "    {\n" <<
        "      name: '" << bundle.name() << "',\n" <<
        "      layout:\n" <<
        "        [\n";
    for(int id: bundle.bundleIds())
    {
        std::cout <<
        "          {id: " << id << ", size: " << bundle.memberSize(id) << ", " << tag_poses[id] << "},\n";
    }
    std::cout <<
        "        ]\n" <<
        "    },\n\n\n";

}
