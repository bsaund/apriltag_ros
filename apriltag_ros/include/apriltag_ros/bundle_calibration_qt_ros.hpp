#ifndef APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_
#define APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


#include <string>
#include <iostream>
#include <QThread>
#include <QStringListModel>
#include <QPixmap>
#include "apriltag_ros/common_functions.h"
#include "common/homography.h"
#include "apriltag_ros/calibration_types.h"



#define PIXEL_MOTION_THRESHOLD_FOR_NEW_CALIBRATION_POINT 30.0


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace apriltag_ros {

/*****************************************************************************
 ** Class
 *****************************************************************************/
    /*
     *
     *
     */
    

    class QNode : public QThread {
        Q_OBJECT
    public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
        QPixmap& getPixmap();
        std::set<int> visible_tags;

        std::vector<calibration_datum> calibration_data;

    public:
        const std::vector<TagBundleDescription>& getTagBundleDescriptions()
        {
            return tag_detector_->getTagBundleDescriptions();
        }
        std::vector<calibration_datum> cleanCalibrationData(std::set<int> tags_to_calibrate) const;
        void calibrateBundle(int bundle_id);



    Q_SIGNALS:
        void imageUpdated();
        void newTagObserved(int id);
        void rosShutdown();

    protected:
        int init_argc;
        char** init_argv;

        std::unique_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber camera_image_subscriber_;
        std::unique_ptr<TagDetector> tag_detector_;
        ros::Publisher tag_detections_publisher_;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
        cv_bridge::CvImagePtr cv_image_;
        QPixmap pixmap;
        std::set<int> observed_tags;
        tf2_ros::TransformBroadcaster tf_br;

    protected:
        void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                           const sensor_msgs::CameraInfoConstPtr& camera_info);
        void publishBundleTagTransforms(const sensor_msgs::CameraInfoConstPtr& camera_info,
                                        const std_msgs::Header& header,
                                        std::string camera_frame_name="camera");
        
        void addToObservedSet(int id);
        bool tooSimilarToPrevious(const calibration_datum &cur) const;
            

    };

}  // namespace apriltag_ros

#endif /* APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_ */
