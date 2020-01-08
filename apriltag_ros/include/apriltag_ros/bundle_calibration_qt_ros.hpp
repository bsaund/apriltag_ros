#ifndef APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_
#define APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QPixmap>
#include "apriltag_ros/common_functions.h"



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace apriltag_ros {

/*****************************************************************************
 ** Class
 *****************************************************************************/

    class QNode : public QThread {
        Q_OBJECT
    public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
        QPixmap& getPixmap();


    Q_SIGNALS:
        void imageUpdated();
        void rosShutdown();

    protected:
        std::unique_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber camera_image_subscriber_;
        std::unique_ptr<TagDetector> tag_detector_;
        cv_bridge::CvImagePtr cv_image_;
        QPixmap pixmap;

    protected:
        void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                           const sensor_msgs::CameraInfoConstPtr& camera_info);


    private:
        int init_argc;
        char** init_argv;
    };

}  // namespace apriltag_ros

#endif /* APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_ */
