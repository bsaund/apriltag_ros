#ifndef APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_
#define APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
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
	// bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
         ** Logging
         **********************/
	// enum LogLevel {
        //     Debug,
        //     Info,
        //     Warn,
        //     Error,
        //     Fatal
        // };

	// QStringListModel* loggingModel() { return &logging_model; }
	// void log( const LogLevel &level, const std::string &msg);
        void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                           const sensor_msgs::CameraInfoConstPtr& camera_info);


    Q_SIGNALS:
	// void loggingUpdated();
        void rosShutdown();

    protected:
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber camera_image_subscriber_;


    // private:
        int init_argc;
        char** init_argv;
    //     ros::Publisher chatter_publisher;
    //     QStringListModel logging_model;
            

    };

}  // namespace apriltag_ros

#endif /* APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_ */
